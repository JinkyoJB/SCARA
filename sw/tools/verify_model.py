"""모델 ↔ 실물 매핑을 **관절 하나씩 격리해서** 검증한다.

왜 필요한가
-----------
세 관절이 섞인 자세(IK 결과)만 보면 어느 축이 어긋났는지 알 수 없다.
(500, 0) 이 엉뚱한 곳으로 가는데 원인을 못 좁힌 적이 있다.
**한 번에 한 축만 움직이면 그 축의 방향·크기·영점이 한꺼번에 드러난다.**

검증하는 것
-----------
각 단계에서 사람이 눈으로 보고 y/n 만 답한다.

  0단계  q = [0, 0, 0]     -> 팔이 일직선인가?  +x 기준과 맞는가?   (영점 + 기준)
  1단계  q = [60, 0, 0]    -> 팔 전체가 왼쪽 60도 로 돌았는가?      (J1 방향·크기)
  2단계  q = [0, 60, 0]    -> 링크1 은 그대로, 링크2·3 만 꺾였는가? (J2 방향·크기)
  3단계  q = [0, 0, 60]    -> 링크1·2 는 일직선, 링크3 만 꺾였는가? (J3 방향·크기)

부호 약속:
**위에서 내려다볼 때 반시계(CCW) = +**,  `+` 는 앞 링크 기준 **왼쪽**으로 꺾이는 방향.

팔이 최대 650 mm 로 펴진다. 반경 650 mm 를 비울 것.
각 단계 전에 Enter 를 받는다. 아무것도 저장하지 않는다 (읽기 + 이동만).

사용:
    python sw/tools/verify_model.py --plan     # 계획만, 움직이지 않는다
    python sw/tools/verify_model.py
"""

import argparse
import logging
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import console  # noqa: E402
console.setup()

from scara import config, safety                    # noqa: E402
from scara import kinematics as kin                 # noqa: E402
from scara.client import Scara                      # noqa: E402

logging.basicConfig(level=logging.ERROR, format="%(levelname)-7s %(message)s")
D = math.pi / 180.0
R = 180.0 / math.pi

TEST_DEG = 60.0          # 눈으로 확실히 보이면서 가동범위에 여유가 큰 각
SPEED_FRACTION = 0.10


def expect(q_deg):
    """이 관절각에서 각 링크 끝이 어디에 있어야 하는지."""
    q = [v * D for v in q_deg]
    pts = kin.joint_positions(q)
    x, y, _, phi = kin.fk(q)
    return pts, (x * 1000, y * 1000), phi * R


STEPS = [
    (
        [0.0, 0.0, 0.0],
        "0단계 — 영점 자세",
        [
            "팔 세 마디가 **한 직선**으로 펴져 있는가?",
            "그 직선이 **베이스 판 앞모서리와 직각**인가? (= +x 기준)",
        ],
        "아니면: 영점 또는 J1 기준이 깨진 것. 06/08 스크립트로 재설정해야 한다.",
    ),
    (
        [TEST_DEG, 0.0, 0.0],
        f"1단계 — J1 만 +{TEST_DEG:.0f}도",
        [
            f"팔이 **여전히 일직선**인 채로 통째로 돌았는가?",
            f"**왼쪽(반시계)** 으로 돌았는가?  (오른쪽이면 J1 방향이 반대)",
            f"돈 각도가 눈대중으로 **{TEST_DEG:.0f}도** 쯤 되는가? (많이 다르면 기어비 문제)",
        ],
        "아니면: J1 의 방향 또는 기어비 문제.",
    ),
    (
        [0.0, TEST_DEG, 0.0],
        f"2단계 — J2 만 +{TEST_DEG:.0f}도",
        [
            "**링크1 은 +x 방향 그대로** 있는가?",
            f"링크2·3 이 링크1 기준 **왼쪽으로** 꺾였는가? (오른쪽이면 J2 반대)",
            f"꺾인 각도가 **{TEST_DEG:.0f}도** 쯤 되는가?",
        ],
        "아니면: J2 의 방향 또는 기어비 문제.",
    ),
    (
        [0.0, 0.0, TEST_DEG],
        f"3단계 — J3 만 +{TEST_DEG:.0f}도",
        [
            "**링크1 과 링크2 가 +x 로 일직선** 인가?",
            f"링크3 만 **왼쪽으로** 꺾였는가? (오른쪽이면 J3 반대)",
            f"꺾인 각도가 **{TEST_DEG:.0f}도** 쯤 되는가?",
        ],
        "아니면: J3 의 방향 또는 기어비 문제.",
    ),
]


def show_expected(q_deg):
    pts, tcp, phi = expect(q_deg)
    names = ["베이스(J1축)", "J2축(팔꿈치)", "J3축(손목)", "TCP"]
    print("  모델이 예측하는 위치 [mm]:")
    for name, (x, y) in zip(names, pts):
        ang = math.degrees(math.atan2(y, x)) if (x or y) else 0.0
        print(f"     {name:14s} ({x*1000:7.1f}, {y*1000:7.1f})"
              + (f"   원점에서 {ang:+6.1f}도" if (x or y) else ""))
    print(f"     TCP 반경 {math.hypot(*tcp):.1f} mm,  원점에서 "
          f"{math.degrees(math.atan2(tcp[1], tcp[0])):+.1f}도,  phi={phi:+.1f}도")


def goto(bot, q_deg) -> bool:
    q = [v * D for v in q_deg]
    try:
        safety.check_joint_limits(q)
    except safety.SafetyViolation as exc:
        print(f"  [거부] {exc}")
        return False

    q_now = bot.joint_positions()
    dq = [(a - b) * R for a, b in zip(q, q_now)]
    print(f"  이동량  J1 {dq[0]:+.2f}   J2 {dq[1]:+.2f}   J3 {dq[2]:+.2f} deg  "
          f"(최대 {max(abs(v) for v in dq):.2f})")
    if max(abs(v) for v in dq) < 0.05:
        print("  이미 그 자세다. 이동 생략.")
        return True

    try:
        ans = input("  움직인다. Enter 진행 / n 건너뜀 / q 종료 > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        return False
    if ans.startswith("q"):
        raise KeyboardInterrupt
    if ans.startswith("n"):
        print("  건너뜀")
        return False

    op = bot.status().get("operation_status", {})
    if not op.get("manual_mode"):
        print("  [모드] AUTO -> MANUAL")
        bot.set_operational_mode("MANUAL")
    bot.servo(True)

    vel = min(safety.rated_joint_speeds()) * SPEED_FRACTION
    print(f"  이동 중 ({vel*R:.1f} deg/s) ...", end="", flush=True)
    try:
        bot.move_joint([{"q": list(q), "vel": vel, "acc": vel * 2, "radius": 0.0}])
        bot.wait_until_done(timeout_s=60.0)
    except KeyboardInterrupt:
        print()
        bot.stop()
        raise
    except Exception as exc:
        print(f"\n  [오류] {type(exc).__name__}: {exc}")
        bot.stop()
        return False
    print(" 완료")

    q_a = bot.joint_positions()
    p = bot.pose()
    print(f"  실제 관절각  J1={q_a[0]*R:+8.3f}  J2={q_a[1]*R:+8.3f}  J3={q_a[2]*R:+8.3f} deg")
    print(f"  컨트롤러 TCP ({p[0]*1000:7.2f}, {p[1]*1000:7.2f}) mm  "
          f"원점에서 {math.degrees(math.atan2(p[1], p[0])):+.1f}도")
    return True


def ask(questions) -> list:
    out = []
    for qtext in questions:
        while True:
            try:
                a = input(f"    {qtext}  [y/n] > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                raise KeyboardInterrupt
            if a.startswith("y"):
                out.append(True); break
            if a.startswith("n"):
                out.append(False); break
            print("      y 또는 n 으로 답할 것")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="계획만 출력, 움직이지 않는다")
    args = ap.parse_args()

    bot = Scara()
    problems = safety.startup_checklist(bot)
    if problems:
        print("[기동 전 점검 실패]")
        for p in problems:
            print("  -", p)
        bot.close()
        return 1

    print("=" * 72)
    print("모델 <-> 실물 매핑 검증  (관절 하나씩 격리)")
    print("=" * 72)
    print("부호 약속: **위에서 내려다볼 때 반시계(CCW) = +**")
    print("           + 는 앞 링크 기준 **왼쪽**으로 꺾이는 방향")
    print()
    print(f"현재 DIRECTION_SIGN = {config.DIRECTION_SIGN}")
    q = bot.joint_positions()
    print(f"현재 관절각  J1={q[0]*R:+8.3f}  J2={q[1]*R:+8.3f}  J3={q[2]*R:+8.3f} deg")
    print()
    print("팔이 최대 650 mm 로 펴진다. 반경 650 mm 를 비울 것.")

    if args.plan:
        for q_deg, title, _, _ in STEPS:
            print()
            print("-" * 72)
            print(f"{title}   q = {q_deg}")
            print("-" * 72)
            show_expected(q_deg)
        print()
        print("[계획 전용] --plan 을 빼면 실제로 진행한다.")
        bot.close()
        return 0

    results = {}
    try:
        for q_deg, title, questions, hint in STEPS:
            print()
            print("=" * 72)
            print(f"{title}   q = {q_deg}")
            print("=" * 72)
            show_expected(q_deg)
            print()
            if not goto(bot, q_deg):
                results[title] = None
                continue
            print()
            print("  **눈으로 확인** (위에서 내려다볼 것):")
            ans = ask(questions)
            results[title] = ans
            if all(ans):
                print("    -> 통과")
            else:
                print(f"    -> ** 문제 있음. ** {hint}")
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        print()
        print("=" * 72)
        print("결과 요약")
        print("=" * 72)
        for q_deg, title, questions, hint in STEPS:
            r = results.get(title)
            if r is None:
                print(f"  {title:28s}  건너뜀")
            elif all(r):
                print(f"  {title:28s}  통과")
            else:
                bad = [questions[i] for i, v in enumerate(r) if not v]
                print(f"  {title:28s}  ** 실패 **")
                for b in bad:
                    print(f"      - {b}")
        print()
        print("이 요약을 그대로 붙여주면 원인을 좁힐 수 있다.")
        try:
            bot.stop()
            time.sleep(0.3)
            bot.servo(False)
        except Exception:
            pass
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
