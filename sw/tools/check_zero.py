"""영점 반복 정밀도 검증 — 딴 데 갔다가 돌아와도 같은 자세인가.

무엇을 검증하는가
-----------------
영점을 박은 뒤 **정말 그 자세로 돌아오는가**를 본다. 두 가지가 한꺼번에 걸린다.

  1. **영점이 제대로 박혔는가**  — `q=[0,0,0]` 에서 팔이 일직선인가
  2. **반복 정밀도**            — 여러 번 왕복해도 같은 자리로 오는가
     (엔코더 미끄러짐, 감속기 백래시, 탈조 등이 여기서 드러난다)

왜 컨트롤러 숫자만으로는 부족한가
---------------------------------
컨트롤러는 위치 폐루프라 **언제나** `q = [0, 0, 0]` 을 보고한다.
그건 "지령대로 갔다" 는 뜻이지 "물리적으로 그 자세다" 는 뜻이 아니다.
**사람 눈이 유일한 독립 측정기다.** 그래서 매번 물어본다.

실제로 겪은 예: 서보를 끈 뒤 영점을 박는 바람에 팔이 흘러내린 자세가 0 으로
박혔다. 그런데 검사는 항상 0.000deg 로 통과했다. 같은 좌표계 안에서 스스로를
검사하면 이런 오류는 절대 안 잡힌다.

측정하는 것
-----------
  - 왕복 후 컨트롤러가 보고하는 `q` (참고용)
  - 왕복 후 TCP 좌표 (참고용)
  - **팔이 일직선인가** (사람 판단 — 이게 본질)
  - **J1 축 ~ TCP 직선거리** (선택. 자로 재면 정량 확인이 된다)

    일직선이면 650.0 mm.  J2 가 굽은 만큼 짧아진다:
        10도 -> 647.7    20도 -> 640.7    30도 -> 629.1    45도 -> 603.3

팔이 650 mm 로 펴진다. 반경 650 mm 를 비울 것.
아무것도 저장하지 않는다. 영점을 건드리지 않는다. 이동과 조회만 한다.

사용:
    python sw/tools/check_zero.py --plan     # 계획만
    python sw/tools/check_zero.py
    python sw/tools/check_zero.py --rounds 3
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

ZERO = [0.0, 0.0, 0.0]
SPEED_FRACTION = 0.12

#: 왕복에 쓸 임의 자세들. 세 축을 모두 크게 움직이면서 가동범위에 여유가 있다.
AWAY_POSES = [
    [30.0, -40.0, 20.0],
    [-25.0, 50.0, -30.0],
    [60.0, -70.0, 45.0],
]


def straight_distance(bend2_deg, bend3_deg=0.0) -> float:
    x, y, _, _ = kin.fk([0.0, bend2_deg * D, bend3_deg * D])
    return math.hypot(x, y) * 1000.0


def show(bot, label=""):
    q = bot.joint_positions()
    p = bot.pose()
    r = math.hypot(p[0], p[1]) * 1000
    print(f"  {label:10s} q=[{q[0]*R:+8.3f},{q[1]*R:+8.3f},{q[2]*R:+8.3f}]deg  "
          f"TCP=({p[0]*1000:7.2f},{p[1]*1000:7.2f})  반경 {r:7.2f} mm")
    return q, p


def goto(bot, q_deg, label) -> bool:
    q = [v * D for v in q_deg]
    try:
        safety.check_joint_limits(q)
    except safety.SafetyViolation as exc:
        print(f"  [거부] {exc}")
        return False

    q_now = bot.joint_positions()
    dq = [(a - b) * R for a, b in zip(q, q_now)]
    travel = max(abs(v) for v in dq)
    if travel < 0.02:
        print(f"  [{label}] 이미 그 자세다. 이동 생략.")
        return True
    print(f"  [{label}] -> q={q_deg}   최대 이동 {travel:.2f}deg")

    op = bot.status().get("operation_status", {})
    if not op.get("manual_mode"):
        bot.set_operational_mode("MANUAL")
    bot.servo(True)
    vel = min(safety.rated_joint_speeds()) * SPEED_FRACTION
    try:
        bot.move_joint([{"q": list(q), "vel": vel, "acc": vel * 2, "radius": 0.0}])
        bot.wait_until_done(timeout_s=90.0)
    except KeyboardInterrupt:
        bot.stop(); raise
    except Exception as exc:
        print(f"  [오류] {type(exc).__name__}: {exc}")
        bot.stop()
        return False
    return True


def ask_yn(text) -> bool:
    while True:
        try:
            a = input(f"    {text}  [y/n] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise KeyboardInterrupt
        if a.startswith("y"):
            return True
        if a.startswith("n"):
            return False
        print("      y 또는 n")


def ask_mm(text):
    """자로 잰 값. 그냥 Enter 면 건너뛴다."""
    try:
        a = input(f"    {text} (모르면 Enter) > ").strip()
    except (EOFError, KeyboardInterrupt):
        raise KeyboardInterrupt
    if not a:
        return None
    try:
        return float(a)
    except ValueError:
        print("      숫자가 아니다. 건너뛴다.")
        return None


def judge_distance(mm):
    """잰 거리로 굽음을 역산한다."""
    if mm is None:
        return
    if mm > 651.0:
        print(f"      -> {mm:.1f} mm 는 최대 도달(650.0)보다 크다. 재는 기준점을 확인할 것.")
        return
    # 650 에서 얼마나 줄었는지로 J2 굽음을 역산 (J3=0 가정)
    lo, hi = 0.0, 90.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if straight_distance(mid) > mm:
            lo = mid
        else:
            hi = mid
    bend = (lo + hi) / 2
    if bend < 2.0:
        print(f"      -> {mm:.1f} mm  ≈ 650.  **일직선으로 볼 수 있다** (굽음 {bend:.1f}도 이하)")
    else:
        print(f"      -> {mm:.1f} mm.  J2 가 약 **{bend:.1f}도** 굽어 있다는 뜻이다 (J3=0 가정)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="계획만 출력, 움직이지 않는다")
    ap.add_argument("--rounds", type=int, default=len(AWAY_POSES),
                    help=f"왕복 횟수 (기본 {len(AWAY_POSES)})")
    args = ap.parse_args()

    rounds = max(1, min(args.rounds, len(AWAY_POSES)))

    print("=" * 74)
    print("영점 반복 정밀도 검증")
    print("=" * 74)
    print("  절차:  [0,0,0] 확인  ->  임의 자세로 이동  ->  [0,0,0] 복귀  ->  다시 확인")
    print(f"  왕복 {rounds}회.  임의 자세: " + ", ".join(str(p) for p in AWAY_POSES[:rounds]))
    print()
    print("  참고 — J1축 ~ TCP 직선거리로 본 굽음 정도:")
    for b in (0, 5, 10, 20, 30, 45):
        print(f"     J2 {b:2d}도 굽음 -> {straight_distance(b):6.1f} mm")

    if args.plan:
        print()
        print("[계획 전용] --plan 을 빼면 실제로 진행한다.")
        return 0

    bot = Scara()
    problems = safety.startup_checklist(bot)
    if problems:
        print("[기동 전 점검 실패]")
        for p in problems:
            print("  -", p)
        bot.close()
        return 1

    print()
    print("반경 650 mm 를 비울 것.")
    try:
        if input("준비되면 Enter (취소 n) > ").strip().lower().startswith("n"):
            bot.close(); return 130
    except (EOFError, KeyboardInterrupt):
        bot.close(); return 130

    results = []
    try:
        # --- 기준선 ---
        print()
        print("-" * 74)
        print("기준선 — 영점 자세 확인")
        print("-" * 74)
        show(bot, "현재")
        if not goto(bot, ZERO, "영점"):
            return 1
        show(bot, "-> 도착")
        print()
        base_straight = ask_yn("팔 세 마디가 **한 직선**으로 펴져 있는가?")
        base_mm = ask_mm("J1축 중심 ~ TCP 직선거리 [mm]")
        judge_distance(base_mm)
        if not base_straight:
            print()
            print("  ** 영점부터 안 맞는다. 반복 정밀도를 볼 단계가 아니다. **")
            print("     main.py 의 영점 단계로 다시 정렬할 것.")
            return 1
        print("    -> 기준선 확보. 이 자세를 기억할 것. 가능하면 TCP 밑에 표시를 해둘 것.")

        # --- 왕복 ---
        for i in range(rounds):
            away = AWAY_POSES[i]
            print()
            print("-" * 74)
            print(f"{i+1}회차 — {away} 로 갔다가 영점 복귀")
            print("-" * 74)
            if not goto(bot, away, f"{i+1}회차 이탈"):
                results.append(None); continue
            show(bot, "-> 이탈")
            time.sleep(0.5)
            if not goto(bot, ZERO, f"{i+1}회차 복귀"):
                results.append(None); continue
            q, p = show(bot, "-> 복귀")

            err_deg = max(abs(v) * R for v in q)
            print(f"  컨트롤러 복귀 오차 {err_deg:.4f} deg "
                  f"(폐루프라 항상 작다. 참고용일 뿐이다)")
            print()
            straight = ask_yn("팔이 **여전히 한 직선**인가?")
            same = ask_yn("**기준선 때와 같은 자리**로 돌아왔는가? (표시해 둔 위치)")
            mm = ask_mm("J1축 중심 ~ TCP 직선거리 [mm]")
            judge_distance(mm)
            results.append((straight, same, mm))
            print("    -> " + ("통과" if (straight and same) else "** 문제 있음 **"))

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        print()
        print("=" * 74)
        print("결과")
        print("=" * 74)
        print(f"  기준선: 일직선={'예' if base_straight else '아니오'}"
              + (f", 실측 {base_mm:.1f} mm" if base_mm else ""))
        allok = True
        for i, r in enumerate(results):
            if r is None:
                print(f"  {i+1}회차: 건너뜀"); allok = False; continue
            st, sm, mm = r
            print(f"  {i+1}회차: 일직선={'예' if st else '아니오'}  "
                  f"같은자리={'예' if sm else '아니오'}"
                  + (f"  실측 {mm:.1f} mm" if mm else ""))
            if not (st and sm):
                allok = False
        print()
        if results and allok:
            print("  ** 통과 — 영점이 제대로 박혔고 왕복해도 같은 자세로 돌아온다. **")
            print("     다음: tools/verify_model.py 로 각 축의 방향·크기를 확인할 것.")
        else:
            print("  ** 문제 있음. ** 위 항목을 그대로 붙여줄 것.")
            print("     일직선이 깨지면 -> 영점 문제 (main.py 영점 단계)")
            print("     자리가 다르면   -> 반복 정밀도 문제 (백래시·미끄러짐·탈조)")
        try:
            bot.stop(); time.sleep(0.3); bot.servo(False)
        except Exception:
            pass
        bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
