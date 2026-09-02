"""좌표 입력식 대화형 제어 CLI. UI 없이 디버깅할 때 쓴다.

x, y [mm] 를 주면 IK 를 풀어 그 위치로 간다. 이동 전에 항상 계획을 보여주고
Enter 를 받는다.

실행 전제 — 이 순서를 안 지키면 지령이 먹지 않는다 (docs/controller.md):
    1. 드라이버 제어 전원 L1C, L2C
    2. 드라이버 주회로 전원 L1, L3     <- 빠뜨리면 통신은 되는데 모터만 안 돈다
    3. StellaX 전원
    4. 브라우저에서 Stella Designer 연결
    5. Step 4: Verification 까지 진행   <- EtherCAT OP 진입

사용:
    python sw/tools/move_cli.py                          # 대화형
    python sw/tools/move_cli.py --x 500 --y 100          # 계획만 (dry-run)
    python sw/tools/move_cli.py --x 500 --y 100 --execute
"""

import argparse
import logging
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import console  # noqa: E402
console.setup()

from scara import config, motion, safety            # noqa: E402
from scara import kinematics as kin                 # noqa: E402
from scara.robot import SCARA                       # noqa: E402
from scara.safety import SafetyViolation            # noqa: E402

logging.basicConfig(level=logging.WARNING, format="%(levelname)-7s %(message)s")
D = math.pi / 180.0
R = 180.0 / math.pi

#: 컨트롤러 조회값 — 관절 가속 한계 [rad/s^2]
ACC_LIMIT_RAD_S2 = 4.1655


def qdeg(q):
    return "[" + ", ".join(f"{a*R:+7.2f}" for a in q) + "]deg"


# =============================================================================
# 표시
# =============================================================================

def show_state(bot: SCARA):
    st = bot.state()
    print(f"  관절   {st['q_deg']} deg")
    print(f"  TCP    ({st['tcp_mm'][0]:.2f}, {st['tcp_mm'][1]:.2f}) mm   "
          f"phi {st['phi_deg']:+.2f} deg")
    print(f"  틈새   {st['clearance_mm']:.1f} mm  ({st['clearance_pair'][0]} <-> "
          f"{st['clearance_pair'][1]})")
    print(f"  서보   {'ON' if st['servo_on'] else 'OFF'}")


def show_profile(speed_fraction, acc_ratio):
    """지금 설정이 실제로 어떤 속도·가속이 되는지 환산해서 보여준다.

    비율만 봐서는 감이 안 오므로 deg/s 와 한계 대비 %, 그리고 공진 여기율까지
    같이 찍는다.
    """
    vel = min(safety.rated_joint_speeds()) * speed_fraction
    acc = vel * acc_ratio
    vel_limit = min(safety.max_joint_speeds())
    print(f"  프로파일  속도 {vel*R:6.1f} deg/s ({vel/vel_limit:4.0%} of 최대)   "
          f"가속 {acc*R:6.1f} deg/s^2 ({acc/ACC_LIMIT_RAD_S2:4.0%} of 한계)   "
          f"가속시간 {1.0/acc_ratio:.2f} s")

    f = config.RESONANCE_HZ
    n = f / acc_ratio
    exc = abs(math.sin(math.pi * n))
    tag = ("공진 여기 없음" if exc < 0.15 else
           "공진을 조금 때림" if exc < 0.5 else "** 공진을 세게 때린다 **")
    print(f"  공진       1차 비틀림 {f:.2f} Hz (주기 {1/f:.3f} s). "
          f"가속시간 {1/acc_ratio:.3f} s = {n:.2f} 주기 -> 여기율 {exc:.2f}  {tag}")
    if exc >= 0.15:
        print(f"             여기율 0 이 되는 acc 배수: "
              f"{[round(f / k, 3) for k in (1, 2, 3)]}")

    print(f"  TCP 한계   작업공간 {motion.TCP_VEL_LIMIT*1000:.0f} mm/s. 반경별 관절속도 상한:")
    for r_mm in (650, 550, 450, 350, 250):
        cap = motion.TCP_VEL_MARGIN * motion.TCP_VEL_LIMIT / (r_mm / 1000.0)
        flag = "  <- 지금 설정이 넘는다" if vel > cap else ""
        print(f"               반경 {r_mm:3d} mm -> {cap*R:5.1f} deg/s{flag}")
    print("             넘는 지령은 자동으로 잘라서 보낸다.")


def report_accuracy(bot: SCARA, x, y, phi):
    """지령 대비 실제 도달 위치. 가상/실물 정합성 실험의 원자료가 된다."""
    p = bot.pose()
    q = bot.q()
    ex, ey = (p[0] - x) * 1000, (p[1] - y) * 1000
    print(f"  도달       x={p[0]*1000:8.2f}  y={p[1]*1000:8.2f} mm   "
          f"phi={p[5]*R:+7.2f}deg")
    print(f"  오차       dx={ex:+.3f}  dy={ey:+.3f} mm  "
          f"(거리 {math.hypot(ex, ey):.3f} mm)  dphi={(p[5]-phi)*R:+.3f}deg")
    fx, fy, _, _ = kin.fk(q)
    print(f"  모델대조   FK 와 컨트롤러 pose 차이 "
          f"{math.hypot(fx - p[0], fy - p[1])*1000:.6f} mm")


# =============================================================================
# 계획 · 진단
# =============================================================================

def show_plan(bot: SCARA, x, y, phi, elbow, linear):
    """계획을 펼쳐 보인다. 통과하면 (q_target, phi) 를 돌려준다."""
    q_now = bot.q()
    print()
    print("-" * 70)
    print(f"목표   x={x*1000:.2f}  y={y*1000:.2f} mm   phi={phi*R:+.2f}deg   "
          f"({'move_linear 직선' if linear else 'move_joint 관절보간'})")
    print("-" * 70)
    q_t = motion.plan_joint_target(x, y, phi, seed_q=q_now, elbow=elbow)
    print(f"  현재 관절 {qdeg(q_now)}")
    print(f"  목표 관절 {qdeg(q_t)}")
    print(f"  이동량    {qdeg([a - b for a, b in zip(q_t, q_now)])}")
    print(f"  여유      한계까지 "
          f"{[round(v, 1) for v in motion.joint_margins_deg(q_t)]} deg")
    vel = min(safety.rated_joint_speeds()) * bot.speed_fraction
    cap = motion.tcp_speed_cap(q_now, q_t)
    if cap < vel:
        print(f"  속도상한  TCP 한계 때문에 {vel*R:.1f} -> {cap*R:.1f} deg/s 로 잘린다")
    return q_t


def suggest_auto(bot: SCARA, x, y, elbow):
    """지령이 거부됐을 때 phi 만 바꾸면 갈 수 있는지 알려준다.

    가장 흔한 실패 원인이 phi 상속이다. phi 를 직전 자세에서 물려받으면
    목표점이 J2 한계를 넘는 자세를 요구하게 되는 일이 잦다.
    격자점 135개 기준으로 phi 고정이면 42개, auto 면 96개가 도달 가능했다.
    """
    ph, _ = motion.auto_phi(x, y, bot.q(), elbow)
    if ph is not None:
        print(f"  'auto' 를 붙이면 갈 수 있다. phi={ph*R:+.1f}deg 로 풀린다")
        print(f"     {x*1000:.0f}, {y*1000:.0f}, auto   라고 입력할 것")
        return True
    print(f"  어떤 phi 로도 갈 수 없는 점이다 (반경 {math.hypot(x, y)*1000:.1f} mm).")
    print("  대략 반경 210~640 mm 안에서만 움직인다. 갈 수 있는 곳은")
    print("  python sw/scara/ui.py 로 지도를 보면 한눈에 나온다.")
    return False


def diagnose_no_motion(bot: SCARA, q_before):
    """지령은 받았는데 안 움직였을 때 원인을 좁힌다."""
    try:
        q_now = bot.q()
    except Exception:
        return
    moved = max(abs(a - b) for a, b in zip(q_now, q_before)) * R
    if moved > 0.05:
        print(f"  (참고) 최대 {moved:.3f}deg 움직였다 — 구동 자체는 되고 있었다")
        return

    print(f"  관절이 전혀 움직이지 않았다 (최대 {moved:.4f}deg). 상태를 읽는다.")
    try:
        st = bot.raw.status()
        op, sv, tr = st["operation_status"], st["servo_status"], st["trajectory_status"]
        print(f"    모드={'MANUAL' if op.get('manual_mode') else 'AUTO'}  "
              f"ready={op.get('ready')}  moving={op.get('moving')}  "
              f"paused={op.get('paused')}")
        print(f"    servo_on={sv.get('servo_on')}  pds={sv.get('servo_drive_coe_pds')[:3]}  "
              f"err={[hex(e) for e in sv.get('servo_drive_error')[:3]]}")
        tq = [round(math.degrees(v), 3) for v in tr.get("target_joint_position", [])[:3]]
        aq = [round(math.degrees(v), 3) for v in tr.get("actual_joint_position", [])[:3]]
        print(f"    target_q={tq}")
        print(f"    actual_q={aq}")
        if tq == aq:
            print("    -> 목표각이 현재각과 같다. 궤적이 아예 생성되지 않았다.")
        else:
            print("    -> 목표각은 갱신됐는데 실제각이 안 따라간다. "
                  "드라이버가 지령을 안 받는다 (주회로 전원 / EtherCAT OP 확인).")
    except Exception as exc:
        print(f"    상태 조회 실패: {exc}")


# =============================================================================
# 실행
# =============================================================================

def confirm(assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        return not input("\n  실행? [Enter=예 / n=아니오] > ").strip().lower().startswith("n")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def do_move(bot: SCARA, x, y, phi, elbow, linear, assume_yes=False) -> bool:
    """좌표 이동. phi=None 이면 자동 선택."""
    q_before = bot.q()
    if phi is None:
        phi, _ = motion.auto_phi(x, y, q_before, elbow)
        if phi is None:
            print("  갈 수 있는 phi 가 없다.")
            suggest_auto(bot, x, y, elbow)
            return False
        print(f"  phi 자동 선택 -> {phi*R:+.2f} deg")
    try:
        show_plan(bot, x, y, phi, elbow, linear)
    except SafetyViolation as exc:
        print(f"  [거부] {exc}")
        suggest_auto(bot, x, y, elbow)
        return False
    except kin.IKError as exc:
        print(f"  [거부] IK 실패: {exc}")
        suggest_auto(bot, x, y, elbow)
        return False

    if not confirm(assume_yes):
        print("  취소")
        return False

    try:
        if linear:
            bot.prepare()
            motion.move_linear_safe(bot.raw, x, y, phi)
        else:
            bot.move_xy(x * 1000, y * 1000, phi_deg=phi * R)
    except SafetyViolation as exc:
        print(f"  [거부] {exc}")
        return False
    except KeyboardInterrupt:
        print("\n  중단 — 정지시킨다.")
        bot.stop()
        return False
    except Exception as exc:
        print(f"  [실패] {type(exc).__name__}: {exc}")
        bot.stop()
        return False

    report_accuracy(bot, x, y, phi)
    diagnose_no_motion(bot, q_before)
    return True


def do_move_joint(bot: SCARA, q_deg_target, assume_yes=False) -> bool:
    """관절각 직접 지정. 특이점에서 빠져나오는 유일한 방법이다."""
    q_before = bot.q()
    print()
    print("-" * 70)
    print(f"목표 관절각 [{', '.join(f'{v:+.2f}' for v in q_deg_target)}]deg")
    print("-" * 70)
    try:
        safety.check_joint_limits([v * D for v in q_deg_target])
    except SafetyViolation as exc:
        print(f"  [거부] {exc}")
        return False
    if not confirm(assume_yes):
        print("  취소")
        return False
    try:
        bot.move_q_deg(q_deg_target)
    except SafetyViolation as exc:
        print(f"  [거부] {exc}")
        return False
    except KeyboardInterrupt:
        print("\n  중단 — 정지시킨다.")
        bot.stop()
        return False
    print(f"  도달 {qdeg(bot.q())}")
    diagnose_no_motion(bot, q_before)
    return True


# =============================================================================
# 입력 해석
# =============================================================================

HELP = """
  500, 100          TCP 를 x=500mm, y=100mm 로 (phi 는 현재값 유지)
  500, 100, 30      phi=30deg 까지 지정
  500, 100, auto    phi 를 알아서 고른다 — 한계·특이점·충돌에서 가장 먼 값
  r 50, 0           상대이동. 현재 TCP 에서 dx=+50mm
  j 0, 45, -45      관절각 직접 지정 [deg] (특이점 탈출용)
  home              일직선 자세 q=[0,0,0] 으로
  L                 다음 이동을 직선(move_linear)으로 토글. 기본 move_joint
  elbow up|down|both
  speed 0.15        관절속도 (정격 대비 비율). 0 < f <= 0.5
  acc 3.125         가속 배수. 가속시간 = 1/배수 초. 기본값이 공진 여기율 0
  prof              지금 속도·가속 설정이 실제로 어떤 값인지 보기
  s                 현재 상태
  h                 도움말
  q                 종료
"""


def parse(raw):
    """(kind, payload). kind: xy / rel / joint / cmd / None"""
    t = raw.strip().lower()
    if not t:
        return None, None
    for keys, name in ((("q", "quit", "exit"), "quit"),
                       (("h", "help", "?"), "help"),
                       (("s",), "status"),
                       (("l",), "linear"),
                       (("prof",), "prof"),
                       (("home",), "home")):
        if t in keys:
            return "cmd", (name, None)
    for prefix in ("elbow", "speed", "acc"):
        if t.startswith(prefix):
            return "cmd", (prefix, t[len(prefix):].strip())

    kind = "xy"
    if t.startswith("r"):
        kind, t = "rel", t[1:]
    elif t.startswith("j"):
        kind, t = "joint", t[1:]

    nums = [p for p in t.replace(",", " ").split() if p]
    auto = bool(nums) and nums[-1] in ("a", "auto")
    if auto:
        nums = nums[:-1]
    try:
        vals = [float(v) for v in nums]
    except ValueError:
        return None, None
    if kind == "joint":
        return ("joint", vals) if (len(vals) == 3 and not auto) else (None, None)
    if auto:
        return (kind, vals + ["auto"]) if len(vals) == 2 else (None, None)
    return (kind, vals) if len(vals) in (2, 3) else (None, None)


# =============================================================================

def interactive(bot: SCARA, elbow: str, linear: bool) -> int:
    print(HELP)
    show_profile(bot.speed_fraction, bot.acc_ratio)
    while True:
        try:
            raw = input(f"\n[{'직선' if linear else '관절'} elbow={elbow} "
                        f"speed={bot.speed_fraction:.2f} acc={bot.acc_ratio:.2f}] > ")
        except (EOFError, KeyboardInterrupt):
            break
        kind, payload = parse(raw)
        if kind is None:
            if raw.strip():
                print("  해석 불가. h 로 도움말.")
            continue

        if kind == "cmd":
            name, val = payload
            if name == "quit":
                break
            if name == "help":
                print(HELP)
            elif name == "status":
                show_state(bot)
            elif name == "prof":
                show_profile(bot.speed_fraction, bot.acc_ratio)
            elif name == "linear":
                linear = not linear
                print(f"  이동방식 -> "
                      f"{'move_linear (직선)' if linear else 'move_joint (관절보간)'}")
            elif name == "home":
                do_move_joint(bot, [0.0, 0.0, 0.0])
            elif name == "elbow":
                if val in ("up", "down", "both"):
                    elbow = val
                    print(f"  elbow -> {elbow}")
                else:
                    print("  up / down / both 중 하나")
            elif name == "speed":
                try:
                    v = float(val)
                except ValueError:
                    print("  숫자를 줄 것")
                    continue
                if 0 < v <= 0.5:
                    bot.speed_fraction = v
                    show_profile(bot.speed_fraction, bot.acc_ratio)
                else:
                    print("  0 < speed <= 0.5")
            elif name == "acc":
                try:
                    v = float(val)
                except ValueError:
                    print("  숫자를 줄 것")
                    continue
                if 0.5 <= v <= 8.0:
                    bot.acc_ratio = v
                    show_profile(bot.speed_fraction, bot.acc_ratio)
                else:
                    print("  0.5 <= acc <= 8.0")
            continue

        if kind == "joint":
            do_move_joint(bot, payload)
            continue

        auto = payload[-1] == "auto"
        vals = payload[:-1] if auto else payload
        x, y = vals[0] / 1000.0, vals[1] / 1000.0
        if kind == "rel":
            p = bot.pose()
            x, y = p[0] + x, p[1] + y
        if auto:
            phi = None
        elif len(vals) == 3:
            phi = vals[2] * D
        else:
            phi = bot.pose()[5]
        do_move(bot, x, y, phi, elbow, linear)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, help="목표 x [mm]")
    ap.add_argument("--y", type=float, help="목표 y [mm]")
    ap.add_argument("--phi", type=float, default=None,
                    help="목표 방향 [deg]. 생략하면 현재값 유지")
    ap.add_argument("--execute", action="store_true", help="실제로 움직인다")
    ap.add_argument("--linear", action="store_true", help="직선(move_linear)으로")
    ap.add_argument("--elbow", default="both", choices=["both", "up", "down"])
    ap.add_argument("--speed-fraction", type=float, default=0.15)
    ap.add_argument("--acc-ratio", type=float, default=motion.ACC_RATIO_ZERO_VIB)
    args = ap.parse_args()

    try:
        bot = SCARA(speed_fraction=args.speed_fraction, acc_ratio=args.acc_ratio)
    except SafetyViolation as exc:
        print(f"[기동 전 점검 실패] {exc}")
        print("  전원(제어+주회로) -> StellaX -> Designer 연결 -> Step 4 Verification")
        return 1
    except Exception as exc:
        print(f"[접속 실패] {type(exc).__name__}: {exc}")
        return 1

    try:
        print("=" * 70)
        print("현재 상태")
        print("=" * 70)
        show_state(bot)

        if args.x is not None and args.y is not None:
            x, y = args.x / 1000.0, args.y / 1000.0
            phi = args.phi * D if args.phi is not None else bot.pose()[5]
            if not args.execute:
                try:
                    show_plan(bot, x, y, phi, args.elbow, args.linear)
                    print("\n[dry-run] --execute 를 붙이면 실제로 움직인다.")
                except Exception as exc:
                    print(f"\n  [거부] {type(exc).__name__}: {exc}")
                    return 1
                return 0
            return 0 if do_move(bot, x, y, phi, args.elbow, args.linear,
                                assume_yes=True) else 1

        return interactive(bot, args.elbow, args.linear)
    finally:
        bot.close()
        print("서보 OFF, 연결 해제.")


if __name__ == "__main__":
    raise SystemExit(main())
