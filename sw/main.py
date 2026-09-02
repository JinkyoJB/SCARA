"""SCARA 메인 — 켜고, 영점 맞추고, 클릭으로 제어한다.

흐름
----
    1. 접속 + 기동 전 점검 (에러 / 제어루프 / 회전방향)
    2. **영점 확정** — 팔을 일직선으로 만들고 그 자세를 q=[0,0,0] 으로 선언
    3. 클릭 UI 실행 (갈 수 있는 곳 초록, 더블클릭하면 이동)
    4. UI 창을 닫거나 Ctrl+C 로 종료

왜 영점부터인가
---------------
이 로봇에서 가장 자주 깨지는 것이 영점이다. 엔코더(배터리 앱솔루트)는 위치를
잃지 않지만, **'카운트 -> 각도' 오프셋 설정**은 Designer 에서 설정을 바꿔 적용할
때마다 프로젝트의 옛 값으로 되돌아간다.

그리고 영점이 틀렸는지는 **소프트웨어가 스스로 알 수 없다.** 컨트롤러는 언제나
"지령대로 갔다" 고 보고하기 때문이다 (자기 좌표계 안의 순환논리, 5.9절).
그래서 시작할 때마다 **사람 눈으로 한 번** 확인한다. 이것이 유일한 정직한 검사다.

영점이 불명확한 상태에서는 자동으로 큰 이동을 하지 않는다. 컨트롤러가 믿는
각도와 실제 팔이 어긋나 있어서 충돌 검사의 입력부터 틀리기 때문이다.
손으로 대충 편 뒤 작은 조그로만 다듬는다.

사용:
    python sw/main.py
    python sw/main.py --skip-zero     # 영점이 확실할 때만 (검증을 건너뛴다)
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from scara import console  # noqa: E402
console.setup()

from scara import config                       # noqa: E402
from scara.robot import SCARA                  # noqa: E402
from scara.safety import SafetyViolation       # noqa: E402

R = 180.0 / math.pi
LINE = "=" * 72


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        raise KeyboardInterrupt


def ask_yn(prompt: str) -> bool:
    while True:
        a = ask(f"  {prompt} [y/n] > ").lower()
        if a.startswith("y"):
            return True
        if a.startswith("n"):
            return False
        print("    y 또는 n 으로 답할 것")


# =============================================================================
# 1. 영점 확정
# =============================================================================

def align_manually(bot: SCARA) -> bool:
    """사람이 팔을 일직선으로 만드는 단계. 큰 자동 이동은 하지 않는다.

    1) 서보를 끄고 손으로 대충 편다 (제일 빠르고 안전하다)
    2) 필요하면 작은 조그로 다듬는다
    """
    print()
    print("-" * 72)
    print("팔을 일직선으로 만든다")
    print("-" * 72)
    print("  영점이 불명확하므로 자동으로 크게 움직이지 않는다.")
    print("    작은 조그(한 번에 10도 이하)로만 다듬는다.")
    print()
    print("  ** 손으로 돌아가는 축과 안 돌아가는 축이 다르다 (감속비 때문) **")
    print("     J3 (50.5:1)          -> 손으로 돌아간다")
    print("     J2 (100:1)           -> 손으로는 거의 안 된다. **조그를 쓸 것**")
    print("     J1 (100:1 + 브레이크) -> 손으로 안 된다.   **조그를 쓸 것**")
    print("     100:1 을 출력측에서 역구동하려면 마찰이 감속비만큼 증폭된다. 정상이다.")
    print()

    if ask_yn("서보를 끄고 손으로 J3 부터 펴시겠는가? (건너뛰고 조그만 써도 됨)"):
        bot.stop()
        bot.servo(False)
        print("  서보 OFF. 손으로 펼 수 있는 만큼 펴 주세요 (주로 J3).")
        print("  조그(j2 10 등)를 쓰면 서보가 다시 켜진다. 그 뒤에 손으로 밀지 말 것")
        print("     — 모터와 힘겨루기가 되어 과부하(Err16.0)가 난다.")
        ask("  다 됐으면 Enter > ")

    print()
    print("  조그로 마저 맞춘다. 서보가 자동으로 켜진다.")
    print("    j2 10      J2 를 +10도       (한 번에 10도까지)")
    print("    10         직전 축을 +10도   (반복할 때 편하다)")
    print("    -5         직전 축을 -5도")
    print("    s          현재 상태 다시 보기")
    print("    ok         정렬 끝  /  q  취소")
    print()
    print("  요령: 크게(10도) 몇 번 -> 눈으로 보고 -> 작게(1도, 0.2도) 다듬기")
    last = 1                                   # 직전에 움직인 축 (기본 J2)
    while True:
        q = bot.q_deg()
        print(f"    현재 q = [{q[0]:+8.3f}, {q[1]:+8.3f}, {q[2]:+8.3f}] deg"
              f"   (직전 축 {config.JOINTS[last].name})")
        raw = ask("  [정렬] > ").lower()
        if raw in ("q", "quit", "exit"):
            return False
        if raw == "ok":
            return True
        if raw == "s":
            continue
        parts = raw.replace(",", " ").split()
        if parts and parts[0] in ("j1", "j2", "j3"):
            last = int(parts[0][1]) - 1
            parts = parts[1:]
            if not parts:                      # "j2" 만 -> 축 선택만
                continue
        if len(parts) != 1:
            print("    형식:  j2 10   /   10   /   -5   /   ok   /   q")
            continue
        try:
            delta = float(parts[0])
        except ValueError:
            print("    각도는 숫자로.")
            continue
        try:
            if not bot.jog_deg(last, delta):
                print("    거의 안 움직였다. 전원·Designer Step4 를 확인할 것.")
        except SafetyViolation as exc:
            print(f"    [거부] {exc}")


def ensure_zero(bot: SCARA) -> bool:
    """영점을 확정한다. True 면 이후 단계로 진행해도 좋다."""
    print()
    print(LINE)
    print("1단계 — 영점 확정")
    print(LINE)
    st = bot.state()
    print(f"  컨트롤러가 읽는 q = {st['q_deg']} deg")
    print(f"  컨트롤러가 믿는 TCP = {st['tcp_mm']} mm")
    print()
    print("  ** 화면 숫자만으로는 영점이 맞는지 알 수 없다. 팔을 직접 볼 것. **")
    print()

    straight = ask_yn("지금 팔이 **한 직선**으로 쭉 펴져 있는가?")

    if straight and bot.zero_looks_valid():
        print("  -> 팔이 일직선이고 q 도 0 이다. **영점 정상.**")
        return True

    if straight and not bot.zero_looks_valid():
        print(f"  -> 팔은 일직선인데 q 가 0 이 아니다 ({st['q_deg']}).")
        print("     = 영점(오프셋)이 되돌아간 상태. 지금 자세를 0 으로 선언하면 된다.")
        if not ask_yn("지금 이 자세를 q=[0,0,0] 으로 선언할까?"):
            return False
    else:
        print("  -> 팔이 일직선이 아니다. 먼저 펴야 한다.")
        if not align_manually(bot):
            print("  영점 설정을 중단했다.")
            return False
        if not ask_yn("이제 팔이 한 직선으로 펴졌는가?"):
            print("  일직선이 아니면 선언하면 안 된다. 중단.")
            return False

    res = bot.declare_zero()
    print(f"  선언 완료. drift 보정 {res['drift_deg']} deg")
    print(f"  q = {res['q_deg']} deg,  TCP = {res['tcp_mm']} mm  (기대 650.00, 0.00)")
    print(f"  백업: sw/zero_backups/zero_backup_{res['backup']}.json")
    print()
    print("  이 보정은 런타임 값이다. Designer 에서 설정을 바꿔 적용하면 되돌아간다.")
    print("     영구 저장은 Motion Studio 의 origin calibration (Encoder Offset) 뿐이다.")

    # calibrate_joint_angle 은 엔코더 오프셋을 쓰면서 드라이버를 EtherCAT OP
    # 밖으로 내보낸다. 직후 세 축이 Err80.x (0xFF50) 로 폴트에 빠지고 제어루프가
    # 멈춘다. reset_trigger() 로는 안 풀리고 마스터가 OP 를 다시 세워야 한다.
    # 여기서 확인하지 않으면 다음 단계인 UI 진입에서 점검 실패로 떨어진다.
    if not wait_for_ethercat(bot):
        return False
    return True


def wait_for_ethercat(bot: SCARA) -> bool:
    """영점 적용 뒤 드라이버가 OP 에서 떨어졌으면 복구를 안내하고 기다린다."""
    problems = bot.health()
    if not problems:
        print("  드라이버 상태 정상. 계속 진행한다.")
        return True

    print()
    print("-" * 72)
    print("영점 적용 부작용 — 드라이버가 EtherCAT OP 에서 떨어졌다")
    print("-" * 72)
    for p in problems:
        print("  -", p)
    print()
    print("  알려진 현상이다 (docs/controller.md 참조).")
    print("  calibrate_joint_angle 이 엔코더 오프셋을 쓰면서 드라이버를 OP 밖으로 낸다.")
    print("  영점 자체는 제대로 들어갔으니 다시 잡을 필요는 없다.")
    print()
    print("  복구:")
    print("    1) 브라우저에서 Stella Designer 를 다시 연결")
    print("    2) **Step 4: Verification** 까지 진행")
    print("    3) (Designer 는 동작모드를 AUTO 로 되돌린다 — 이동할 때 자동으로 MANUAL 전환된다)")
    print()
    for attempt in range(1, 6):
        try:
            ask(f"  복구했으면 Enter (건너뛰려면 s + Enter)  [{attempt}/5] > ")
        except KeyboardInterrupt:
            return False
        problems = bot.health()
        if not problems:
            print("  복구 확인. 계속 진행한다.")
            return True
        print("  아직 정상이 아니다:")
        for p in problems:
            print("    -", p)
    print("  복구되지 않아 중단한다.")
    return False


# =============================================================================
# 종료 절차
# =============================================================================

def park_home(bot: SCARA, zero_ok: bool) -> None:
    """종료 전에 일직선 자세 q=[0,0,0] 으로 되돌린다.

    **왜**: 항상 같은 자세로 끝나면 다음 시작 때 "팔이 일직선인가" 를 바로 볼 수 있어
    영점 확인이 쉬워진다. 보관 자세로도 안전하다 (접힌 채 두면 다음에 손이 안 들어간다).

    조건이 맞을 때만 한다:
      - 영점이 확정된 세션에서만. 영점이 불명확하면 q=[0,0,0] 이 어디인지 몰라
        큰 이동이 위험하다.
      - 제어 루프가 살아 있고 에러가 없을 때만. 죽었으면 어차피 못 움직인다.
      - 이미 일직선이면 생략.
      - 사람이 확인한 뒤에. 종료할 때 팔이 갑자기 움직이면 위험하다.
    """
    if not zero_ok:
        print("  (영점이 확정되지 않은 세션이라 홈 복귀를 건너뛴다)")
        return
    try:
        if bot.health():
            print("  (드라이버/제어루프 상태가 정상이 아니라 홈 복귀를 건너뛴다)")
            return
        q = bot.q_deg()
    except Exception:
        return
    if max(abs(v) for v in q) < 0.5:
        print("  이미 일직선 자세다. 그대로 종료한다.")
        return

    print()
    print(f"  현재 q = [{q[0]:+.2f}, {q[1]:+.2f}, {q[2]:+.2f}] deg")
    print("  종료 전에 **일직선 자세(q=[0,0,0])로 되돌린다.**")
    print("  다음에 시작할 때 영점을 눈으로 바로 확인할 수 있다.")
    try:
        if not ask_yn("팔이 움직인다. 되돌릴까?"):
            print("  건너뛴다.")
            return
    except KeyboardInterrupt:
        print("  건너뛴다.")
        return
    try:
        bot.home()
        p = bot.tcp_mm()
        print(f"  홈 복귀 완료. TCP = ({p[0]:.2f}, {p[1]:.2f}) mm")
    except SafetyViolation as exc:
        print(f"  홈 복귀 거부: {exc}")
        print("  (경로에 충돌 위험이 있으면 막힌다. 수동으로 빼낼 것)")
    except Exception as exc:
        print(f"  홈 복귀 실패: {type(exc).__name__}: {exc}")
        try:
            bot.stop()
        except Exception:
            pass


# =============================================================================

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-zero", action="store_true",
                    help="영점 검증을 건너뛴다 (확실할 때만)")
    ap.add_argument("--grid", type=int, default=25, help="UI 도달격자 간격 [mm]")
    args = ap.parse_args()

    print(LINE)
    print("SCARA 제어")
    print(LINE)

    bot = None
    zero_ok = False          # 영점이 확정된 세션인가 (종료 시 홈 복귀 조건)
    try:
        try:
            bot = SCARA(auto_health=False)
        except Exception as exc:
            print(f"[접속 실패] {exc}")
            print("  전원(제어+주회로) -> StellaX -> Designer 연결 -> Step 4 Verification")
            return 1

        problems = bot.health()
        if problems:
            print("[기동 전 점검 실패]")
            for p in problems:
                print("  -", p)
            print()
            print("  대개 원인: 주회로 전원 미투입 / Designer Step 4 미실행 / 회전방향 미확정")
            print("  기동 순서는 docs/controller.md 참조")
            return 1
        print("  기동 전 점검 통과.")

        if args.skip_zero:
            print("  [경고] --skip-zero. 영점 검증을 건너뛴다.")
            zero_ok = False                     # 검증 안 했으니 홈 복귀도 안 한다
        elif not ensure_zero(bot):
            print()
            print("영점이 확정되지 않아 종료한다. (제어를 시작하지 않는다)")
            return 1
        else:
            zero_ok = True

        # ---- 2단계: 클릭 UI ------------------------------------------------
        print()
        print(LINE)
        print("2단계 — 클릭 위치제어")
        print(LINE)
        print("  초록 = 갈 수 있는 곳,  주황 = 못 가는 곳")
        print("  **초록 영역 더블클릭 -> 그 위치로 이동**")
        print("  우측 세로 슬라이더로 가속(acc) 조절 (초록선 3.125 = 공진 회피 최적)")
        print("  종료: 창 닫기 또는 Ctrl+C")
        print()
        if not ask_yn("팔이 움직인다. 주변을 비웠는가?"):
            return 130

        from scara import ui
        return ui.run_ui(bot.raw, grid_mm=args.grid,
                         confirm=False, own_client=False)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C — 종료한다.")
        return 130
    finally:
        if bot is not None:
            try:
                park_home(bot, zero_ok)
            except Exception:
                pass
            try:
                bot.close()          # stop -> 정지확인 -> servo OFF -> 연결해제
            except Exception:
                pass
        print("정리 완료. 서보 OFF.")


if __name__ == "__main__":
    raise SystemExit(main())
