"""안전 계층 — 명령을 보내기 전에 막는다.

이 기계의 위험 요소는 세 가지다.

1. **속도 여유가 0.** 모터 최고회전수 ÷ 기어비 = 감속기 허용 출력회전수와 정확히 같다.
   (PQ003-H: 6000/100 = 60 rpm = 감속기 한계 60 rpm)
   즉 모터가 낼 수 있는 최고 속도가 곧 감속기 파손 속도다.

2. **토크 병목이 축마다 다르다.** J1·J2는 감속기(29 N·m), J3는 모터(16.2 N·m)가 먼저 한계.

3. **STO 미지원.** A6BE는 안전토크차단이 없어 소프트웨어가 마지막 방어선이 아니다 —
   반드시 외부 E-stop 회로가 있어야 한다. 여기 코드는 그 보조일 뿐이다.

이 모듈은 하드웨어 보호용이지 인명 안전 장치가 아니다.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Sequence

from . import config
from .config import JOINTS, PENDING

log = logging.getLogger(__name__)


class SafetyViolation(RuntimeError):
    """명령이 하드웨어 한계를 넘었을 때."""


# -----------------------------------------------------------------------------
# 속도 / 가속도
# -----------------------------------------------------------------------------

def max_joint_speeds() -> tuple:
    """축별 최대 각속도 [rad/s] (하드웨어 절대 한계)."""
    return tuple(j.max_joint_speed_rad_s for j in JOINTS)


def rated_joint_speeds() -> tuple:
    """축별 정격 각속도 [rad/s]. 연속 운전은 이 안에서."""
    return tuple(j.rated_joint_speed_rad_s for j in JOINTS)


def check_speed(vel: Sequence[float], margin: float = 0.95) -> None:
    """관절 속도가 한계 안인지 검사.

    margin: 하드웨어 한계 대비 허용 비율. 기본 95% — 여유가 0인 기계라
            나머지 5%는 제어 오버슈트 몫으로 남긴다.
    """
    limits = max_joint_speeds()
    for i, (v, lim) in enumerate(zip(vel, limits)):
        if abs(v) > lim * margin:
            raise SafetyViolation(
                f"{JOINTS[i].name} 속도 {abs(v):.3f} rad/s 가 한계 "
                f"{lim:.3f} rad/s 의 {margin:.0%}({lim*margin:.3f})를 넘는다. "
                f"이 기계는 감속기 허용 회전수와 모터 최고 회전수가 같아 여유가 없다."
            )


def clamp_speed(vel: Sequence[float], margin: float = 0.95) -> list:
    """한계를 넘으면 잘라낸다. 잘랐으면 경고 로그."""
    limits = max_joint_speeds()
    out = []
    for i, (v, lim) in enumerate(zip(vel, limits)):
        cap = lim * margin
        if abs(v) > cap:
            log.warning("%s 속도 %.3f -> %.3f rad/s 로 클램프", JOINTS[i].name, v, cap)
            v = math.copysign(cap, v)
        out.append(v)
    return out


# -----------------------------------------------------------------------------
# 토크
# -----------------------------------------------------------------------------

def continuous_torque_limits() -> tuple:
    """축별 연속 토크 한계 [N·m]."""
    return tuple(j.continuous_torque_limit_nm for j in JOINTS)


def peak_torque_limits() -> tuple:
    return tuple(j.peak_torque_limit_nm for j in JOINTS)


def check_effort(effort: Sequence[float], margin: float = 0.8) -> list:
    """실측 토크를 감시. 한계를 넘은 축 인덱스 목록을 돌려준다 (예외는 안 던짐).

    호출부에서 정지 여부를 결정하도록 목록만 반환한다.
    감시 루프에서 매 주기 부르는 용도.
    """
    limits = continuous_torque_limits()
    over = []
    for i, (t, lim) in enumerate(zip(effort, limits)):
        if abs(t) > lim * margin:
            log.warning("%s 토크 %.2f N·m (연속한계 %.2f 의 %.0f%%)",
                        JOINTS[i].name, t, lim, 100 * abs(t) / lim)
            over.append(i)
    return over


# -----------------------------------------------------------------------------
# 가동범위
# -----------------------------------------------------------------------------



def check_joint_limits(q: Sequence[float]) -> None:
    """관절각이 가동범위 안인지 검사.

    가동범위가 미확정(PENDING)이면 통과시키지 않고 막는다. 모르는 값을
    추정치로 채우면 기구를 부순다.
    """
    limits = config.JOINT_LIMITS_RAD
    if limits is PENDING or limits == PENDING:
        raise SafetyViolation(
            "관절 가동범위가 아직 확정되지 않았다 (config.JOINT_LIMITS_RAD = PENDING). "
            "기구 리미트를 실측해서 채울 것."
        )
    for i, (angle, (lo, hi)) in enumerate(zip(q, limits)):
        if not (lo <= angle <= hi):
            raise SafetyViolation(
                f"{JOINTS[i].name} 목표각 {math.degrees(angle):.2f}° 가 "
                f"가동범위 [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]° 밖이다."
            )


def check_direction_sign() -> None:
    """관절 회전 방향이 모델 규약과 맞는지 확인. 안 맞으면 막는다.

    **이 기계에서 두 번째로 위험한 검사다** (첫째는 특이점).
    부호가 하나라도 반대면 IK 가 푼 자세와 팔이 실제로 가는 자세가 달라진다.

    J2, J3 가 반대인 채로 (500, 0) 을 지령했더니 팔이 (96.6, 490.6),
    +x 로부터 +78.9 deg 로 갔다. 오차 635 mm. 그런데 모델도 컨트롤러도
    도착했다고 보고했다 — 둘이 같은 잘못된 규약을 쓰기 때문이다.

    고치는 곳은 컨트롤러다 (Designer -> Step 3 -> Joint Mapping -> Transmission 부호).
    고친 뒤 tools/verify_model.py 로 축별 확인하고 config 를 (1,1,1) 로 갱신할 것.
    """
    sign = config.DIRECTION_SIGN
    if sign is PENDING or sign == PENDING:
        raise SafetyViolation(
            "관절 회전 방향이 아직 확인되지 않았다 (config.DIRECTION_SIGN = PENDING). "
            "tools/verify_model.py 로 먼저 확인할 것."
        )
    bad = [JOINTS[i].name for i, v in enumerate(sign) if v != 1]
    if bad:
        raise SafetyViolation(
            f"회전 방향이 모델과 반대인 축이 있다: {', '.join(bad)} "
            f"(DIRECTION_SIGN = {tuple(sign)}). "
            "이 상태로 작업공간 이동을 하면 팔이 전혀 다른 곳으로 간다 "
            "(실측 오차 635 mm). "
            "Stellar Designer -> Step 3 -> Joint Mapping 에서 Transmission 계수의 "
            "부호를 뒤집고, dir 로 재확인한 뒤 config.DIRECTION_SIGN 을 (1, 1, 1) 로 "
            "고칠 것."
        )


def check_reach(x: float, y: float) -> None:
    """평면 도달 가능 여부. SCARA라 z는 무관하다."""
    r = math.hypot(x, y)
    if r > config.MAX_REACH_M:
        raise SafetyViolation(
            f"목표 반경 {r*1000:.1f} mm 가 최대 도달반경 "
            f"{config.MAX_REACH_M*1000:.0f} mm 를 넘는다."
        )


# -----------------------------------------------------------------------------
# 특이점
# -----------------------------------------------------------------------------

#: 특이점 판정 하한 [rad]. |q2| 가 이보다 작으면 팔이 거의 펴진 상태다.
#:
#: 15°에서 관절속도 증폭은 1/sin(15°) ≈ 3.9배.
#: 이 기계는 속도 여유가 0이라 이 이상은 위험하다.
#: 시뮬레이션으로 실측한 뒤 조정할 것.
SINGULARITY_MIN_Q2_RAD = math.radians(15.0)

#: 경고만 내는 구간 (하한의 2배까지)
SINGULARITY_WARN_Q2_RAD = math.radians(30.0)


def singularity_margin(q) -> float:
    """특이점까지의 여유. |sin q2| 를 그대로 쓴다 (0 = 특이점, 1 = 최선).

    평면 3R의 조작성은 |det J| = a1·a2·|sin q2| 이므로
    자세 관련 항은 |sin q2| 하나로 요약된다.
    """
    return abs(math.sin(q[1]))


def speed_amplification(q) -> float:
    """이 자세에서 TCP 속도 대비 관절속도가 몇 배로 증폭되는가.

    1/|sin q2| 에 비례한다. 특이점에서 무한대.
    """
    s = singularity_margin(q)
    return float("inf") if s < 1e-12 else 1.0 / s


def check_singularity(q, min_q2_rad: float = None) -> None:
    """자세가 특이점에 너무 가까우면 막는다.

    **이 기계에서 가장 중요한 검사다.** 속도 여유가 0(모터 최고속도 = 감속기 한계)이라
    특이점 근처에서 관절속도가 조금만 증폭돼도 곧바로 한계를 넘는다.
    """
    if min_q2_rad is None:
        min_q2_rad = SINGULARITY_MIN_Q2_RAD

    q2 = abs(_wrap_pi(q[1]))
    # q2 = 0 (완전히 펴짐) 과 q2 = ±180° (완전히 접힘) 둘 다 특이점
    dist = min(q2, abs(math.pi - q2))

    if dist < min_q2_rad:
        amp = speed_amplification(q)
        kind = "완전히 펴진" if q2 < math.pi / 2 else "완전히 접힌"
        raise SafetyViolation(
            f"특이점에 너무 가깝다: q2 = {math.degrees(q[1]):.2f}° ({kind} 자세). "
            f"하한 {math.degrees(min_q2_rad):.1f}°. "
            f"이 자세에서 관절속도가 TCP 속도의 {amp:.1f}배로 증폭된다 — "
            f"속도 여유가 없는 기계라 위험하다."
        )
    if dist < SINGULARITY_WARN_Q2_RAD:
        log.warning("특이점 근처: q2 = %.2f°, 속도 증폭 %.1f배",
                    math.degrees(q[1]), speed_amplification(q))


def _wrap_pi(a: float) -> float:
    a = math.fmod(a, 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return abs(a)


# -----------------------------------------------------------------------------
# 기동 / 정지 시퀀스
# -----------------------------------------------------------------------------

def brake_joints() -> tuple:
    """브레이크가 달린 축 목록. 현재 J1만."""
    return tuple(j.name for j in JOINTS if j.has_brake)


def active_errors(err) -> list:
    """`get_error_status()` 딕셔너리에서 **실제로 발생한** 에러만 골라낸다.

    `any(err.values())` 로 판정하면 안 된다. `alarm` 필드는 알람이 없어도
    `{'alarm_code': '', 'alarm_description': ''}` 라는 비어 있지 않은 딕셔너리라서
    항상 truthy 다. 거짓 양성이 난다.

    실제 에러 없음:
        {'emergency': False, ..., 'total_error': False,
         'alarm': {'alarm_code': '', 'alarm_description': ''}}
    """
    if not isinstance(err, dict):
        return []
    hits = []
    for key, val in err.items():
        if key == "alarm":
            if isinstance(val, dict):
                code = str(val.get("alarm_code", "")).strip()
                if code:
                    hits.append(f"alarm={code} ({val.get('alarm_description', '')})")
            elif val:
                hits.append(f"alarm={val}")
        elif val is True:
            hits.append(key)
    return hits


def startup_checklist(bot) -> list:
    """기동 전 확인. 문제 목록을 돌려준다 (비어 있으면 정상)."""
    problems = []

    if not bot.is_ready():
        problems.append("is_ready() = False — 비상정지 또는 충돌 상태. reset_trigger() 필요")

    try:
        hits = active_errors(bot.error_status())
        if hits:
            problems.append("에러 발생: " + ", ".join(hits))
    except Exception as exc:
        problems.append(f"에러 상태 조회 실패: {exc}")

    try:
        q = bot.joint_positions()
        if q is None or len(q) != config.DOF:
            problems.append(f"관절 수가 {config.DOF}가 아니다: {q}")
    except Exception as exc:
        problems.append(f"관절 위치 조회 실패: {exc}")

    problems.extend(check_control_loop_alive(bot))
    return problems


def check_control_loop_alive(bot, wait_s: float = 1.0) -> list:
    """StellaX 실시간 제어 루프가 돌고 있는지 확인. 문제 목록을 돌려준다.

    드라이버가 폴트로 떨어지면 StellaX 의 RT 루프가 멈춘다. 그런데 RPC 계층은
    멀쩡히 살아 있어서 `get_status()` 가 죽기 직전의 스냅샷을 계속 돌려준다.
    `ready`, `pds`, `servo_on` 이 전부 낡은 값이라 그대로 믿으면 "지령은 받는데
    안 움직인다" 를 한참 헤매게 된다.

    `client.status()` 는 캐시하지 않고 매번 RPC 를 던지므로, `time_stamp` 가
    그대로면 컨트롤러 쪽이 멈춘 것이 확실하다.

    복구는 RPC 로 안 된다. **브라우저에서 Stella Designer 를 다시 연결해
    `Step 4: Verification` 까지 진행**해야 EtherCAT 이 다시 올라온다.
    """
    def ts():
        return bot.status()["trajectory_status"]["time_stamp"]

    try:
        a = ts()
        time.sleep(wait_s)
        b = ts()
    except Exception as exc:
        return [f"제어 루프 확인 실패: {exc}"]

    if a != b:
        return []
    return [
        f"StellaX 제어 루프가 멈춰 있다 "
        f"(time_stamp 가 {wait_s:.1f}초 동안 {a:.3f} 에서 변하지 않음).\n"
        "    지금 읽히는 pds / servo_on / ready 는 전부 '멈추기 직전 스냅샷'이라 믿으면 안 된다.\n"
        "    복구: 브라우저에서 Stella Designer 를 다시 연결하고 "
        "**Step 4: Verification** 까지 진행할 것.\n"
        "    (reset_trigger() 나 드라이버만 전원 재투입으로는 안 풀린다)"
    ]


def summarize_limits() -> str:
    """한계값을 사람이 읽을 수 있게 출력. 시운전 전 눈으로 확인하는 용도."""
    lines = ["축   최대속도[°/s]  정격속도[°/s]  연속토크[N·m]  피크토크[N·m]  브레이크"]
    for j in JOINTS:
        lines.append(
            f"{j.name}   {math.degrees(j.max_joint_speed_rad_s):9.1f}  "
            f"{math.degrees(j.rated_joint_speed_rad_s):11.1f}  "
            f"{j.continuous_torque_limit_nm:12.1f}  "
            f"{j.peak_torque_limit_nm:12.1f}  "
            f"{'있음' if j.has_brake else '없음':>6}"
        )
    return "\n".join(lines)
