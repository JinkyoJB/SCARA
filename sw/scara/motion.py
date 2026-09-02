"""동작 명령 — 검사 파이프라인을 거친 뒤에만 StellaX로 내보낸다.

파이프라인:

    목표 (x, y, φ)
       ① 도달 가능성      kinematics.is_reachable
       ② IK + 자세 연속성  kinematics.ik(seed_q=현재자세)
       ③ 특이점 검사       safety.check_singularity
       ④ 가동범위 검사     safety.check_joint_limits
       ⑤ 속도 클램프       safety.clamp_speed
       → StellaX move_joint / move_linear
       ⑦ 토크 감시         EffortMonitor

궤적 생성·보간은 StellaX가 한다. 여기서는 **목표점과 속도만** 정해 넘긴다.

`client.Scara` 를 직접 쓰지 말고 여기를 통할 것.
"""

from __future__ import annotations

import logging
import math
from typing import Optional, Sequence

from . import collision, config, kinematics as kin, safety
from .client import Scara

log = logging.getLogger(__name__)

__all__ = [
    "plan_joint_target", "move_joint_safe", "move_joint_path",
    "move_to_xy", "EffortMonitor",
]


def _waypoint(q, vel: float, acc: float, radius: float = 0.0) -> dict:
    return {"q": list(q), "vel": vel, "acc": acc, "radius": radius}


def _default_speed(speed_fraction: float) -> float:
    """가장 느린 축 기준 속도 [rad/s]. 전 축이 한계를 넘지 않도록 보수적으로."""
    return min(safety.rated_joint_speeds()) * speed_fraction


# =============================================================================
# 계획 (전송하지 않음 — 검사만)
# =============================================================================

def plan_joint_target(
    x: float,
    y: float,
    phi: float = 0.0,
    seed_q: Optional[Sequence[float]] = None,
    elbow: str = kin.ElbowConfig.BOTH,
    check_limits: bool = True,
    check_singularity: bool = True,
) -> tuple:
    """(x, y, φ) 목표를 관절각으로 바꾸고 안전성을 검사한다. **전송하지 않는다.**

    실기 없이도 돌아가므로 경로 전체를 미리 검사하는 데 쓴다.

    여기서는 특이점을 **기본으로 검사한다.** 작업공간 좌표로 목표를 준다는 것은
    작업공간 이동을 의도한다는 뜻이기 때문이다. 관절각을 직접 주는
    `move_joint_safe` 와는 기본값이 반대다.

    Raises:
        kinematics.IKError: 도달 불가
        safety.SafetyViolation: 특이점 / 가동범위 위반
    """
    # 회전 방향이 틀리면 IK 결과와 실제 자세가 달라진다. 여기서 먼저 막는다.
    safety.check_direction_sign()
    safety.check_reach(x, y)

    limits = None
    if check_limits and config.JOINT_LIMITS_RAD is not config.PENDING:
        limits = config.JOINT_LIMITS_RAD

    q = kin.ik(x, y, phi, seed_q=seed_q, elbow=elbow, joint_limits=limits)

    if check_singularity:
        safety.check_singularity(q)
    if check_limits:
        safety.check_joint_limits(q)   # PENDING 이면 여기서 막힌다

    # CAD 기반 자기충돌 검사. 박스 한계를 완화한 대신 이것이 진짜 관문이다.
    collision.check_self_collision(q)

    return q


def plan_path(points, seed_q=None, **kwargs) -> list:
    """여러 점을 순서대로 계획한다. 앞 점의 해를 다음 점의 시드로 넘겨
    **자세 연속성(elbow 뒤집힘 방지)** 을 보장한다.

    Args:
        points: [(x, y, phi), ...]

    Returns:
        관절각 리스트
    """
    out = []
    for i, p in enumerate(points):
        x, y = p[0], p[1]
        phi = p[2] if len(p) > 2 else 0.0
        try:
            q = plan_joint_target(x, y, phi, seed_q=seed_q, **kwargs)
        except Exception as exc:
            raise type(exc)(f"경유점 {i} (x={x:.4f}, y={y:.4f}): {exc}") from exc
        out.append(q)
        seed_q = q
    return out


# =============================================================================
# 실행
# =============================================================================

def move_joint_safe(
    bot: Scara,
    q_target: Sequence[float],
    vel: Optional[float] = None,
    acc: Optional[float] = None,
    speed_fraction: float = 0.2,
    wait: bool = True,
    timeout_s: float = 30.0,
    check_singularity: bool = False,
):
    """관절 공간 이동 (단일 목표점).

    **특이점을 기본으로 검사하지 않는다.** 관절공간 이동은 각 축이 자기 각도로
    보간되어 갈 뿐이고, IK도 야코비안도 쓰지 않는다. 속도 증폭은 "TCP 속도 →
    관절 속도" 변환에서 생기는 현상이므로 `move_linear` 에만 해당한다.

    따라서 **특이점에서 빠져나오는 유일한 안전한 방법이 `move_joint`** 다.
    여기서 특이점을 막으면 한번 들어간 자세에서 나올 수 없게 된다.

    다만 목표가 특이점 근처면 **경고는 남긴다** — 이후 작업공간 이동이 막힐 것이기 때문이다.

    Args:
        speed_fraction: 정격 속도 대비 비율. 시운전 기본 20%.
        check_singularity: True로 주면 특이점 자세를 거부한다 (기본 False).
    """
    if len(q_target) != config.DOF:
        raise ValueError(f"관절 수가 맞지 않는다: {len(q_target)} != {config.DOF}")

    safety.check_direction_sign()
    safety.check_joint_limits(q_target)
    collision.check_self_collision(q_target)

    # 경로 중간도 검사한다 — 관절보간 경로가 목표는 안전해도 **중간에**
    # 충돌 자세를 지날 수 있다 (예: q3 를 크게 뒤집으며 지나가는 경우).
    # 최대 이동 5도당 한 점씩 샘플링.
    q_now = bot.joint_positions()
    travel = max(abs(a - b) for a, b in zip(q_target, q_now))
    n = max(2, int(math.degrees(travel) / 5.0))
    for k in range(1, n):
        t = k / n
        qm = [a + (b - a) * t for a, b in zip(q_now, q_target)]
        try:
            collision.check_self_collision(qm)
        except safety.SafetyViolation as exc:
            raise safety.SafetyViolation(
                f"관절보간 경로 {t:.0%} 지점에서 충돌: {exc}") from exc

    if check_singularity:
        safety.check_singularity(q_target)
    else:
        amp = safety.speed_amplification(q_target)
        if amp > 1.0 / math.sin(safety.SINGULARITY_MIN_Q2_RAD):
            log.warning(
                "목표가 특이점 근처다 (q2=%.2f°, 속도증폭 %.1f배). "
                "관절공간 이동이라 진행하지만, 이 자세에서 move_linear 는 막힌다.",
                math.degrees(q_target[1]), amp)

    if vel is None:
        vel = _default_speed(speed_fraction)
    if acc is None:
        acc = vel * 2.0          # 0.5초에 목표속도 도달
    safety.check_speed([vel] * config.DOF)

    log.info("move_joint -> %s (vel=%.4f rad/s = %.2f°/s)",
             [f"{math.degrees(a):.2f}°" for a in q_target], vel, math.degrees(vel))

    result = bot.move_joint([_waypoint(q_target, vel, acc)])
    if wait:
        bot.wait_until_done(timeout_s=timeout_s)
    return result


def move_joint_path(
    bot: Scara,
    q_list: Sequence[Sequence[float]],
    vel: Optional[float] = None,
    acc: Optional[float] = None,
    radius: float = 0.0,
    speed_fraction: float = 0.2,
    wait: bool = True,
    timeout_s: float = 60.0,
    check_singularity: bool = False,
):
    """여러 경유점을 이어서 이동. **모든 점을 미리 검사한 뒤** 한 번에 넘긴다.

    radius > 0 이면 경유점에서 정지하지 않고 블렌딩한다 (사이클타임 개선).
    특이점 검사는 관절공간 이동에 불필요하므로 기본 꺼져 있다 (`move_joint_safe` 참조).
    """
    for i, q in enumerate(q_list):
        try:
            safety.check_joint_limits(q)
            if check_singularity:
                safety.check_singularity(q)
        except safety.SafetyViolation as exc:
            raise safety.SafetyViolation(f"경유점 {i}: {exc}") from exc

    if vel is None:
        vel = _default_speed(speed_fraction)
    if acc is None:
        acc = vel * 2.0
    safety.check_speed([vel] * config.DOF)

    log.info("move_joint_path: 경유점 %d개, vel=%.2f°/s, radius=%.4f",
             len(q_list), math.degrees(vel), radius)

    wps = [_waypoint(q, vel, acc, radius) for q in q_list]
    result = bot.move_joint(wps)
    if wait:
        bot.wait_until_done(timeout_s=timeout_s)
    return result


def move_to_xy(
    bot: Scara,
    x: float,
    y: float,
    phi: float = 0.0,
    seed_q: Optional[Sequence[float]] = None,
    elbow: str = kin.ElbowConfig.BOTH,
    **kwargs,
):
    """평면 좌표(m)로 이동. SCARA라 z는 기구적으로 고정이다.

    IK 는 `kinematics.py` 로 푼다. StellaX RPC 왕복 없이 즉시 계산되고,
    특이점·자세 선택 정보를 함께 얻을 수 있기 때문이다.
    StellaX 내장 IK 와는 소수점 6자리까지 일치하는 것을 확인했다.
    """
    if seed_q is None:
        seed_q = bot.joint_positions()

    q = plan_joint_target(x, y, phi, seed_q=seed_q, elbow=elbow)
    return move_joint_safe(bot, q, **kwargs)


# =============================================================================
# 감시
# =============================================================================

def move_linear_safe(
    bot: Scara,
    x: float,
    y: float,
    phi: Optional[float] = None,
    vel: float = 0.05,
    acc: float = 0.25,
    wait: bool = True,
    timeout_s: float = 30.0,
    check_singularity: bool = True,
):
    """직선(작업공간) 이동. TCP가 직선을 그린다.

    특이점 근처에서 위험하다. 작업공간 이동은 경로 위 모든 점에서 IK 가 풀려야
    하고, 특이점에 가까우면 관절속도가 폭발한다. 이 기계는 속도 여유가 0 이라
    시작·끝 자세를 둘 다 미리 검사한다.

    Args:
        vel: TCP 속도 [m/s]. 기본 0.05 = 50 mm/s (보수적)
        acc: TCP 가속도 [m/s²]
        phi: 목표 방향 [rad]. None이면 현재 방향 유지
    """
    safety.check_reach(x, y)

    q_now = bot.joint_positions()
    pose_now = bot.pose()
    if phi is None:
        phi = pose_now[5]

    # 목표 자세를 미리 풀어서 검사한다 (StellaX에 보내기 전)
    q_goal = plan_joint_target(x, y, phi, seed_q=q_now,
                               check_singularity=check_singularity)

    if check_singularity:
        safety.check_singularity(q_now)   # 출발 자세도 확인

    # 경로 중간도 표본 검사 — 직선 경로는 중간에 특이점을 지날 수 있다
    n = 10
    for k in range(1, n):
        t = k / n
        xm = pose_now[0] + (x - pose_now[0]) * t
        ym = pose_now[1] + (y - pose_now[1]) * t
        pm = pose_now[5] + (phi - pose_now[5]) * t
        try:
            qm = kin.ik(xm, ym, pm, seed_q=q_now)
        except kin.IKError as exc:
            raise kin.IKError(f"직선 경로 {t:.0%} 지점에서 IK 실패: {exc}") from exc
        if check_singularity:
            try:
                safety.check_singularity(qm)
            except safety.SafetyViolation as exc:
                raise safety.SafetyViolation(
                    f"직선 경로 {t:.0%} 지점이 특이점을 지난다: {exc}") from exc
        safety.check_joint_limits(qm)

    task_v = min(vel, 0.25)   # 컨트롤러 Task Space 한계와 맞춤
    wp = {"pose": [x, y, pose_now[2], pose_now[3], pose_now[4], phi],
          "vel": task_v, "acc": acc, "radius": 0.0}

    log.info("move_linear -> (%.1f, %.1f) mm, phi=%.2f°, v=%.0f mm/s",
             x * 1000, y * 1000, math.degrees(phi), task_v * 1000)

    result = bot.move_linear([wp])
    if wait:
        bot.wait_until_done(timeout_s=timeout_s)
    return result


class EffortMonitor:
    """동작 중 관절 토크를 감시하는 컨텍스트 매니저.

        with EffortMonitor(bot) as mon:
            move_joint_safe(bot, q)
        print(mon.peak)

    폴링 방식이라 샘플 간격이 성기다. 정밀 감시가 필요하면 PDO 매핑
    4(1603h/1A03h)로 `6077h` 를 매 주기 받는 쪽을 검토할 것.
    """

    def __init__(self, bot: Scara, margin: float = 0.8, stop_on_over: bool = True):
        self.bot = bot
        self.margin = margin
        self.stop_on_over = stop_on_over
        self.peak = [0.0] * config.DOF
        self.violations: list = []

    def sample(self) -> None:
        try:
            effort = self.bot.joint_efforts()
        except Exception as exc:
            log.debug("토크 샘플 실패: %s", exc)
            return
        if not effort:
            return
        for i, t in enumerate(effort[: config.DOF]):
            self.peak[i] = max(self.peak[i], abs(t))
        over = safety.check_effort(effort, margin=self.margin)
        if over:
            self.violations.append((list(effort), over))
            if self.stop_on_over:
                log.error("토크 한계 초과 — 정지시킨다. 축=%s",
                          [config.JOINTS[i].name for i in over])
                self.bot.stop()

    def __enter__(self) -> "EffortMonitor":
        self.peak = [0.0] * config.DOF
        self.violations = []
        return self

    def __exit__(self, *exc) -> None:
        log.info("최대 관절토크: %s",
                 [f"{config.JOINTS[i].name}={v:.2f}N·m" for i, v in enumerate(self.peak)])


# =============================================================================
# 고수준 도우미
# =============================================================================

#: 가속 배수 기본값. 가속시간이 공진주기의 2배가 되어 잔류 진동 여기율이 0 이다.
ACC_RATIO_ZERO_VIB = config.RESONANCE_HZ / 2.0        # = 3.125

#: auto phi 가 허용하는 phi 변화 폭 [deg] (크게 바꾸면 이동 6배 느림 + 관능평가)
PHI_WINDOW_DEG = 60.0

#: 컨트롤러 작업공간 속도 한계 [m/s] 와 사용 비율 (경계에서 컨트롤러가 조이면 끊긴다)
TCP_VEL_LIMIT = 0.25
TCP_VEL_MARGIN = 0.85


def joint_margins_deg(q):
    """각 관절이 가동범위 끝까지 남긴 여유 [deg]."""
    return [min(v - lo, hi - v) * 180.0 / math.pi
            for v, (lo, hi) in zip(q, config.JOINT_LIMITS_RAD)]


def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def auto_phi(x, y, seed_q, elbow=kin.ElbowConfig.BOTH, step_deg=0.5):
    """위치만 주어졌을 때 phi 를 고른다 — 한계·특이점·충돌을 전부 통과하는
    후보 중 관절 여유가 최대인 것. 현재 phi 에서 +-PHI_WINDOW_DEG 안을 우선.

    Returns: (phi, q) 또는 (None, None)
    """
    phi_now = _wrap_deg(math.degrees(sum(seed_q)))
    best_near = best_any = None
    n = int(round(360.0 / step_deg))
    salog = logging.getLogger("scara.safety")
    prev = salog.level
    salog.setLevel(logging.ERROR)
    try:
        for k in range(n):
            phi = (-180.0 + k * step_deg) * math.pi / 180.0
            for sol in kin.ik_all(x, y, phi):
                if elbow != kin.ElbowConfig.BOTH and sol.elbow != elbow:
                    continue
                m = min(joint_margins_deg(sol.q))
                if m <= 0:
                    continue
                try:
                    safety.check_singularity(sol.q)
                    collision.check_self_collision(sol.q)
                except safety.SafetyViolation:
                    continue
                near = -sum((a - b) ** 2 for a, b in zip(sol.q, seed_q))
                key = (round(m, 2), near)
                cand = (key, phi, sol.q)
                if best_any is None or key > best_any[0]:
                    best_any = cand
                if abs(_wrap_deg(math.degrees(phi) - phi_now)) <= PHI_WINDOW_DEG:
                    if best_near is None or key > best_near[0]:
                        best_near = cand
    finally:
        salog.setLevel(prev)
    best = best_near or best_any
    return (best[1], best[2]) if best else (None, None)


def tcp_speed_cap(q_from, q_to):
    """TCP 속도가 작업공간 한계를 안 넘는 관절속도 상한 [rad/s]."""
    r = 0.0
    for q in (q_from, q_to):
        x, y, _, _ = kin.fk(q)
        r = max(r, math.hypot(x, y))
    if r < 1e-6:
        return float("inf")
    return TCP_VEL_MARGIN * TCP_VEL_LIMIT / r
