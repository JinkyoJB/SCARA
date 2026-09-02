"""평면 3R SCARA 해석적 기구학.

세 관절축이 모두 연직·평행이므로 기구학이 닫힌 형태로 풀린다.
StellaX가 자체 기구학을 갖고 있으므로 **런타임 필수 모듈은 아니지만**, 다음에 쓴다.

  1. StellaX의 FK/IK 결과 **대조 검증** (값이 어긋나면 설정이 잘못된 것)
  2. 명령 전송 **전에** 도달 가능성·특이점·해 개수 판단
  3. 오프라인 궤적 계획

DH 규약은 `HW/model/robot.py` 와 동일하다:

    T_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)

α = 0 이므로 각 링크 변환은

    [[c, -s, 0, a·c],
     [s,  c, 0, a·s],
     [0,  0, 1, d  ],
     [0,  0, 0, 1  ]]

따라서 위치는

    x = a1·c1 + a2·c12 + a3·c123
    y = a1·s1 + a2·s12 + a3·s123
    z = d1 + d2 + d3            (관절각과 무관 — 3자유도 평면 아암)
    φ = q1 + q2 + q3            (z축 둘레 방향)

**z는 제어할 수 없다.** 높이가 필요하면 기구를 바꿔야 한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from . import config

__all__ = [
    "ElbowConfig", "IKSolution", "IKError",
    "fk", "fk_pose", "ik", "ik_all", "is_reachable", "manipulability",
]


class IKError(ValueError):
    """역기구학 해가 없을 때."""


class ElbowConfig:
    """팔꿈치 자세. 평면 2링크는 해가 최대 2개다."""
    UP = "up"       # q2 > 0
    DOWN = "down"   # q2 < 0
    BOTH = "both"


@dataclass(frozen=True)
class IKSolution:
    q: tuple            # (q1, q2, q3) [rad]
    elbow: str          # ElbowConfig.UP / DOWN
    manipulability: float


def _links() -> tuple:
    return config.LINK_LENGTHS_M


def _d_total(d3: Optional[float] = None) -> float:
    """TCP 높이 = d1 + d2 + d3 (베이스 좌표계0 기준).

    d3(툴)은 엔드이펙터가 정해져야 결정된다. 미정이면 0으로 본다.
    """
    d1, d2, d3_cfg = config.DH_D_M
    if d3 is None:
        d3 = 0.0 if d3_cfg is config.PENDING or d3_cfg == config.PENDING else d3_cfg
    return d1 + d2 + d3


# =============================================================================
# 정기구학
# =============================================================================

def fk(q: Sequence[float], d3: Optional[float] = None) -> tuple:
    """관절각 → TCP 자세.

    Args:
        q: (q1, q2, q3) [rad]
        d3: 툴 z오프셋 [m]. None이면 config 값(미정이면 0).

    Returns:
        (x, y, z, phi) — 위치 [m], z축 둘레 방향 [rad]
    """
    if len(q) != 3:
        raise ValueError(f"관절 3개가 필요하다: {len(q)}개 받음")
    a1, a2, a3 = _links()
    q1, q2, q3 = q
    c1, s1 = math.cos(q1), math.sin(q1)
    c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)
    c123, s123 = math.cos(q1 + q2 + q3), math.sin(q1 + q2 + q3)

    x = a1 * c1 + a2 * c12 + a3 * c123
    y = a1 * s1 + a2 * s12 + a3 * s123
    return (x, y, _d_total(d3), q1 + q2 + q3)


def fk_pose(q: Sequence[float], d3: Optional[float] = None) -> list:
    """StellaX `get_actual_pose()` 와 같은 형식으로.

    Returns:
        [x, y, z, rx, ry, rz] — 위치 [m], 회전 [rad].
        평면 아암이라 rx = ry = 0, rz = q1+q2+q3.
    """
    x, y, z, phi = fk(q, d3)
    return [x, y, z, 0.0, 0.0, phi]


def joint_positions(q: Sequence[float]) -> list:
    """각 관절축의 평면 좌표. 시각화·충돌 검사용.

    Returns:
        [(0,0), (관절2 위치), (관절3 위치), (TCP 위치)]
    """
    a1, a2, a3 = _links()
    q1, q2, q3 = q
    p0 = (0.0, 0.0)
    p1 = (a1 * math.cos(q1), a1 * math.sin(q1))
    p2 = (p1[0] + a2 * math.cos(q1 + q2), p1[1] + a2 * math.sin(q1 + q2))
    p3 = (p2[0] + a3 * math.cos(q1 + q2 + q3), p2[1] + a3 * math.sin(q1 + q2 + q3))
    return [p0, p1, p2, p3]


# =============================================================================
# 역기구학
# =============================================================================

def is_reachable(x: float, y: float, phi: float = 0.0) -> bool:
    """(x, y, phi) 가 도달 가능한지. 해를 구하지 않고 빠르게 판정."""
    a1, a2, a3 = _links()
    xw = x - a3 * math.cos(phi)
    yw = y - a3 * math.sin(phi)
    r = math.hypot(xw, yw)
    return abs(a1 - a2) - 1e-12 <= r <= a1 + a2 + 1e-12


def ik_all(x: float, y: float, phi: float = 0.0, tol: float = 1e-9) -> list:
    """가능한 모든 해를 구한다 (최대 2개: elbow up / down).

    Args:
        x, y: TCP 평면 위치 [m]
        phi:  TCP z축 둘레 방향 [rad]

    Returns:
        IKSolution 리스트. 도달 불가면 빈 리스트.
    """
    a1, a2, a3 = _links()

    # 손목점 = TCP 에서 마지막 링크를 뺀 위치
    xw = x - a3 * math.cos(phi)
    yw = y - a3 * math.sin(phi)
    r2 = xw * xw + yw * yw
    r = math.sqrt(r2)

    # 2링크 도달 판정
    if r > a1 + a2 + tol or r < abs(a1 - a2) - tol:
        return []

    c2 = (r2 - a1 * a1 - a2 * a2) / (2.0 * a1 * a2)
    c2 = max(-1.0, min(1.0, c2))   # 수치오차로 |c2|가 1을 살짝 넘는 경우 방지
    s2_mag = math.sqrt(max(0.0, 1.0 - c2 * c2))

    base = math.atan2(yw, xw)
    out = []
    for s2, elbow in ((+s2_mag, ElbowConfig.UP), (-s2_mag, ElbowConfig.DOWN)):
        q2 = math.atan2(s2, c2)
        q1 = base - math.atan2(a2 * s2, a1 + a2 * c2)
        q3 = phi - q1 - q2
        q = (_wrap(q1), _wrap(q2), _wrap(q3))
        out.append(IKSolution(q=q, elbow=elbow, manipulability=manipulability(q)))
        if s2_mag < tol:
            break  # 완전히 편/접힌 특이점 — 해가 하나뿐
    return out


def ik(
    x: float,
    y: float,
    phi: float = 0.0,
    seed_q: Optional[Sequence[float]] = None,
    elbow: str = ElbowConfig.BOTH,
    joint_limits: Optional[Sequence] = None,
) -> tuple:
    """해 하나를 골라서 돌려준다.

    Args:
        seed_q: 있으면 여기서 **관절공간 거리가 가장 가까운** 해를 고른다.
                연속 동작에서 자세가 갑자기 뒤집히는 것을 막는다.
        elbow:  UP / DOWN 으로 고정하고 싶을 때.
        joint_limits: [(lo,hi), ...]. 주면 범위 밖 해를 버린다.
                      None이면 검사하지 않는다 (가동범위 미확정 상태 대응).

    Raises:
        IKError: 해가 없거나 전부 걸러졌을 때.
    """
    sols = ik_all(x, y, phi)
    if not sols:
        a1, a2, a3 = _links()
        raise IKError(
            f"도달 불가: (x={x:.4f}, y={y:.4f}, phi={math.degrees(phi):.1f}°). "
            f"손목점 반경이 [{abs(a1-a2):.4f}, {a1+a2:.4f}] m 범위를 벗어난다."
        )

    if elbow != ElbowConfig.BOTH:
        sols = [s for s in sols if s.elbow == elbow]
        if not sols:
            raise IKError(f"elbow={elbow} 해가 없다 (특이점 근처일 수 있다)")

    if joint_limits is not None:
        kept = [s for s in sols
                if all(lo <= a <= hi for a, (lo, hi) in zip(s.q, joint_limits))]
        if not kept:
            raise IKError(
                f"해는 있으나 전부 가동범위 밖이다. "
                f"후보: {[[round(math.degrees(a), 2) for a in s.q] for s in sols]}"
            )
        sols = kept

    if seed_q is not None:
        sols.sort(key=lambda s: sum((_wrap(a - b)) ** 2 for a, b in zip(s.q, seed_q)))
    else:
        # 시드가 없으면 특이점에서 먼(조작성이 큰) 해를 고른다
        sols.sort(key=lambda s: -s.manipulability)

    return sols[0].q


# =============================================================================
# 야코비안 / 특이점
# =============================================================================

def jacobian(q: Sequence[float]) -> list:
    """평면 야코비안 3×3.  [dx, dy, dphi]ᵀ = J · [dq1, dq2, dq3]ᵀ"""
    a1, a2, a3 = _links()
    q1, q2, q3 = q
    s1, c1 = math.sin(q1), math.cos(q1)
    s12, c12 = math.sin(q1 + q2), math.cos(q1 + q2)
    s123, c123 = math.sin(q1 + q2 + q3), math.cos(q1 + q2 + q3)

    return [
        [-a1 * s1 - a2 * s12 - a3 * s123, -a2 * s12 - a3 * s123, -a3 * s123],
        [ a1 * c1 + a2 * c12 + a3 * c123,  a2 * c12 + a3 * c123,  a3 * c123],
        [ 1.0,                             1.0,                   1.0],
    ]


def manipulability(q: Sequence[float]) -> float:
    """조작성 지표 = |det J|.

    0에 가까우면 특이점이다. 평면 3R의 특이점은 **팔이 완전히 펴지거나 접힌 자세**
    (q2 = 0 또는 ±π)이며, 이때 |det J| = a1·a2·|sin q2| → 0.
    특이점 근처에서는 작은 TCP 속도에도 관절속도가 폭발한다.
    이 기계는 속도 여유가 0 이므로 특히 위험하다.
    """
    a1, a2, _ = _links()
    return abs(a1 * a2 * math.sin(q[1]))


def joint_velocity(q: Sequence[float], twist: Sequence[float]) -> list:
    """TCP 속도 → 관절 속도. 특이점 근처에서 값이 폭발하므로 검사에 쓴다.

    Args:
        twist: (vx, vy, omega) — [m/s, m/s, rad/s]
    """
    J = jacobian(q)
    det = (J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1])
           - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0])
           + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]))
    if abs(det) < 1e-12:
        raise IKError(f"특이점이라 관절속도를 구할 수 없다 (det J = {det:.3e}, "
                      f"q2 = {math.degrees(q[1]):.2f}°)")

    # 3x3 역행렬 (크래머)
    def cof(r, c):
        m = [[J[i][j] for j in range(3) if j != c] for i in range(3) if i != r]
        return ((-1) ** (r + c)) * (m[0][0] * m[1][1] - m[0][1] * m[1][0])

    inv = [[cof(j, i) / det for j in range(3)] for i in range(3)]
    return [sum(inv[i][k] * twist[k] for k in range(3)) for i in range(3)]


# =============================================================================
# 유틸
# =============================================================================

def _wrap(a: float) -> float:
    """각도를 (-π, π] 로."""
    a = math.fmod(a, 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return a


def workspace_radius() -> tuple:
    """(내측 반경, 외측 반경) [m]. phi=0 기준 TCP 도달 범위."""
    a1, a2, a3 = _links()
    return (max(0.0, abs(a1 - a2) - a3), a1 + a2 + a3)
