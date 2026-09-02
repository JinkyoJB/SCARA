"""CAD 기반 자기충돌 검사.

관절 박스 한계로 보수적으로 막으면 실제로는 안전한 자세까지 잘려서 작업공간이
크게 줄어든다 (TCP 반경 210 mm 안쪽 접근 불가). 이 모듈은 CAD 형상에서 뽑은
2D 단면 모델로 "이 q 에서 부딪히는가" 를 직접 계산한다. 박스 한계는 완화하고
이것이 진짜 관문이 된다.

기하 데이터
-----------
`HW/HWscar-V2/HWscar-V2.1.obj` (조립 상태, 미터 단위, 영점 자세) 를 부품별로
파싱해 얻은 bbox 를 정리한 것이다.
CAD 좌표 -> 로봇 좌표: +x = CAD -y, +y = CAD +x, z 동일.
실물이 CAD 대로 제작됐음은 총도달 650 mm 실측 일치로 확인했다.

층층이 쌓인 평면 로봇
---------------------
링크가 서로 다른 높이에 있다:

    link-1        z 0.185~0.215   <- 제일 위
    link-2        z 0.171~0.195
    link-3        z 0.153~0.172   <- 제일 아래
    J1 지지구조물  z 0.000~0.190   (기둥 ~0.167, J1 감속기 0.164~0.184)

z 구간이 겹치는 쌍만 부딪힐 수 있다. 그래서 각 몸체를 "부착 프레임 기준
2D 도형(캡슐/원) + z 구간" 으로 두고, z 가 겹치는 쌍만 2D 거리 검사를 한다.

위험한 쌍:
  - link-3 는 J1 지지구조물과 같은 높이 -> base_* 3종
  - link-2 는 J1 감속기 높이와 겹침 -> base_col_red 등
  - link-2 끝의 J3 구동부(z 0.169~0.327)는 link-1 높이대와 겹침
  - link-3 끝의 RF 타워(z 0.157~0.382)는 link-1·link-2·모터들과 겹침

q1 무관성
---------
지지구조물을 원점 중심 원기둥으로 근사했으므로 (기둥 4개의 모서리 반경 88 mm 를
덮는 원 89 mm — 면 방향으로는 26 mm 보수적), 자기충돌은 q1 과 무관하게
(q2, q3) 만으로 결정된다. 검사도 지도도 2차원이면 충분하다.

덮지 못하는 것: 케이블 하네스. 기본 여유 10 mm 는 CAD-실물 오차와 얇은
케이블까지 흡수하도록 잡은 값이다. 굵은 케이블 뭉치가 지나는 경로가 생기면
margin 을 키우거나 몸체를 추가할 것.
"""
from __future__ import annotations

import math
from typing import Sequence

from .safety import SafetyViolation

__all__ = ["clearance", "check_self_collision", "MARGIN_M", "BODIES", "PAIRS"]

#: 기본 안전 여유 [m]. CAD-실물 오차 + 얇은 케이블 몫.
MARGIN_M = 0.010

# =============================================================================
# 몸체 정의
#   frame: 0=베이스(고정)  1=링크1(J1과 회전)  2=링크2  3=링크3
#   prim : ("capsule", (x0,y0), (x1,y1), r)  또는  ("circle", (x,y), r)  [m]
#   z    : (하한, 상한) [m]
#   출처 주석의 좌표는 OBJ(CAD) 원본값.
# =============================================================================

BODIES = {
    # --- 베이스 (고정) ------------------------------------------------------
    # 기둥(bar x4): xy +-0.0625 사각 배열, z 0.017~0.167. 모서리 반경 88mm 를 원으로.
    "base_pillars": dict(frame=0, prim=("circle", (0.0, 0.0), 0.089),
                         z=(0.000, 0.167)),
    # J1 감속기(LzigTre): r 0.070, z 0.164~0.184
    "base_reducer": dict(frame=0, prim=("circle", (0.0, 0.0), 0.072),
                         z=(0.164, 0.184)),
    # J1 허브(PQ003 하우징 등): r 0.039, z 0.121~0.188
    "base_hub": dict(frame=0, prim=("circle", (0.0, 0.0), 0.041),
                     z=(0.121, 0.190)),

    # --- 링크1 (frame 1) ----------------------------------------------------
    # 오블롱 판: 축간 250, 반폭 41 (OBJ y -0.291~+0.039, x +-0.041) -> 캡슐과 정확히 일치
    "l1_body": dict(frame=1, prim=("capsule", (0.0, 0.0), (0.250, 0.0), 0.041),
                    z=(0.185, 0.215)),
    # 링크1 끝(팔꿈치)의 J2 감속기: 원 r39 @ (250,0), z 0.192~0.258
    "j2_reducer": dict(frame=1, prim=("circle", (0.250, 0.0), 0.039),
                       z=(0.192, 0.258)),
    # J2 모터 (위로 솟음): OBJ y -0.269~-0.204, x +-0.019 -> 캡슐 204~269
    "j2_motor": dict(frame=1, prim=("capsule", (0.223, 0.0), (0.250, 0.0), 0.019),
                     z=(0.233, 0.350)),

    # --- 링크2 (frame 2) ----------------------------------------------------
    "l2_body": dict(frame=2, prim=("capsule", (0.0, 0.0), (0.250, 0.0), 0.041),
                    z=(0.171, 0.195)),
    "j3_reducer": dict(frame=2, prim=("circle", (0.250, 0.0), 0.039),
                       z=(0.169, 0.234)),
    "j3_motor": dict(frame=2, prim=("capsule", (0.223, 0.0), (0.250, 0.0), 0.019),
                     z=(0.209, 0.327)),

    # --- 링크3 (frame 3) ----------------------------------------------------
    # OBJ y -0.670~-0.461 -> J3 기준 -39~+170. 캡슐 (0,0)-(131,0) r39 가 정확히 덮음
    "l3_body": dict(frame=3, prim=("capsule", (0.0, 0.0), (0.131, 0.0), 0.039),
                    z=(0.153, 0.172)),
    # TCP 근처(150mm 지점)의 RF 타워 3단 (위로 솟음)
    "rf_pillar": dict(frame=3, prim=("circle", (0.150, 0.0), 0.012),
                      z=(0.157, 0.329)),
    "rf_hold": dict(frame=3, prim=("circle", (0.150, 0.0), 0.018),
                    z=(0.319, 0.349)),
    "rf_box": dict(frame=3, prim=("circle", (0.150, 0.0), 0.027),
                   z=(0.337, 0.382)),
}

#: 검사에서 뺄 쌍 — **구조상 항상 붙어 있는** 곳 (관절 연결부 / 뿌리 적층).
#: 이들을 넣으면 모든 q 에서 거짓 충돌이 난다.
_EXCLUDE = {
    frozenset(("l1_body", "base_hub")),       # 링크1 뿌리가 허브 위에 얹혀 있다
    frozenset(("l1_body", "base_reducer")),   # 〃 (z 여유 확대로 걸리게 된 적층 쌍)
    frozenset(("l1_body", "l2_body")),        # J2 관절 연결부
    frozenset(("l2_body", "l3_body")),        # J3 관절 연결부
    frozenset(("l2_body", "j2_reducer")),     # 링크2 뿌리 = J2 감속기 출력
    frozenset(("l3_body", "j3_reducer")),     # 링크3 뿌리 = J3 감속기 출력
}

#: z 방향 안전 여유 [m].
#:
#: "z 구간이 안 겹치면 못 만난다" 로 쌍을 자르면 CAD 명목 간격이 2~4 mm 뿐인
#: 쌍까지 검사에서 빠진다:
#:     l2_body    <-> base_pillars   명목 4 mm
#:     j3_reducer <-> base_pillars   명목 2 mm
#: 실물에는 볼트 머리·와셔·처짐·공차가 있어 2 mm 는 없는 간격이나 같다.
#: 이 값보다 z 간격이 작으면 같은 층으로 보고 검사한다.
#: 이 여유를 넣기 전에 베이스 근처에서 실제로 충돌이 났다.
Z_MARGIN_M = 0.008

_Z_EPS = 0.002


def _build_pairs():
    names = list(BODIES)
    pairs = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = BODIES[a], BODIES[b]
            if A["frame"] == B["frame"]:
                continue                      # 같은 몸에 붙은 것끼리는 안 부딪힌다
            if frozenset((a, b)) in _EXCLUDE:
                continue
            zlo = max(A["z"][0], B["z"][0])
            zhi = min(A["z"][1], B["z"][1])
            if zhi - zlo < -Z_MARGIN_M:
                continue                      # z 간격이 여유(8mm)보다 커야 다른 층으로 인정
            pairs.append((a, b))
    return pairs


PAIRS = _build_pairs()


# =============================================================================
# 2D 기하
# =============================================================================

def _seg_seg_dist(p1, p2, p3, p4):
    """2D 선분-선분 최소 거리."""
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1]

    def clamp01(t):
        return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

    d1 = (p2[0] - p1[0], p2[1] - p1[1])
    d2 = (p4[0] - p3[0], p4[1] - p3[1])
    r = (p1[0] - p3[0], p1[1] - p3[1])
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    if a < 1e-12 and e < 1e-12:
        return math.hypot(r[0], r[1])
    if a < 1e-12:
        t = clamp01(f / e)
        q = (p3[0] + d2[0] * t, p3[1] + d2[1] * t)
        return math.hypot(p1[0] - q[0], p1[1] - q[1])
    c = dot(d1, r)
    if e < 1e-12:
        s = clamp01(-c / a)
        q = (p1[0] + d1[0] * s, p1[1] + d1[1] * s)
        return math.hypot(q[0] - p3[0], q[1] - p3[1])
    b = dot(d1, d2)
    den = a * e - b * b
    s = clamp01((b * f - c * e) / den) if den > 1e-12 else 0.0
    t = clamp01((b * s + f) / e)
    s = clamp01((b * t - c) / a)
    P = (p1[0] + d1[0] * s, p1[1] + d1[1] * s)
    Q = (p3[0] + d2[0] * t, p3[1] + d2[1] * t)
    return math.hypot(P[0] - Q[0], P[1] - Q[1])


def _frame_pose(q, frame):
    """프레임 원점(월드 xy)과 방향각. 링크 길이는 이 파일 안에서 자체 보유
    (config 순환 참조를 피하려고 상수로 둔다 — 값은 config 와 동일해야 한다)."""
    A1, A2 = 0.250, 0.250
    if frame == 0:
        return (0.0, 0.0), 0.0
    t1 = q[0]
    if frame == 1:
        return (0.0, 0.0), t1
    p1 = (A1 * math.cos(t1), A1 * math.sin(t1))
    t12 = t1 + q[1]
    if frame == 2:
        return p1, t12
    p2 = (p1[0] + A2 * math.cos(t12), p1[1] + A2 * math.sin(t12))
    return p2, t12 + q[2]


def _placed(body, q):
    """몸체의 2D 도형을 월드 좌표로. (p0, p1, r) 캡슐로 통일 (원은 p0==p1)."""
    (ox, oy), th = _frame_pose(q, body["frame"])
    c, s = math.cos(th), math.sin(th)

    def xf(p):
        return (ox + c * p[0] - s * p[1], oy + s * p[0] + c * p[1])

    prim = body["prim"]
    if prim[0] == "circle":
        p = xf(prim[1])
        return p, p, prim[2]
    return xf(prim[1]), xf(prim[2]), prim[3]


def clearance(q: Sequence[float]):
    """모든 검사 쌍의 최소 틈새.

    Returns:
        (min_clearance_m, (이름A, 이름B))  — 음수면 관통.
    """
    placed = {name: _placed(BODIES[name], q) for name in BODIES}
    worst = (float("inf"), ("", ""))
    for a, b in PAIRS:
        p1, p2, r1 = placed[a]
        p3, p4, r2 = placed[b]
        d = _seg_seg_dist(p1, p2, p3, p4) - r1 - r2
        if d < worst[0]:
            worst = (d, (a, b))
    return worst


def check_self_collision(q: Sequence[float], margin: float = MARGIN_M) -> None:
    """여유가 margin 미만이면 SafetyViolation.

    관절 박스 한계 대신 이것이 진짜 관문이다.
    """
    d, (a, b) = clearance(q)
    if d < margin:
        deg = [round(math.degrees(v), 2) for v in q]
        raise SafetyViolation(
            f"자기충돌 위험: {a} <-> {b} 틈새 {d*1000:.1f} mm "
            f"(여유 기준 {margin*1000:.0f} mm), q={deg}deg. "
            "CAD 기반 충돌 모델(collision.py)이 막았다."
        )
