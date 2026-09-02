"""기구학 검증. 실기 없이 돌아간다.

    python -m pytest sw/tests/ -v
    python sw/tests/test_kinematics.py     (pytest 없이도 실행됨)
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import config, kinematics as kin   # noqa: E402


DEG = math.pi / 180.0
TOL = 1e-9


# -----------------------------------------------------------------------------
# 정기구학
# -----------------------------------------------------------------------------

def test_fk_zero_pose_is_fully_extended():
    """q=[0,0,0] 이면 팔이 +x로 완전히 펴진다 (CAD 포즈와 대응)."""
    x, y, z, phi = kin.fk([0, 0, 0])
    assert abs(x - config.MAX_REACH_M) < TOL, f"x={x}"
    assert abs(y) < TOL
    assert abs(phi) < TOL


def test_fk_z_is_constant():
    """3자유도 평면 아암이므로 z는 관절각과 무관하다."""
    z0 = kin.fk([0, 0, 0])[2]
    for q in ([30 * DEG, -60 * DEG, 45 * DEG], [-90 * DEG, 120 * DEG, -30 * DEG]):
        assert abs(kin.fk(q)[2] - z0) < TOL


def test_fk_z_matches_cad_heights():
    """d1+d2 = 링크3 상면 − 베이스 오프셋 = 171.5 − 215.5 = −44.0 mm"""
    z = kin.fk([0, 0, 0])[2]
    assert abs(z - (-0.0440)) < 1e-9, f"z={z}"


def test_joint_positions_match_cad():
    """q=0 에서 관절 위치가 CAD 실측(0 / 250 / 500 / 650 mm)과 맞아야 한다."""
    pts = kin.joint_positions([0, 0, 0])
    xs = [round(p[0] * 1000, 6) for p in pts]
    assert xs == [0.0, 250.0, 500.0, 650.0], xs


# -----------------------------------------------------------------------------
# 역기구학 왕복
# -----------------------------------------------------------------------------

def test_ik_roundtrip():
    """FK로 만든 자세를 IK로 되돌리면 같은 TCP가 나와야 한다."""
    cases = [
        [10 * DEG,  40 * DEG,  20 * DEG],
        [-30 * DEG, 80 * DEG, -50 * DEG],
        [90 * DEG, -60 * DEG,  30 * DEG],
        [0 * DEG,   90 * DEG,  0 * DEG],
        [45 * DEG, -100 * DEG, 15 * DEG],
    ]
    for q_ref in cases:
        x, y, _, phi = kin.fk(q_ref)
        q_sol = kin.ik(x, y, phi, seed_q=q_ref)
        x2, y2, _, phi2 = kin.fk(q_sol)
        assert abs(x - x2) < 1e-9, f"{q_ref}: x {x} != {x2}"
        assert abs(y - y2) < 1e-9, f"{q_ref}: y {y} != {y2}"
        assert abs(kin._wrap(phi - phi2)) < 1e-9, f"{q_ref}: phi"


def test_ik_seed_picks_nearest():
    """시드를 주면 그 자세에 가까운 해를 골라야 한다 (자세 뒤집힘 방지)."""
    q_ref = [20 * DEG, 70 * DEG, -30 * DEG]     # elbow up
    x, y, _, phi = kin.fk(q_ref)
    q_sol = kin.ik(x, y, phi, seed_q=q_ref)
    for a, b in zip(q_sol, q_ref):
        assert abs(kin._wrap(a - b)) < 1e-9


def test_ik_two_solutions():
    """특이점이 아니면 elbow up/down 두 해가 있어야 한다."""
    x, y, _, phi = kin.fk([20 * DEG, 70 * DEG, -30 * DEG])
    sols = kin.ik_all(x, y, phi)
    assert len(sols) == 2
    assert {s.elbow for s in sols} == {kin.ElbowConfig.UP, kin.ElbowConfig.DOWN}
    # 두 해 모두 같은 TCP를 만들어야 한다
    for s in sols:
        x2, y2, _, phi2 = kin.fk(s.q)
        assert abs(x - x2) < 1e-9 and abs(y - y2) < 1e-9


def test_ik_elbow_selection():
    x, y, _, phi = kin.fk([20 * DEG, 70 * DEG, -30 * DEG])
    q_up = kin.ik(x, y, phi, elbow=kin.ElbowConfig.UP)
    q_dn = kin.ik(x, y, phi, elbow=kin.ElbowConfig.DOWN)
    assert q_up[1] > 0 and q_dn[1] < 0


def test_ik_unreachable_raises():
    """도달 반경 밖은 예외."""
    try:
        kin.ik(1.0, 1.0, 0.0)
    except kin.IKError:
        pass
    else:
        raise AssertionError("도달 불가인데 예외가 안 났다")


def test_ik_respects_joint_limits():
    """가동범위를 주면 범위 밖 해는 버려야 한다."""
    q_ref = [20 * DEG, 70 * DEG, -30 * DEG]
    x, y, _, phi = kin.fk(q_ref)
    # elbow down 만 허용되는 범위를 준다
    limits = [(-math.pi, math.pi), (-math.pi, 0.0), (-math.pi, math.pi)]
    q = kin.ik(x, y, phi, joint_limits=limits)
    assert q[1] <= 0, f"범위 밖 해가 선택됨: q2={math.degrees(q[1])}"


# -----------------------------------------------------------------------------
# 특이점
# -----------------------------------------------------------------------------

def test_singularity_at_full_extension():
    """q2=0 (완전히 펴짐) 은 특이점이다."""
    assert kin.manipulability([0, 0, 0]) < 1e-12
    assert kin.manipulability([30 * DEG, 0, 0]) < 1e-12


def test_manipulability_max_at_90deg():
    """q2 = ±90° 에서 조작성이 최대."""
    m90 = kin.manipulability([0, 90 * DEG, 0])
    a1, a2, _ = config.LINK_LENGTHS_M
    assert abs(m90 - a1 * a2) < 1e-12
    assert m90 > kin.manipulability([0, 45 * DEG, 0])


def test_joint_velocity_explodes_near_singularity():
    """특이점에 가까울수록 같은 TCP 속도에 필요한 관절속도가 커진다."""
    twist = (0.05, 0.0, 0.0)   # 50 mm/s
    qd_far = kin.joint_velocity([0, 90 * DEG, 0], twist)
    qd_near = kin.joint_velocity([0, 5 * DEG, 0], twist)
    assert max(abs(v) for v in qd_near) > max(abs(v) for v in qd_far) * 5


def test_joint_velocity_at_singularity_raises():
    try:
        kin.joint_velocity([0, 0, 0], (0.05, 0.0, 0.0))
    except kin.IKError:
        pass
    else:
        raise AssertionError("특이점인데 예외가 안 났다")


# -----------------------------------------------------------------------------
# 작업영역
# -----------------------------------------------------------------------------

def test_workspace_radius():
    inner, outer = kin.workspace_radius()
    assert abs(outer - 0.650) < TOL
    a1, a2, a3 = config.LINK_LENGTHS_M
    assert abs(inner - max(0.0, abs(a1 - a2) - a3)) < TOL


def test_reachability_matches_ik():
    """is_reachable() 과 ik_all() 의 판정이 일치해야 한다."""
    for x in [i * 0.05 for i in range(-14, 15)]:
        for y in [i * 0.05 for i in range(-14, 15)]:
            assert kin.is_reachable(x, y, 0.0) == bool(kin.ik_all(x, y, 0.0))


# -----------------------------------------------------------------------------

def _run_all():
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{len(fns) - failed}/{len(fns)} 통과")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
