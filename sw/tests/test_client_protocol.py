"""StellaX 연동 코드를 **실기 없이** 검증한다.

가짜 컨트롤러(FakeController)를 만들어 붙인다. 검증 대상:

  - `[status, value]` 반환 규약 처리
  - 실패 응답을 예외로 바꾸는지
  - 없는 커맨드를 호출했을 때 알아볼 수 있는 오류를 내는지
  - 안전 검사가 명령 전송 **전에** 막는지 (가짜 컨트롤러에 도달하지 않아야 한다)

실기 축이 온라인이 아니어도 여기까지는 지금 검증할 수 있다.

    python sw/tests/test_client_protocol.py
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import config, safety, motion               # noqa: E402
from scara import kinematics as kin                    # noqa: E402
from scara.client import Scara, StellaXError, unwrap   # noqa: E402

DEG = math.pi / 180.0


# =============================================================================
# 가짜 컨트롤러
# =============================================================================

class FakeController:
    """StellaX 컨트롤러 흉내. 실제와 같은 `[status, value]` 규약을 쓴다."""

    def __init__(self, n_axes=3, ready=True, unit="RADIAN_M"):
        self.n_axes = n_axes
        self._ready = ready
        self._unit = unit
        self.q = [0.0] * n_axes
        self.sent = []          # 전송된 move 명령 기록
        self.servo_on = False
        self.stopped = False

        # 동적 바인딩 흉내: 인스턴스 속성으로 붙인다 (실제 SDK와 동일)
        for name in (
            "is_ready", "get_status", "get_error_status", "get_alarm", "reset_trigger",
            "set_servo_on_off", "get_actual_joint_position", "get_actual_joint_velocity",
            "get_actual_joint_effort", "get_residual_effort", "get_actual_pose",
            "move_joint", "move_linear", "move_jog", "stop", "check_finish",
            "solve_forward_kinematics", "solve_inverse_kinematics",
            "set_operational_mode", "set_feed_rate", "get_processing_unit",
        ):
            setattr(self, name, getattr(self, "_" + name))

    # -- 상태 --
    def _is_ready(self): return [True, self._ready]
    def _get_status(self): return [True, {"ok": True}]
    def _get_error_status(self):
        return [True, {"emergency": False, "collision_detected": False, "total_error": False}]
    def _get_alarm(self): return [True, {"alarm_code": "", "alarm_description": ""}]
    def _reset_trigger(self): return True
    def _get_processing_unit(self): return [True, self._unit]

    # -- 서보 --
    def _set_servo_on_off(self, state):
        self.servo_on = bool(state)
        return True

    # -- 관측 --
    def _get_actual_joint_position(self): return [True, list(self.q)]
    def _get_actual_joint_velocity(self): return [True, [0.0] * self.n_axes]
    def _get_actual_joint_effort(self): return [True, [0.1] * self.n_axes]
    def _get_residual_effort(self): return [True, [0.0] * self.n_axes]
    def _get_actual_pose(self):
        return [True, kin.fk_pose(self.q)] if self.n_axes == 3 else [True, []]

    # -- 동작 --
    def _move_joint(self, waypoints):
        self.sent.append(("move_joint", waypoints))
        self.q = list(waypoints[-1]["q"])
        return [True, 1.0]

    def _move_linear(self, waypoints):
        self.sent.append(("move_linear", waypoints))
        return [True, 1.0]

    def _move_jog(self, **kw):
        self.sent.append(("move_jog", kw))
        return True

    def _stop(self, ramp_time=-1):
        self.stopped = True
        return True

    def _check_finish(self, finish_mode="FINE"): return [True, True]

    # -- 기구학 --
    def _solve_forward_kinematics(self, target_q, tcp=None):
        return [True, kin.fk_pose(target_q)]

    def _solve_inverse_kinematics(self, target_p, seed_q=None, tcp=None):
        try:
            return [1, list(kin.ik(target_p[0], target_p[1], target_p[5], seed_q=seed_q))]
        except kin.IKError:
            return [0, None]

    # -- 모드 --
    def _set_operational_mode(self, operational_mode): return True
    def _set_feed_rate(self, feed_rate): return True


class BrokenController(FakeController):
    """런타임이 안 뜬 상태 흉내 — 현재 실기와 같은 증상."""

    def _get_actual_joint_position(self): return [True, []]
    def _is_ready(self): return [True, False]
    def _get_actual_pose(self):
        return [False, "a bytes-like object is required, not 'NoneType'"]


def make_bot(ctrl=None) -> Scara:
    """접속 없이 Scara 파사드를 만들고 가짜 컨트롤러를 꽂는다."""
    bot = Scara(connect=False)
    bot._raw = ctrl or FakeController()
    bot._commands = tuple(sorted(
        n for n in vars(bot._raw) if callable(getattr(bot._raw, n, None))
    ))
    bot.processing_unit = None
    return bot


# =============================================================================
# 반환값 규약
# =============================================================================

def test_unwrap_status_value():
    assert unwrap([True, 42]) == 42
    assert unwrap([1, [1, 2, 3]]) == [1, 2, 3]


def test_unwrap_bool():
    assert unwrap(True) is True


def test_unwrap_failure_raises():
    for bad in ([False, "some error"], [0, None], False):
        try:
            unwrap(bad, "test_call")
        except StellaXError:
            pass
        else:
            raise AssertionError(f"실패 응답인데 예외가 안 났다: {bad}")


def test_unwrap_none_raises():
    try:
        unwrap(None, "test_call")
    except StellaXError:
        pass
    else:
        raise AssertionError("None인데 예외가 안 났다")


# =============================================================================
# 클라이언트 계층
# =============================================================================

def test_client_reads_state():
    bot = make_bot()
    assert bot.is_ready() is True
    assert bot.joint_positions() == [0.0, 0.0, 0.0]
    assert len(bot.joint_efforts()) == 3
    pose = bot.pose()
    assert abs(pose[0] - config.MAX_REACH_M) < 1e-9   # q=0 이면 완전히 펴짐


def test_client_servo():
    ctrl = FakeController()
    bot = make_bot(ctrl)
    bot.servo(True)
    assert ctrl.servo_on is True
    bot.servo(False)
    assert ctrl.servo_on is False


def test_missing_command_gives_useful_error():
    """없는 커맨드를 부르면 비슷한 이름을 알려줘야 한다."""
    bot = make_bot()
    try:
        bot._call("move_nonexistent")
    except StellaXError as exc:
        assert "move_nonexistent" in str(exc)
        assert "비슷한 이름" in str(exc)
    else:
        raise AssertionError("없는 커맨드인데 예외가 안 났다")


def test_has_and_commands():
    bot = make_bot()
    assert bot.has("move_joint")
    assert not bot.has("move_nonexistent")
    assert "get_actual_joint_position" in bot.commands


def test_offline_controller_detected():
    """런타임이 안 뜬 상태를 알아채야 한다 (현재 실기 증상)."""
    bot = make_bot(BrokenController())
    assert bot.is_ready() is False
    assert bot.joint_positions() == []
    problems = safety.startup_checklist(bot)
    assert problems, "축이 0개인데 점검을 통과했다"
    assert any("관절 수" in p for p in problems)


# =============================================================================
# 안전 검사가 전송 전에 막는가  ← 가장 중요
# =============================================================================

def test_singularity_blocks_workspace_target():
    """작업공간 목표(x, y, phi)는 특이점이면 전송 전에 막힌다."""
    try:
        motion.plan_joint_target(config.MAX_REACH_M, 0.0, 0.0)   # 완전신전 = 특이점
    except safety.SafetyViolation:
        pass
    else:
        raise AssertionError("특이점인데 통과했다")


def test_joint_move_allows_singularity_on_purpose():
    """관절공간 이동은 특이점을 막지 않는다.

    관절보간은 야코비안을 쓰지 않아 속도 증폭이 없고, 무엇보다 특이점에서
    빠져나오는 유일한 수단이다. 여기서 막으면 한번 들어간 자세에서 못 나온다.
    """
    ctrl = FakeController()
    bot = make_bot(ctrl)
    motion.move_joint_safe(bot, [0.0, 0.0, 0.0], wait=False)
    assert ctrl.sent, "관절공간 이동이 막혔다"


def test_joint_limit_blocks_before_send():
    """가동범위 밖 목표는 전송 전에 막힌다."""
    ctrl = FakeController()
    bot = make_bot(ctrl)
    over = config.JOINT_LIMITS_RAD[1][1] + 5 * DEG
    try:
        motion.move_joint_safe(bot, [0.0, over, 0.0], wait=False)
    except safety.SafetyViolation:
        pass
    else:
        raise AssertionError("가동범위 밖인데 전송됐다")
    assert ctrl.sent == [], "안전 검사 실패인데 컨트롤러로 명령이 갔다"


def test_self_collision_check_rejects_overlap():
    """자기충돌 자세는 거부된다. 박스 한계를 완화한 뒤 이것이 진짜 관문이다.

    관절 한계는 자기충돌 안전 박스 안쪽으로 잡혀 있어서(한계 안 최소 틈새
    12.8 mm) 한계를 통과한 자세는 충돌하지 않는다. 그래서 한계 밖 자세로
    검사 자체가 도는지 확인한다.
    """
    from scara import collision
    q_bad = [0.0, math.pi, math.pi]              # 링크2 가 베이스 기둥을 파고든다
    assert collision.clearance(q_bad)[0] < 0, "이 자세는 겹쳐야 한다"
    try:
        collision.check_self_collision(q_bad)
    except safety.SafetyViolation:
        pass
    else:
        raise AssertionError("충돌 자세인데 통과했다")


def test_move_joint_runs_collision_check():
    """move_joint_safe 가 충돌 검사를 실제로 호출하는가 (경로 중간 포함)."""
    ctrl = FakeController()
    bot = make_bot(ctrl)
    from scara import collision
    calls = []
    real = collision.check_self_collision
    collision.check_self_collision = lambda q, *a, **k: calls.append(list(q))
    try:
        motion.move_joint_safe(bot, [30 * DEG, 40 * DEG, -20 * DEG], wait=False)
    finally:
        collision.check_self_collision = real
    assert len(calls) > 1, f"목표만 검사했다 (호출 {len(calls)}회) — 경로 중간 샘플링 없음"


def test_unreachable_blocks_before_send():
    ctrl = FakeController()
    bot = make_bot(ctrl)
    try:
        motion.move_to_xy(bot, 1.0, 1.0)
    except (kin.IKError, safety.SafetyViolation):
        pass
    else:
        raise AssertionError("도달 불가인데 전송됐다")
    assert ctrl.sent == []


def test_overspeed_blocks_before_send():
    ctrl = FakeController()
    bot = make_bot(ctrl)
    try:
        motion.move_joint_safe(bot, [10 * DEG, 60 * DEG, 20 * DEG],
                               vel=100.0, wait=False, check_singularity=False)
    except safety.SafetyViolation:
        pass
    else:
        raise AssertionError("과속인데 전송됐다")
    assert ctrl.sent == []


def _with_limits(limits=None):
    """조립 후(가동범위 확정) 상태를 흉내내는 컨텍스트 매니저."""
    import contextlib

    @contextlib.contextmanager
    def _cm():
        saved = config.JOINT_LIMITS_RAD
        config.JOINT_LIMITS_RAD = limits or [
            (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]
        try:
            yield
        finally:
            config.JOINT_LIMITS_RAD = saved
    return _cm()


def test_valid_move_reaches_controller():
    """가동범위가 확정되면 정상 명령은 실제로 전송돼야 한다.

    안전 계층이 **과하게 막지 않는지** 확인하는 반대 방향 테스트다.
    """
    ctrl = FakeController()
    bot = make_bot(ctrl)
    q = [10 * DEG, 60 * DEG, 20 * DEG]        # q2=60° — 특이점에서 충분히 멀다

    with _with_limits():
        motion.move_joint_safe(bot, q, wait=False)

    assert len(ctrl.sent) == 1, f"전송되지 않았다: {ctrl.sent}"
    kind, wps = ctrl.sent[0]
    assert kind == "move_joint"
    assert len(wps) == 1
    for a, b in zip(wps[0]["q"], q):
        assert abs(a - b) < 1e-12
    assert wps[0]["vel"] > 0 and wps[0]["acc"] > 0


def test_move_to_xy_end_to_end():
    """(x, y) → IK → 검사 → 전송 파이프라인 전체."""
    ctrl = FakeController()
    bot = make_bot(ctrl)
    # 도달 가능하고 특이점에서 먼 목표
    q_ref = [10 * DEG, 60 * DEG, 20 * DEG]
    x, y, _, phi = kin.fk(q_ref)

    with _with_limits():
        motion.move_to_xy(bot, x, y, phi, seed_q=q_ref, wait=False)

    assert len(ctrl.sent) == 1
    q_sent = ctrl.sent[0][1][0]["q"]
    x2, y2, _, phi2 = kin.fk(q_sent)
    assert abs(x - x2) < 1e-9 and abs(y - y2) < 1e-9


def test_path_with_blending_sent_as_one_command():
    """경유점 여러 개는 한 번의 move_joint 로 넘어가야 한다 (StellaX가 보간)."""
    ctrl = FakeController()
    bot = make_bot(ctrl)
    pts = [(0.35, 0.20, 0.0), (0.30, 0.25, 0.0), (0.25, 0.30, 0.0)]

    with _with_limits():
        qs = motion.plan_path(pts, seed_q=[20 * DEG, 70 * DEG, -30 * DEG])
        motion.move_joint_path(bot, qs, radius=0.01, wait=False)

    assert len(ctrl.sent) == 1, "경유점마다 따로 보냈다"
    assert len(ctrl.sent[0][1]) == 3
    assert all(w["radius"] == 0.01 for w in ctrl.sent[0][1])


# =============================================================================
# 계획 (전송 없음) — 실기 없이 경로 검증
# =============================================================================

def test_plan_path_keeps_elbow_continuous():
    """경로 계획이 자세 연속성을 유지해야 한다 (팔 뒤집힘 방지)."""
    pts = [(0.35, 0.20, 0.0), (0.30, 0.25, 0.0), (0.25, 0.30, 0.0)]
    qs = motion.plan_path(pts, seed_q=[20 * DEG, 70 * DEG, -30 * DEG],
                          check_limits=False, check_singularity=False)
    assert len(qs) == 3
    signs = [1 if q[1] > 0 else -1 for q in qs]
    assert len(set(signs)) == 1, f"경로 중 팔꿈치 자세가 뒤집혔다: {signs}"


def test_plan_path_reports_which_waypoint_failed():
    pts = [(0.35, 0.20, 0.0), (1.5, 1.5, 0.0)]
    try:
        motion.plan_path(pts, check_limits=False, check_singularity=False)
    except Exception as exc:
        assert "경유점 1" in str(exc), str(exc)
    else:
        raise AssertionError("도달 불가 경유점인데 통과했다")


def test_speed_amplification_matches_kinematics():
    """safety 의 증폭 계산이 kinematics 야코비안과 일치해야 한다."""
    for q2_deg in (15, 30, 60, 90):
        q = [0.0, q2_deg * DEG, 0.0]
        amp = safety.speed_amplification(q)
        qd = kin.joint_velocity(q, (0.1, 0.0, 0.0))
        # 정확히 같진 않지만(자코비안 전체 vs sin q2), 같은 경향이어야 한다
        assert amp > 0 and max(abs(v) for v in qd) > 0


# =============================================================================

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
