"""SCARA — 실기 하나를 대표하는 고수준 객체.

    상태   q() / q_deg() / pose() / tcp_mm() / clearance() / state()
    진단   health()        접속·에러·제어루프·회전방향
           verify_zero()   일직선 명령 -> 사람 눈 확인
    이동   home() / move_q_deg() / move_xy() / move_rel() / stop() / servo()
    영점   jog_deg() / declare_zero() / zero_looks_valid()
    도달성 reachable(x, y)

모든 이동이 같은 파이프라인을 지난다:
    가동범위 -> 특이점 -> CAD 자기충돌(경로 중간 포함) -> TCP 속도 상한
    -> 공진 회피 가속

사용:
    from scara.robot import SCARA
    bot = SCARA()
    bot.move_xy(400, 100)      # mm. phi 는 자동 (한계·충돌 피해서 고른다)
    bot.home()
    bot.close()

이 클래스는 이동 전에 사람에게 묻지 않는다. 확인이 필요하면 호출하는 쪽
(main.py, ui.py)이 묻는다.
"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

from . import collision, config, motion, safety
from .client import Scara as _Client
from .safety import SafetyViolation

D = math.pi / 180.0
R = 180.0 / math.pi

BACKUP_DIR = Path(__file__).resolve().parents[1] / "zero_backups"


class SCARA:

    def __init__(self,
                 speed_fraction: float = 0.15,
                 acc_ratio: float = motion.ACC_RATIO_ZERO_VIB,
                 auto_health: bool = True):
        #: 정격 대비 관절속도 비율. TCP 속도 상한이 이보다 낮으면 그쪽이 이긴다.
        self.speed_fraction = speed_fraction
        #: 가속 배수. 기본값은 공진 여기율이 0 이 되는 비율이다.
        self.acc_ratio = acc_ratio
        self._c = _Client()
        if auto_health:
            problems = self.health()
            if problems:
                raise SafetyViolation("기동 전 점검 실패: " + "; ".join(problems))

    # ------------------------------------------------------------------ 상태
    def q(self) -> list:
        """관절각 [rad]."""
        return self._c.joint_positions()

    def q_deg(self) -> list:
        return [v * R for v in self.q()]

    def pose(self) -> list:
        """TCP [x, y, z, rx, ry, rz] (m, rad)."""
        return self._c.pose()

    def tcp_mm(self) -> tuple:
        p = self.pose()
        return (p[0] * 1000.0, p[1] * 1000.0)

    def clearance(self) -> tuple:
        """현재 자세의 최소 충돌 틈새 (m, (몸체A, 몸체B))."""
        return collision.clearance(self.q())

    def state(self) -> dict:
        q = self.q()
        p = self.pose()
        d, pair = collision.clearance(q)
        return {
            "q_deg": [round(v * R, 3) for v in q],
            "tcp_mm": (round(p[0] * 1000, 2), round(p[1] * 1000, 2)),
            "phi_deg": round(p[5] * R, 2),
            "clearance_mm": round(d * 1000, 1),
            "clearance_pair": pair,
            "servo_on": self._c.status()["servo_status"]["servo_on"],
        }

    # ------------------------------------------------------------------ 진단
    def health(self) -> list:
        """문제 목록. 비어 있으면 정상."""
        problems = safety.startup_checklist(self._c)
        try:
            safety.check_direction_sign()
        except SafetyViolation as exc:
            problems.append(str(exc))
        return problems

    def verify_zero(self, ask=input) -> bool:
        """영점 신뢰성 검사 — 일직선으로 보낸 뒤 사람이 눈으로 확인한다.

        영점 상실은 소프트웨어가 스스로 잡을 수 없다. 컨트롤러는 언제나
        "지령대로 갔다" 고 보고하기 때문이다(자기 좌표계 안의 순환논리).
        사람 눈이 유일한 독립 측정기다.
        """
        self.home()
        a = ask("팔이 한 직선으로 완전히 펴져 있는가? [y/n] > ").strip().lower()
        return a.startswith("y")

    # ------------------------------------------------------------------ 이동
    def servo(self, on: bool) -> None:
        self._c.servo(on)

    def stop(self) -> None:
        self._c.stop()

    def prepare(self) -> None:
        """이동 직전 준비: MANUAL 모드 확인 + 서보 ON."""
        st = self._c.status().get("operation_status", {})
        if not st.get("manual_mode"):
            self._c.set_operational_mode("MANUAL")
        self._c.servo(True)

    def _vel(self, q_from, q_to) -> float:
        vel = min(safety.rated_joint_speeds()) * self.speed_fraction
        return min(vel, motion.tcp_speed_cap(q_from, q_to))

    def move_q_deg(self, q_deg: Sequence[float], wait: bool = True):
        """관절각 [deg] 로 이동."""
        q_t = [v * D for v in q_deg]
        q_now = self.q()
        vel = self._vel(q_now, q_t)
        self.prepare()
        return motion.move_joint_safe(self._c, q_t, vel=vel,
                                      acc=vel * self.acc_ratio, wait=wait)

    def home(self, wait: bool = True):
        """일직선 자세 q=[0,0,0] 로."""
        return self.move_q_deg([0.0, 0.0, 0.0], wait=wait)

    def move_xy(self, x_mm: float, y_mm: float,
                phi_deg: Optional[float] = None, wait: bool = True) -> dict:
        """EE 를 (x, y) [mm] 로. phi_deg=None 이면 자동 선택.

        Returns: {'q_deg', 'phi_deg', 'err_mm'}
        """
        x, y = x_mm / 1000.0, y_mm / 1000.0
        q_now = self.q()
        if phi_deg is None:
            phi, _ = motion.auto_phi(x, y, q_now)
            if phi is None:
                raise SafetyViolation(
                    f"({x_mm:.0f}, {y_mm:.0f}) mm: 어떤 phi 로도 "
                    "한계·특이점·충돌 조건을 만족하는 해가 없다.")
        else:
            phi = phi_deg * D
        q_t = motion.plan_joint_target(x, y, phi, seed_q=q_now)
        vel = self._vel(q_now, q_t)
        self.prepare()
        motion.move_joint_safe(self._c, q_t, vel=vel,
                               acc=vel * self.acc_ratio, wait=wait)
        p = self.pose()
        return {"q_deg": [round(v * R, 3) for v in q_t],
                "phi_deg": round(phi * R, 2),
                "err_mm": round(math.hypot(p[0] - x, p[1] - y) * 1000, 3)}

    def move_rel(self, dx_mm: float, dy_mm: float, **kw) -> dict:
        x, y = self.tcp_mm()
        return self.move_xy(x + dx_mm, y + dy_mm, **kw)

    def reachable(self, x_mm: float, y_mm: float) -> bool:
        """어떤 phi 로든 갈 수 있는가 (한계·특이점·충돌 전부 통과)."""
        phi, _ = motion.auto_phi(x_mm / 1000.0, y_mm / 1000.0, self.q())
        return phi is not None

    # ------------------------------------------------------------ 영점 설정
    #: 한 번에 허용하는 조그 각도 [deg]
    JOG_MAX_DEG = 10.0
    #: 조그 속도 (정격 대비). 눈으로 따라갈 수 있게 느리게.
    JOG_SPEED_FRACTION = 0.02
    #: 조그 중 토크 상한 [N·m]. 감속기 한계(29)가 아니라 실측 주행토크
    #: (config.RUNNING_TORQUE_NM, 최대 9.5) 의 1.5배쯤이다.
    #: 드라이버 과부하(Err16.0)가 트립하기 전에 먼저 멈추는 것이 목적이다.
    JOG_TORQUE_LIMIT_NM = 15.0
    #: 이 시간 동안 움직임이 없으면 막힌 것으로 보고 정지 [s]
    JOG_STALL_SEC = 1.0

    def zero_looks_valid(self, tol_deg: float = 0.05) -> bool:
        """컨트롤러가 q ~ 0 을 읽고 있는가.

        이것만으로는 영점을 보증하지 못한다(자기 좌표계 안의 순환논리).
        팔이 물리적으로 일직선인지는 사람이 봐야 한다.
        """
        return max(abs(v) for v in self.q()) * R < tol_deg

    def jog_deg(self, joint: int, delta_deg: float) -> bool:
        """한 축만 작게 움직인다. 영점 설정용.

        영점이 불명확할 때도 써야 하므로 충돌 검사에 의존하지 않는다.
        영점이 틀리면 검사의 입력 q 자체가 틀려서 검사가 무의미하기 때문이다.
        대신 한 번에 JOG_MAX_DEG 이하, 정격의 2% 속도, 토크·정체 감시로 막는다.
        """
        if abs(delta_deg) > self.JOG_MAX_DEG:
            raise SafetyViolation(
                f"한 번에 {self.JOG_MAX_DEG}도 넘게 조그하지 않는다 "
                f"(요청 {delta_deg:+.2f}도). 나눠서 할 것.")
        if delta_deg == 0.0:
            return False
        j = config.JOINTS[joint]
        q0 = self.q()
        target = list(q0)
        target[joint] += delta_deg * D
        safety.check_joint_limits(target)

        vel = safety.clamp_speed(
            [j.rated_joint_speed_rad_s * self.JOG_SPEED_FRACTION] * config.DOF)[joint]
        self.prepare()
        self._c.move_joint([{"q": target, "vel": vel,
                             "acc": vel * 2, "radius": 0.0}])

        deadline = time.monotonic() + max(5.0, abs(delta_deg) / (vel * R) * 3 + 3.0)
        last_q = q0[joint]
        last_progress = time.monotonic()
        stalled = False
        try:
            while time.monotonic() < deadline:
                if self._c.check_finish():
                    break
                now = time.monotonic()
                try:
                    tq = abs(self._c.joint_efforts()[joint])
                    if tq > self.JOG_TORQUE_LIMIT_NM:
                        self._c.stop()
                        raise SafetyViolation(
                            f"{j.name} 토크 {tq:.1f} N·m 가 조그 한계 "
                            f"{self.JOG_TORQUE_LIMIT_NM:.0f} N·m 를 넘었다. "
                            "축이 무언가에 막혀 있을 수 있다.")
                    cur = self._c.joint_positions()[joint]
                    if abs(cur - last_q) * R > 0.02:      # 움직이고 있으면 갱신
                        last_q, last_progress = cur, now
                    elif now - last_progress > self.JOG_STALL_SEC:
                        stalled = True
                        break
                except SafetyViolation:
                    raise
                except Exception:
                    pass
                time.sleep(0.02)
            else:
                self._c.stop()
                raise SafetyViolation("조그가 제한 시간 안에 끝나지 않아 정지했다.")
        finally:
            if stalled:
                self._c.stop()
        if stalled:
            raise SafetyViolation(
                f"{j.name} 이 {self.JOG_STALL_SEC:.1f}초 동안 움직이지 않아 정지했다. "
                "계속 밀면 모터 과부하(Err16.0)가 난다. "
                "축이 기구에 막혔는지, 반대 방향인지 확인할 것.")
        moved = (self.q()[joint] - q0[joint]) * R
        return abs(moved) >= abs(delta_deg) * 0.1

    def declare_zero(self) -> dict:
        """지금 이 물리적 자세를 q=[0,0,0] 으로 선언한다.

        사람이 팔을 일직선으로 맞춰 둔 상태에서 부를 것.

        서보를 끄면 팔이 조금 흘러내린다. 흘러내린 자세를 그대로 0 으로 박으면
        영점이 그만큼 틀어지므로, 서보 OFF 전후 각도를 둘 다 읽어 그 차이(drift)를
        보정해 박는다. 그러면 사람이 맞춰둔 자세가 정확히 0 이 된다.

        이 보정은 런타임 오프셋이다. Designer 에서 설정을 바꿔 적용하면 프로젝트의
        옛 값으로 되돌아간다. 영구 저장은 Motion Studio 의 origin calibration
        (Encoder Offset) 뿐이다.

        적용 직후 드라이버가 EtherCAT OP 에서 이탈한다 (docs/controller.md 참조).
        호출한 쪽이 health() 로 확인하고 Designer 재연결을 안내해야 한다.
        """
        q_before = self.q()
        BACKUP_DIR.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        (BACKUP_DIR / f"zero_backup_{stamp}.json").write_text(json.dumps({
            "timestamp": stamp,
            "note": "declare_zero() 적용 직전의 원시 관절각",
            "q_before_rad": q_before,
            "q_before_deg": [v * R for v in q_before],
        }, indent=2, ensure_ascii=False), encoding="utf-8")

        self._c.servo(False)
        time.sleep(1.2)
        drift = [a - b for a, b in zip(self.q(), q_before)]
        for i in range(config.DOF):
            self._c.raw.calibrate_joint_angle(joint_index=i, target_value=drift[i])
        time.sleep(0.8)

        if max(abs(v) for v in drift) * R > 0.02:
            self.move_q_deg([0.0, 0.0, 0.0])       # 흘러내린 만큼만 되돌린다
        p = self.pose()
        return {"drift_deg": [round(v * R, 4) for v in drift],
                "q_deg": [round(v * R, 4) for v in self.q()],
                "tcp_mm": (round(p[0] * 1000, 2), round(p[1] * 1000, 2)),
                "backup": stamp}

    # ------------------------------------------------------------------ 수명
    def close(self) -> None:
        """정지 -> 정지 확인 -> 서보 OFF -> 연결 해제.

        순서와 대기가 둘 다 중요하다. stop() 은 감속을 시작만 하므로,
        check_finish() 로 실제로 멈춘 것을 확인한 뒤에 서보를 꺼야 한다.
        궤적이 살아 있는 채로 서보를 떨구면 드라이버가 과부하 폴트를 낸다.
        """
        try:
            self._c.stop()
        except Exception:
            pass
        try:
            for _ in range(20):                    # 최대 2초
                if self._c.check_finish():
                    break
                time.sleep(0.1)
        except Exception:
            pass
        try:
            self._c.servo(False)
        except Exception:
            pass
        time.sleep(0.2)                            # 서보 OFF 반영 여유
        self._c.close()

    def __enter__(self) -> "SCARA":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def raw(self):
        """저수준 클라이언트 (client.Scara)."""
        return self._c
