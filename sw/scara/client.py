"""StellaX 접속 래퍼.

`stellarx_remote.StellarXControlClient` 는 `ClientJSONRPCoverTCP` 를 상속한
JSON-RPC over TCP 클라이언트다. 생성자에서

  1. `create_control_handler` 로 서버에 컨트롤 핸들러 인스턴스를 만들고
  2. `get_available_commands` 로 커맨드 목록을 받아
  3. `setattr` 로 **동적으로 파이썬 메서드를 붙인다**

동적 바인딩이라 오타가 import 시점에 안 잡히고, 컨트롤러 버전에 따라
커맨드 이름이 달라질 수 있다. 그래서 여기서 얇게 감싸
(1) 접속·재시도 (2) 반환값 규약 처리 (3) 커맨드 존재 확인 (4) 로깅 을 한다.

반환값 규약: 모든 커맨드의 **첫 원소가 상태 플래그**, 실제 값은 index 1.
모든 커맨드는 **키워드 인자만** 받는다.

SDK 소스: `vendor/stellarx_remote/stellarx_remote.py`
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path
from typing import Any, Optional, Sequence

from . import config

log = logging.getLogger(__name__)

#: `stellarx_remote.py` 는 pip 패키지가 아니라 단일 모듈이라 sys.path 에 직접 넣어야 한다.
#: (`stxlib` 만 pip 설치 대상)
_VENDOR_DIR = Path(__file__).resolve().parents[2] / "vendor" / "stellarx_remote"


class StellaXError(RuntimeError):
    """StellaX 호출이 실패했을 때."""


def _import_client():
    """`StellarXControlClient` 를 가져온다."""
    if _VENDOR_DIR.is_dir() and str(_VENDOR_DIR) not in sys.path:
        sys.path.insert(0, str(_VENDOR_DIR))
    try:
        from stellarx_remote import StellarXControlClient  # type: ignore
        return StellarXControlClient
    except ImportError as exc:
        raise ImportError(
            f"StellarXControlClient 를 가져오지 못했다 ({exc}).\n"
            f"기대 경로: {_VENDOR_DIR}\n"
            "설치:\n"
            "  cd SCARA/vendor\n"
            "  git clone https://github.com/Sodero-labs/stellarx_remote.git\n"
            "  git clone https://github.com/Sodero-labs/stxlib.git\n"
            "  python -m pip install ./stxlib\n"
        ) from exc


def unwrap(result: Any, call: str = "") -> Any:
    """`[status, value]` 규약을 풀어서 value 만 돌려준다.

    - bool 하나만 오는 함수(set_* 계열)는 그대로 True/False 판정
    - 실패면 StellaXError
    """
    if isinstance(result, bool):
        if not result:
            raise StellaXError(f"{call} 실패 (False 반환)")
        return True

    if isinstance(result, (list, tuple)) and result:
        ok = result[0]
        if ok is False or ok == 0:
            raise StellaXError(f"{call} 실패: {result!r}")
        return result[1] if len(result) == 2 else list(result[1:])

    if result is None:
        raise StellaXError(f"{call} 가 None 을 반환했다 (통신 실패 가능)")

    return result


class Scara:
    """SCARA 제어용 얇은 파사드.

    사용 예:
        with Scara() as bot:
            print(bot.joint_positions())
    """

    def __init__(
        self,
        host: str = config.STELLARX_HOST,
        port: int = config.STELLARX_RPC_PORT,
        name: str = config.ROBOT_NAME,
        connect: bool = True,
        retries: int = 3,
        retry_interval_s: float = 1.0,
    ):
        self.host = host
        self.port = port
        self.name = name
        self._raw = None
        self._commands: tuple = ()
        if connect:
            self.connect(retries=retries, retry_interval_s=retry_interval_s)

    # -- 연결 -----------------------------------------------------------
    def connect(self, retries: int = 3, retry_interval_s: float = 1.0) -> None:
        cls = _import_client()
        last_exc = None
        for attempt in range(1, retries + 1):
            log.info("StellaX 접속 시도 %d/%d: %s:%d", attempt, retries, self.host, self.port)
            try:
                self._raw = cls(
                    name=self.name,
                    ip=self.host,
                    port=self.port,
                    category=config.ROBOT_CATEGORY,
                    manufacturer=config.ROBOT_MANUFACTURER,
                    product=config.ROBOT_PRODUCT,
                    model=config.ROBOT_MODEL,
                    params=config.robot_params(),
                )
                break
            except ValueError as exc:
                # SDK는 인스턴스 생성/커맨드 조회 실패 시 ValueError 를 던진다
                last_exc = exc
                log.warning("접속 실패: %s", exc)
                if attempt < retries:
                    time.sleep(retry_interval_s)
        else:
            raise StellaXError(
                f"{self.host}:{self.port} 접속 실패 ({retries}회 시도). 마지막 오류: {last_exc}\n"
                "확인할 것:\n"
                "  - StellaX 전원, Eth1(10.0.0.1/24) 직결\n"
                "  - PC IP가 같은 대역인지 (예: 10.0.0.2/24)\n"
                f"  - RPC 포트 {self.port} 가 열려 있는지"
            )

        self._commands = tuple(sorted(
            n for n in vars(self._raw) if callable(getattr(self._raw, n, None))
        ))
        log.info("접속 완료 — 커맨드 %d개 바인딩됨", len(self._commands))
        self._check_units()

    def _check_units(self) -> None:
        """컨트롤러의 단위 설정을 기록한다.

        RPC API 는 항상 SI(rad, m) 다.

        `get_processing_unit` 이 `'DEGREE_MM'` 을 돌려주지만 이는 **웹 UI 표시용**이고
        RPC 값에는 영향이 없다. 검증:

            solve_forward_kinematics(target_q=[10°, 60°, 20°] 를 라디안으로)
              → [0.3317069740844692, 0.42833519961320965, -0.044, 0, 0, 1.5707963267948966]
            kinematics.fk_pose(같은 값)
              → [0.331707,           0.428335,            -0.044, 0, 0, 1.570796]

        소수점 6자리까지 일치. 같은 입력을 도(°)로 주면 전혀 다른 값이 나온다.

        단위를 바꾸지는 않는다 (전역 설정이라 웹 UI에 영향을 준다).
        """
        self.processing_unit = None
        if not self.has("get_processing_unit"):
            return
        try:
            self.processing_unit = unwrap(self._call("get_processing_unit"),
                                          "get_processing_unit")
        except StellaXError as exc:
            log.debug("단위 조회 실패: %s", exc)
            return
        log.debug("컨트롤러 표시 단위: %s (RPC는 rad/m)", self.processing_unit)

    @property
    def commands(self) -> tuple:
        """컨트롤러가 실제로 노출한 커맨드 이름 목록 (동적 바인딩 결과)."""
        return self._commands

    def has(self, name: str) -> bool:
        return name in self._commands

    def close(self) -> None:
        close = getattr(self._raw, "close", None)
        if callable(close):
            close()
        self._raw = None

    def __enter__(self) -> "Scara":
        if self._raw is None:
            self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def raw(self):
        """SDK 객체 직접 접근. 래핑되지 않은 API를 쓸 때만."""
        if self._raw is None:
            raise StellaXError("연결되지 않았다. connect() 를 먼저 호출할 것.")
        return self._raw

    def _call(self, name: str, **kwargs) -> Any:
        fn = getattr(self.raw, name, None)
        if fn is None:
            near = [c for c in self._commands if name.split("_")[0] in c]
            raise StellaXError(
                f"컨트롤러에 '{name}' 커맨드가 없다. "
                f"동적 바인딩이라 버전에 따라 이름이 다를 수 있다.\n"
                f"비슷한 이름: {near or '없음'}\n"
                f"전체 목록은 bot.commands 로 확인할 것."
            )
        log.debug("call %s(%s)", name, kwargs)
        return fn(**kwargs)  # Sodero 커맨드는 키워드 인자만 받는다

    # -- 상태 -----------------------------------------------------------
    def is_ready(self) -> bool:
        """비상정지·충돌 없이 동작 가능한 상태인가."""
        try:
            return bool(unwrap(self._call("is_ready"), "is_ready"))
        except StellaXError:
            return False

    def status(self) -> dict:
        return unwrap(self._call("get_status"), "get_status")

    def error_status(self) -> dict:
        return unwrap(self._call("get_error_status"), "get_error_status")

    def alarm(self) -> dict:
        return unwrap(self._call("get_alarm"), "get_alarm")

    def reset_trigger(self) -> bool:
        """비상정지·충돌로 걸린 에러를 해제."""
        return unwrap(self._call("reset_trigger"), "reset_trigger")

    # -- 서보 -----------------------------------------------------------
    def servo(self, on: bool) -> bool:
        log.info("servo %s", "ON" if on else "OFF")
        return unwrap(self._call("set_servo_on_off", state=on), "set_servo_on_off")

    # -- 관측 -----------------------------------------------------------
    @staticmethod
    def _trim(values) -> list:
        """컨트롤러는 관절 배열을 **7칸**으로 돌려주고 뒤를 0으로 채운다.
        이 로봇은 3축이므로 앞 DOF 개만 남긴다.

        실측: get_actual_joint_position() → [0.118, 0.102, 0.111, 0.0, 0.0, 0.0, 0.0]
        """
        if not isinstance(values, list):
            return values
        return values[: config.DOF]

    def joint_positions(self) -> list:
        """현재 관절각 [rad]. 길이 DOF."""
        return self._trim(unwrap(self._call("get_actual_joint_position"),
                                 "get_actual_joint_position"))

    def joint_velocities(self) -> list:
        """현재 관절 각속도 [rad/s]. 길이 DOF."""
        return self._trim(unwrap(self._call("get_actual_joint_velocity"),
                                 "get_actual_joint_velocity"))

    def joint_efforts(self) -> list:
        """현재 관절 토크 [N·m]. 길이 DOF. 토크 한계 감시에 쓴다."""
        return self._trim(unwrap(self._call("get_actual_joint_effort"),
                                 "get_actual_joint_effort"))

    def residual_efforts(self) -> list:
        """외란 토크 [N·m]. 길이 DOF. 충돌 감지 튜닝에 유용."""
        return self._trim(unwrap(self._call("get_residual_effort"),
                                 "get_residual_effort"))

    def target_joint_positions(self) -> list:
        """지령 관절각 [rad]. 추종 오차 확인용."""
        return self._trim(unwrap(self._call("get_target_joint_position"),
                                 "get_target_joint_position"))

    def pose(self) -> list:
        """TCP 포즈 [x, y, z, rx, ry, rz] (m, rad). 6칸이라 자르지 않는다."""
        return unwrap(self._call("get_actual_pose"), "get_actual_pose")

    # -- 동작 -----------------------------------------------------------
    def move_joint(self, waypoints: Sequence[dict]):
        """관절 공간 이동. waypoint = {'q': [...], 'vel': ..., 'acc': ..., 'radius': ...}

        직접 쓰지 말고 `motion.py` 의 검증 래퍼를 통할 것.
        """
        return unwrap(self._call("move_joint", waypoints=list(waypoints)), "move_joint")

    def move_linear(self, waypoints: Sequence[dict]):
        return unwrap(self._call("move_linear", waypoints=list(waypoints)), "move_linear")

    def move_jog(self, mode, type, index, direction, speed_level):
        return unwrap(
            self._call("move_jog", mode=mode, type=type, index=index,
                       direction=direction, speed_level=speed_level),
            "move_jog",
        )

    def stop(self, ramp_time: float = -1) -> bool:
        return unwrap(self._call("stop", ramp_time=ramp_time), "stop")

    def check_finish(self, finish_mode: str = "FINE") -> bool:
        return unwrap(self._call("check_finish", finish_mode=finish_mode), "check_finish")

    def wait_until_done(self, timeout_s: float = 30.0, poll_s: float = 0.05) -> None:
        """동작 완료까지 대기. 타임아웃이면 정지시키고 예외."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.check_finish():
                return
            time.sleep(poll_s)
        self.stop()
        raise StellaXError(f"동작이 {timeout_s}s 안에 끝나지 않아 정지시켰다")

    # -- 기구학 (StellaX 내장) ------------------------------------------
    #
    # 이 두 커맨드는 응답이 이중으로 감싸여 돌아온다:
    #       [True, [True, [x, y, z, rx, ry, rz]]]
    #   바깥은 통신 상태, 안쪽은 연산 성공 여부다. 두 번 벗겨야 한다.
    #
    # 입력 단위는 라디안이다 (get_processing_unit 이 DEGREE_MM 이어도).

    @staticmethod
    def _unwrap_twice(res, call: str):
        inner = unwrap(res, call)
        if isinstance(inner, list) and len(inner) == 2 and isinstance(inner[0], (bool, int)):
            return unwrap(inner, call)
        return inner

    def fk(self, q: Sequence[float], tcp: Optional[Sequence[float]] = None) -> list:
        """정기구학. q [rad] → [x, y, z, rx, ry, rz] (m, rad).

        `kinematics.fk_pose()` 와 소수점 6자리까지 일치하는 것을 확인했다.
        """
        return self._unwrap_twice(
            self._call("solve_forward_kinematics", target_q=list(q), tcp=tcp),
            "solve_forward_kinematics")

    def ik(self, p: Sequence[float], seed_q=None, tcp=None) -> list:
        """역기구학. p [m, rad] → q [rad].

        FK 와 상태 규약이 다르다. 안쪽 첫 원소가 정수 결과코드이며 0 이 성공이다.
          (FK는 bool True 가 성공) API 문서의 `list[int, list[float]|None]` 과 일치.

          실측: [0, [0.11799680575053043, 0.10158867912276714, 0.11061776952563371, 0,0,0,0]]
        """
        inner = unwrap(self._call("solve_inverse_kinematics", target_p=list(p),
                                  seed_q=list(seed_q) if seed_q is not None else None,
                                  tcp=tcp),
                       "solve_inverse_kinematics")
        if not (isinstance(inner, list) and len(inner) == 2):
            raise StellaXError(f"solve_inverse_kinematics 응답 형식이 예상과 다르다: {inner!r}")
        code, q = inner
        if code != 0 or q is None:
            raise StellaXError(
                f"역기구학 해 없음 (결과코드 {code}). target_p={list(p)}")
        return self._trim(q)

    # -- 모드 -----------------------------------------------------------
    def set_operational_mode(self, mode: str) -> bool:
        """'AUTO' 또는 'MANUAL'. MANUAL은 TCP 속도가 250 mm/s로 제한된다."""
        return unwrap(self._call("set_operational_mode", operational_mode=mode),
                      "set_operational_mode")

    def set_feed_rate(self, rate: float) -> bool:
        """0.1 ~ 1.5. 시운전 중에는 낮게."""
        return unwrap(self._call("set_feed_rate", feed_rate=rate), "set_feed_rate")
