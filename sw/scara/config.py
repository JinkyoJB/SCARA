"""하드웨어 상수 — 단일 출처.

근거는 docs/hardware.md. 값을 바꾸면 그 문서도 같이 고칠 것.

미확정 값은 PENDING 문자열로 둔다. 0이나 추정치로 채우면 조용히 틀린 채
움직이므로, 쓰는 쪽에서 assert_resolved() 로 막는다.
"""

from dataclasses import dataclass

import math

DEG = math.pi / 180.0

PENDING = "PENDING"


# =============================================================================
# 기구
# =============================================================================

DOF = 3

#: DH 링크 길이 a [m]. 감속기 출력 플랜지 원 피팅으로 CAD 검증 (오차 < 0.05 mm)
LINK_LENGTHS_M = (0.250, 0.250, 0.150)

#: 최대 도달반경 [m]
MAX_REACH_M = sum(LINK_LENGTHS_M)  # 0.650

#: DH alpha [rad]. 세 관절축이 전부 연직·평행이라 0.
DH_ALPHA_RAD = (0.0, 0.0, 0.0)

#: DH theta 오프셋 [rad]
DH_THETA_OFFSET_RAD = (0.0, 0.0, 0.0)

#: DH d (높이 오프셋) [m]. 기준면 = 각 관절 출력 플랜지면(= 구동되는 링크 판재 상면).
#: d3(툴)은 엔드이펙터가 정해져야 결정된다.
DH_D_M = (-0.0235, -0.0205, PENDING)

#: CAD 실측 높이 [m]. 베이스 판 바닥 = 0, 링크 판재 상면 기준.
LINK_TOP_SURFACE_Z_M = (0.2155, 0.1920, 0.1715)

#: 설치면 -> 좌표계0 (링크1 상면)
BASE_HEIGHT_M = 0.2155


# =============================================================================
# 질량 특성 — DH 좌표계 기준
#
# HW/scara.yaml 원본을 그대로 쓰면 안 된다. yaml 의 질량특성 좌표계는
# (x=좌우, y=링크방향, z=연직), DH 는 (x=링크방향, y=좌우, z=연직) 이다.
# 아래는 Rz(+90도) 를 적용한 값이다. 부호는 CAD 기하로 판별했다
# (Rz(-90도) 면 세 링크 모두 COM 이 링크 밖으로 나가 물리적으로 불가능).
# =============================================================================

#: 링크 질량 [kg]. 표면 메시로는 검증이 안 돼 SolidWorks 추출값을 쓴다.
LINK_MASS_KG = (2.920, 2.499, 0.667)

#: COM [m]. 각 링크의 말단 좌표계(관절 i+1) 기준.
#: x 가 음수인 것은 좌표계가 링크 말단에 있고 몸체가 -x 로 뻗기 때문이다.
LINK_COM_M = (
    (-0.07499,  0.00000,  0.04726),
    (-0.06080, -0.00001,  0.05332),
    (-0.05990, -0.00001, -0.15044),
)

#: 관성텐서 [kg·m^2]. COM 기준, DH 좌표계.
LINK_INERTIA_KGM2 = (
    ((0.007602522, -0.000001333,  0.007515059),
     (-0.000001333, 0.032248307, -0.000000398),
     (0.007515059, -0.000000398,  0.026770962)),
    ((0.006374465, -0.000001126,  0.005652317),
     (-0.000001126, 0.025848809, -0.000000295),
     (0.005652317, -0.000000295,  0.020919796)),
    ((0.005118446, -0.000000376,  0.002459577),
     (-0.000000376, 0.007879923, -0.000000805),
     (0.002459577, -0.000000805,  0.003004316)),
)


# =============================================================================
# 구동계
# =============================================================================

@dataclass(frozen=True)
class JointDrive:
    name: str
    motor_pn: str
    reducer_pn: str
    gear_ratio: float
    has_brake: bool

    # 모터 단품 사양
    motor_rated_torque_nm: float = 0.32
    motor_peak_torque_nm: float = 0.95
    motor_rated_speed_rpm: float = 3000.0
    motor_max_speed_rpm: float = 6000.0

    # 감속기 출력측 한계
    reducer_rated_torque_nm: float = 0.0
    reducer_peak_torque_nm: float = 0.0
    reducer_max_output_rpm: float = 0.0

    @property
    def reduction_ratio(self) -> float:
        """StellaX Actuator Mapping 에 넣는 값 = 기어비의 역수.

        회전 방향 반전이 필요하면 이 값의 부호로 한다 (DIRECTION_SIGN 참조).
        """
        return 1.0 / self.gear_ratio

    @property
    def continuous_torque_limit_nm(self) -> float:
        """연속 출력 토크 한계 [N·m].

        모터 정격 x 기어비 와 감속기 정격 중 작은 쪽이 진짜 한계다.
        감속기 효율은 1.0 으로 낙관 가정한다 (실제로는 더 낮다).
        """
        return min(self.motor_rated_torque_nm * self.gear_ratio,
                   self.reducer_rated_torque_nm)

    @property
    def peak_torque_limit_nm(self) -> float:
        return min(self.motor_peak_torque_nm * self.gear_ratio,
                   self.reducer_peak_torque_nm)

    @property
    def max_joint_speed_rad_s(self) -> float:
        """출력축 최대 각속도 [rad/s].

        모터 최고회전수 / 기어비 와 감속기 허용 출력회전수 중 작은 쪽.
        이 기계는 둘이 정확히 같아 여유가 0 이다 (6000/100 = 60 rpm = 감속기 한계).
        반드시 클램프할 것.
        """
        from_motor = self.motor_max_speed_rpm / self.gear_ratio
        limit_rpm = min(from_motor, self.reducer_max_output_rpm)
        return limit_rpm * 2.0 * math.pi / 60.0

    @property
    def rated_joint_speed_rad_s(self) -> float:
        rpm = self.motor_rated_speed_rpm / self.gear_ratio
        return rpm * 2.0 * math.pi / 60.0


JOINTS = (
    JointDrive(
        name="J1",
        motor_pn="MSMF012L1T2",          # T2 = 브레이크 있음
        reducer_pn="PQ003HS-100",
        gear_ratio=100.0,
        has_brake=True,
        reducer_rated_torque_nm=29.0,
        reducer_peak_torque_nm=102.0,
        reducer_max_output_rpm=60.0,
    ),
    JointDrive(
        name="J2",
        motor_pn="MSMF012L1S2",
        reducer_pn="PQ003HS-100",
        gear_ratio=100.0,
        has_brake=False,
        reducer_rated_torque_nm=29.0,
        reducer_peak_torque_nm=102.0,
        reducer_max_output_rpm=60.0,
    ),
    JointDrive(
        name="J3",
        motor_pn="MSMF012L1S2",
        reducer_pn="PQ003MS-50.5",
        gear_ratio=50.5,
        has_brake=False,
        reducer_rated_torque_nm=28.0,
        reducer_peak_torque_nm=98.0,
        reducer_max_output_rpm=120.0,
    ),
)

#: 23bit 앱솔루트 엔코더 (배터리 백업 다회전)
ENCODER_BITS = 23
ENCODER_COUNTS_PER_REV = 1 << ENCODER_BITS  # 8,388,608


# =============================================================================
# 회전 방향
# =============================================================================

#: 관절 회전 방향 부호. +1 = 규약대로 (+q 가 위에서 볼 때 반시계).
#:
#: 런타임 코드는 이 값을 읽지 않는다. 방향 반전은 컨트롤러 한 곳에서만 해야
#: 하기 때문이다. kinematics.py 와 StellaX 내장 FK/IK 가 같은 규약을 공유하므로
#: (소수점 6자리 일치 확인), 파이썬에서만 부호를 뒤집으면 컨트롤러 내장 IK,
#: Designer 조그 버튼, 티치펜던트가 전부 반대로 남는다.
#:
#: 반전 위치: Designer -> Step 3 -> Joint Mapping -> Transmission Parameters
#: -> Reduction ratio 의 부호. 드라이버 607Eh Polarity 로 뒤집으면 Isaac Sim
#: USD 에 반영할 근거가 코드에 남지 않는다.
#:
#: J2/J3 가 CW 로 돌던 것을 이 방법으로 반전해 세 축 모두 CCW 로 맞췄다.
#: 부호가 틀리면 TCP 가 수백 mm 어긋난다. 실제로 (500, 0) 지령이
#: (96.6, 490.6) 으로 갔다 — 635 mm 오차.
#:
#: (1,1,1) 이 아니면 safety.check_direction_sign() 이 모든 이동을 막는다.
#: 방향을 다시 건드렸으면 tools/verify_model.py 로 축별 확인 후 갱신할 것.
DIRECTION_SIGN = (1, 1, 1)


# =============================================================================
# 실측 기준선
# =============================================================================

#: 무부하 최대 관절측 보고 토크 [N·m] (J1, J2, J3). 감속기 미장착, 모터 단독.
#: 조건: 지령 10도, 정격속도의 10%.
NO_LOAD_TORQUE_NM = (0.704, 0.896, 0.388)

#: 위 값을 기어비로 나눈 모터축 실제 토크 [N·m]. 정격(0.32)의 2.2~2.8%.
#: 세 축이 0.007~0.009 로 거의 같아 신뢰도가 높다.
NO_LOAD_MOTOR_TORQUE_NM = (0.00704, 0.00896, 0.00768)

#: 위치 추종 오차 [deg] — 10도 지령 대비. 참고용 기준선.
NO_LOAD_POSITION_ERROR_DEG = (0.000, -0.002, +0.005)

#: 조립 후 실측 주행 토크 [N·m]. 조그 토크 가드의 근거값이다.
#: 감속기 연속한계(29 N·m)를 기준으로 문턱을 잡으면 영영 안 걸린다.
RUNNING_TORQUE_NM = (6.2, 9.5, 4.3)


# =============================================================================
# 가동범위
# =============================================================================

#: 컨트롤러(Designer)에 설정된 관절 한계 [rad]. 컨트롤러가 이 값을 강제하므로
#: 소프트웨어가 이보다 넓게 잡는 것은 무의미하다.
#: `tools/dump_config.py` 로 실기에서 읽은 값과 맞춰 둘 것.
CONTROLLER_JOINT_LIMITS_RAD = (
    (-150 * DEG, 150 * DEG),
    (-115 * DEG, 115 * DEG),
    (-155 * DEG, 155 * DEG),
)

#: 자기충돌이 절대 일어나지 않는 최대 대칭 박스 [deg]. `tools/collision_map.py` 산출.
#:
#: 이 박스 안에서는 (q2, q3) 어떤 조합도 여유 10 mm 를 지킨다. 경계는 실제
#: 경계다 — J3 를 156도로 한 칸만 넓혀도 최소 틈새가 8.0 mm 로 떨어진다.
#: (q1 은 무관하다. 지지구조물이 원점 중심 회전대칭이라 자기충돌은 q2, q3 만으로
#:  결정된다. J1 값은 컨트롤러 한계를 그대로 쓴다.)
#:
#: 박스 모서리에서의 최소 틈새는 10.4 mm 라 여유 기준에 거의 붙어 있다.
#: 그래서 이 박스는 계획을 거르는 1차 관문일 뿐이고, 실제 관문은 여전히
#: 모든 계획 경로에서 도는 `collision.check_self_collision()` 이다.
SELF_COLLISION_SAFE_BOX_DEG = (150.0, 110.0, 155.0)

#: 소프트웨어가 쓰는 한계 = 두 제약 중 좁은 쪽.
#:
#:   - 컨트롤러 한계에서 1도 뺀 값. 컨트롤러 폴트(리셋 필요)로 가기 전에
#:     소프트웨어가 먼저 SafetyViolation 으로 거부하게 하는 완충이다.
#:   - 자기충돌 안전 박스.
#:
#: 축마다 어느 쪽이 이기는지가 다르다. J2 는 충돌 박스(110)가, J1·J3 는
#: 컨트롤러 완충(149, 154)이 이긴다.
_LIMIT_MARGIN_DEG = 1.0
JOINT_LIMITS_RAD = tuple(
    (max(lo + _LIMIT_MARGIN_DEG * DEG, -box * DEG),
     min(hi - _LIMIT_MARGIN_DEG * DEG, box * DEG))
    for (lo, hi), box in zip(CONTROLLER_JOINT_LIMITS_RAD,
                             SELF_COLLISION_SAFE_BOX_DEG)
)


# =============================================================================
# 원점 · 동특성
# =============================================================================

#: 기구 원점 정의. 엔코더가 초기화되거나 기구를 분해했다 재조립하면
#: 이 문장이 영점을 되살릴 유일한 근거다.
#: Isaac Sim USD 의 베이스 +x 축을 반드시 이것과 같게 맞출 것.
#:
#: J2/J3 는 "팔이 일직선" 이라는 물리 조건이 자세를 결정하므로 선택의 여지가 없다.
#: J1 은 그 일직선이 공간에서 어디를 향하느냐라서 사람이 정한다.
ZERO_POSE_DEFINITION = (
    "q=[0,0,0] 은 세 링크가 완전히 펴진 일직선 자세이며, "
    "그 일직선(= 베이스 +x 축)이 베이스 판 앞모서리에 수직."
)

#: 팔의 1차 비틀림 공진 주파수 [Hz]. 실측값.
#:
#: 측정: 이동 중 TCP 가속도를 100 Hz 로 기록해, 속도가 다른 26개 구간의
#: 스펙트럼을 봤다. 속도를 3.2배(90 -> 286 mm/s) 바꿔도 봉우리가
#: 6.25 +- 0.90 Hz 로 고정됐다. 주파수가 속도와 무관하면 궤적 문제가 아니라
#: 구조 공진이다.
#:
#: 검산: J1 축 둘레 관성 J = 0.756 kg·m^2 (시험 자세) -> K = J(2 pi f)^2
#: = 1165 N·m/rad = 0.34 N·m/arcmin. PQ003 급 소형 유성감속기의 통상 비틀림
#: 강성 범위(0.2~1)에 들어간다.
#:
#: 쓰임: 사다리꼴 속도 프로파일의 가속 램프가 이 공진을 여기한다. 잔류 진동은
#: |sin(pi * f * T_a)| 에 비례하므로 가속 시간 T_a 를 공진 주기의 정수배로
#: 잡으면 이론상 0 이 된다 (input shaping 과 같은 원리).
#: motion.ACC_RATIO_ZERO_VIB 가 이 값을 쓴다.
RESONANCE_HZ = 6.25

#: 관절 최대 가속도 [rad/s^2]. 미정.
JOINT_MAX_ACCEL_RAD_S2 = PENDING

#: 툴 (TCP 오프셋 + 페이로드). 엔드이펙터 미정.
TCP_M_RAD = PENDING
PAYLOAD = PENDING


# =============================================================================
# 접속
# =============================================================================

#: StellaX Eth1 고정 IP (PC 직결용)
STELLARX_HOST = "10.0.0.1"

#: JSON-RPC 포트
STELLARX_RPC_PORT = 9900

#: 컨트롤 핸들러에 붙는 세션 키. 반드시 "stellar_robot" 이어야 한다.
#:
#: 다른 이름을 주면 컨트롤러가 빈 핸들러를 새로 만든다. 설정 조회는 전역이라
#: 되는데 get_actual_joint_position() 이 [] 로 나오고 is_ready() 가 False 가
#: 된다. manufacturer / product / model 은 무엇을 넣어도 동작한다.
ROBOT_NAME = "stellar_robot"
ROBOT_CATEGORY = "manipulator"
ROBOT_MANUFACTURER = "SODERO"
ROBOT_PRODUCT = "custom_robot"
ROBOT_MODEL = "custom_robot"


def robot_params() -> dict:
    """컨트롤 핸들러 생성 파라미터. SDK 기본값에서 속도만 낮췄다."""
    return {
        "debug_level": "INFO",
        "safety_controller_check_enable": False,
        "motion_parameter": {
            "default_tcp_speed": 0.05,        # m/s. SDK 기본 0.2 는 이 기계엔 빠르다
            "default_acceleration_time": 0.5,
        },
        "ip_address": "localhost",
        "command_server_port": 9009,
        "status_server_port": 9010,
    }


#: EtherCAT 데이지체인 순서 -> 축 매핑. StellaX EtherCAT Out -> D1 -> D2 -> D3
ETHERCAT_CHAIN = ("J1", "J2", "J3")


# =============================================================================

def joint(name_or_index) -> JointDrive:
    """이름(J1) 또는 인덱스(0)로 관절 정보를 얻는다."""
    if isinstance(name_or_index, int):
        return JOINTS[name_or_index]
    for j in JOINTS:
        if j.name == name_or_index:
            return j
    raise KeyError(f"unknown joint: {name_or_index!r}")


def assert_resolved(value, what: str):
    """PENDING 값을 실수로 쓰는 것을 막는다."""
    if value is PENDING or value == PENDING:
        raise RuntimeError(
            f"{what} 가 아직 확정되지 않았다 (config.PENDING). "
            f"docs/hardware.md 의 미확정 항목을 먼저 처리할 것.")
    return value
