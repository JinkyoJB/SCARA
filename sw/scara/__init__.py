"""SCARA 제어 패키지.

    main.py / tools/     실행 진입점
       |
    robot.py             SCARA 클래스 — 보통 이것만 쓰면 된다
       |
    motion.py            검사 파이프라인을 지난 뒤에만 명령을 내보낸다
       |
    safety.py            속도·토크·가동범위·제어루프·회전방향
    collision.py         CAD 기반 자기충돌
    kinematics.py        해석적 FK/IK, 야코비안, 특이점
       |
    client.py            StellaX JSON-RPC 래퍼
       |
    config.py            하드웨어 상수 (단일 출처)

실시간 제어(EtherCAT 마스터, 궤적 보간, 기구학)는 StellaX 제어기가 담당한다.
이 패키지는 그 위의 응용 로직과 안전 계층이다.
"""

from . import config, safety  # noqa: F401

__all__ = ["config", "safety", "client", "motion", "robot"]
