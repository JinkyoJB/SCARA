# 소프트웨어

## 역할 분담

| 담당 | 무엇 |
|---|---|
| **StellaX** | EtherCAT 마스터, csp 사이클릭 통신, 궤적 보간, 기구학, 축 상태머신 |
| **`sw/`** | 접속·상태 관리, 안전 한계 검사, 응용 시퀀스, UI |
| `HW/model/` | 오프라인 대조 검증 (동역학·수명). 런타임에 쓰지 않는다 |

실시간 제어를 직접 구현하지 않는다. 목표점과 속도만 정해 StellaX 에 넘긴다.

## 구조

```
sw/
  main.py          진입점 — 점검 -> 영점 확정 -> 클릭 UI -> 종료 시 홈 복귀
  scara/
    robot.py       SCARA 클래스. 보통 이것만 쓰면 된다
    motion.py      검사 파이프라인을 지난 뒤에만 명령을 내보낸다
    safety.py      속도·토크·가동범위·제어루프·회전방향
    collision.py   CAD 기반 자기충돌 (몸체 13개 / 검사쌍 28)
    kinematics.py  해석적 FK/IK, 야코비안, 특이점
    client.py      StellaX JSON-RPC 래퍼
    config.py      하드웨어 상수 단일 출처
    ui.py          클릭 위치제어 UI (matplotlib)
  tools/           진단 도구
  tests/           실기 없이 도는 검증
```

의존 방향은 위에서 아래로만 흐른다. `client.Scara` 를 직접 쓰지 말고
`robot.SCARA` 또는 `motion.*` 를 통한다.

## 안전 파이프라인

모든 이동이 같은 검사를 지난다.

```
목표 (x, y) 또는 관절각
  ├ auto_phi        phi 자유도를 한계·특이점·충돌을 피해 고른다 (창 60도)
  ├ 도달 가능성      safety.check_reach
  ├ IK              kinematics.ik(seed_q=현재자세)   자세 연속성 유지
  ├ 특이점          safety.check_singularity          작업공간 목표만
  ├ 가동범위        safety.check_joint_limits
  ├ 자기충돌        collision.check_self_collision    목표 + 경로 중간 5도마다
  ├ TCP 속도 상한   motion.tcp_speed_cap              작업공간 한계의 85%
  └ 공진 회피 가속  motion.ACC_RATIO_ZERO_VIB
      -> StellaX move_joint / move_linear
```

**관절공간 이동은 특이점을 기본으로 막지 않는다.** 관절보간은 야코비안을 쓰지
않아 속도 증폭이 없고, 무엇보다 **특이점에서 빠져나오는 유일한 수단**이기
때문이다. 여기서 막으면 한번 들어간 자세에서 못 나온다. 반대로 작업공간
목표(x, y, phi)는 특이점을 막는다.

## 도구

일상 제어는 `python sw/main.py`. 아래는 특정 상황에서만 쓴다.

| 도구 | 언제 |
|---|---|
| `verify_model.py` | **Designer 설정을 바꾼 뒤.** 축을 하나씩 +60도 움직여 방향·크기·영점을 눈으로 검증 |
| `check_zero.py` | **영점을 새로 잡은 뒤.** 왕복해도 같은 자세로 오는지(반복 정밀도) |
| `collision_map.py` | 충돌 모델을 손봤을 때. (q2,q3) 안전 지도와 Designer 한계 추천값. **모터를 움직이지 않는다** |
| `dump_config.py` | 컨트롤러에 실제로 뭐가 설정돼 있는지 볼 때 |
| `move_cli.py` | 좌표를 타이핑해서 이동. UI 없이 디버깅할 때 |

```bash
python sw/tools/collision_map.py      # 모터 안 움직임
python sw/tools/verify_model.py --plan
```

## 클릭 UI

`sw/scara/ui.py`. `main.py` 가 부르고, 단독 실행도 된다.

- 베이스가 화면 중앙. 화면 위 = 로봇 +x, 화면 왼쪽 = 로봇 +y 라서 반시계
  회전이 화면에서도 반시계로 보인다
- 초록 = 갈 수 있는 영역(한계·특이점·충돌을 전부 통과하는 phi 가 존재),
  주황 = 못 가는 영역
- 초록 영역 **더블클릭** -> 이동. 좌상단 **Home 버튼** -> 일직선 자세
- 우측 세로 슬라이더로 가속 배수 조절. 초록선 3.125 가 공진 여기율 0
- 실수 클릭 방지: 지도는 더블클릭만 인정, Home 은 한 번 누르면 무장만 되고
  3초 안에 다시 눌러야 실제로 간다

도달 격자는 계산에 몇 초 걸려서 `sw/logs/reach_cache.json` 에 캐시한다.
한계·여유·격자 간격이 바뀌면 자동으로 다시 계산한다.

## 설계 원칙

**미확정 값을 추정치로 채우지 않는다.** `config.PENDING` 항목을 쓰려고 하면
예외가 난다. 모르는 값으로 움직이면 기구를 부순다.

**속도는 항상 클램프한다.** 이 기계는 모터 최고회전수 ÷ 기어비가 감속기 허용
출력회전수와 정확히 같아 여유가 0 이다.

**토크 병목이 축마다 다르다.** J1·J2 는 감속기, J3 는 모터가 먼저 한계다.
`JointDrive.continuous_torque_limit_nm` 이 둘 중 작은 쪽을 고른다.

**가드의 문턱은 실측값 기준으로 잡는다.** 조그 토크 가드를 감속기 연속한계
(29 N·m)의 80% 로 잡으면 실제 주행토크가 9.5 N·m 인 축에서는 영영 안 걸린다.
절대 실패하지 않는 검사는 검사가 아니다.

**안전은 소프트웨어가 최종 방어선이 아니다.** A6BE 는 STO 를 지원하지 않는다.
외부 E-stop 회로가 반드시 있어야 한다.

## 종료 절차

    stop()  ->  check_finish() 로 정지 확인 (최대 2초)  ->  servo(False)  ->  close()

`stop()` 은 감속을 시작만 한다. 확인 없이 서보를 끄면 궤적이 살아있는 채로
드라이버가 폴트를 낸다. `SCARA.close()` 가 이 순서를 지킨다.

## 테스트

```bash
python sw/tests/test_kinematics.py
python sw/tests/test_client_protocol.py
```

실기 없이 돈다. 가짜 컨트롤러를 물려서 **안전 검사가 전송 전에 막는지**를
확인하는 것이 핵심이다.
