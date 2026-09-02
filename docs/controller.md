# 제어기 — Sodero StellaX

**StellaX 가 EtherCAT 마스터다.** 별도 마스터 스택을 구현할 필요가 없고,
궤적 보간과 기구학도 StellaX 가 한다. 이 저장소는 그 위에서 목표점과 속도만
정해 넘긴다.

## 1. 결선

```
StellaX [EtherCAT Out] --> D1 X2A(IN)
                           D1 X2B(OUT) --> D2 X2A(IN)
                                           D2 X2B(OUT) --> D3 X2A(IN)
```

체인 순서가 곧 축 번호다: J1=0, J2=1, J3=2 (`Identification` 의 `Idx`).

| 포트 | 용도 |
|---|---|
| `Eth1` | 고정 IP 10.0.0.1/24 (PC 직결), RPC 포트 9900 |
| `Eth2` | DHCP |
| `EtherCAT Out` | 마스터 출력 -> 첫 슬레이브 IN |
| `DC 24V In` | 전원 |

**드라이버 전원은 두 계통이다.** 제어 전원(L1C, L2C)은 화면·엔코더·EtherCAT
통신을 살리고, 주회로 전원(L1, L3)이 모터 전류를 만든다. 둘을 헷갈리면
2절의 증상이 나온다.

## 2. 기동 순서 — 반드시 이 순서로

```
1. 드라이버 제어 전원   L1C, L2C      -> 화면 / 엔코더 / EtherCAT 통신
2. 드라이버 주회로 전원 L1, L3        -> 모터 전류.  빠뜨리기 쉽다
3. StellaX 전원
4. 브라우저에서 Stella Designer 연결
5. Step 4: Verification 까지 진행     -> EtherCAT OP 진입, 제어 루프 기동
6. 그 다음에 파이썬 실행
```

**2번을 빠뜨리면** 통신·엔코더·FK/IK 가 전부 정상으로 보이는데 모터만 안 돈다.
`servo(True)` 순간 세 축이 `Err88.0` 으로 폴트난다. 3절 참조.

**5번을 빠뜨리거나 폴트로 제어 루프가 멈추면** RPC 로는 못 되살린다. 4절 참조.

**Designer 를 켜면 동작 모드가 AUTO 로 돌아온다.** 외부 RPC 지령은 MANUAL
이어야 한다. `SCARA.prepare()` 가 매 이동 전에 자동으로 전환한다.

## 3. 에러 해독

`servo_drive_error` 는 `0xFF**` 형식이고 `**` 가 Panasonic 알람 MAIN 번호(16진)다.

    0xFF58 -> 0x58 = 88 -> Err88.x
    0xFF50 -> 0x50 = 80 -> Err80.x
    0xFF10 -> 0x10 = 16 -> Err16.0

자주 만나는 것:

| 코드 | 의미 | 원인·대응 |
|---|---|---|
| **Err88.0** | Main power undervoltage | **주회로 전원(L1/L3) 미투입.** 전원을 올리고 Designer 재연결 |
| Err88.1 | Control mode setting error | 6060h Modes of operation 설정 오류 |
| Err88.2 | ESM requirements during operation | EtherCAT 상태 전이 위반 |
| **Err80.x** | EtherCAT ESM 관련 | 드라이버가 OP 밖으로 이탈. Designer Step 4 재실행 |
| **Err16.0** | 모터 과부하 | 막힌 축을 계속 밀었거나, 궤적이 살아있는 채로 서보를 껐다 |

### Err88.0 을 알아보는 법

- RPC 는 성공을 돌려준다 (`move_joint` -> True)
- `wait_until_done()` 이 끝나지 않는다
- 관절각이 0.001 deg 도 변하지 않는다
- `target_joint_position == actual_joint_position` — 궤적 자체가 생성되지 않는다
- **`get_actual_joint_effort()` 가 세 축 모두 정확히 0.0**
- 중단 후 세 축 `pds=7 (Fault)`, `servo_drive_error=0xFF58`

사양서상 발생 조건이 "PDS 가 Ready to switch on 이고 주회로 전원이 OFF 인
상태에서 Switch on 명령을 받았을 때" 로, 위 증상과 정확히 일치한다.
드라이버의 CHARGE 램프와 토크 0.0 을 같이 보면 빠르게 판별된다.

## 4. EtherCAT 재초기화

드라이버가 폴트로 떨어지거나 제어 루프가 멈추면, **브라우저에서 Stella
Designer 를 다시 연결해 `Step 4: Verification` 까지 진행해야** 살아난다.

RPC 로는 복구되지 않는다:

- `reset_trigger()` 는 True 를 돌려주지만 `pds` 가 7 에서 안 내려온다
- 드라이버만 전원 재투입해도 StellaX 마스터가 그대로라 ESM 이 어긋난 채 남는다

Designer 연결이 마스터를 다시 올려 `INIT -> PreOP -> SafeOP -> OP` 를 다시
밟게 한다.

### 제어 루프가 살아 있는지 판별

`trajectory_status.time_stamp` 가 증가하는지 본다.

```python
a = bot.status()["trajectory_status"]["time_stamp"]
time.sleep(1.0)
b = bot.status()["trajectory_status"]["time_stamp"]
# a == b 이면 제어 루프가 멈춘 것
```

`status()` 는 캐시하지 않고 매번 RPC 를 던지므로, 값이 그대로면 컨트롤러 쪽이
멈춘 것이 확실하다. `safety.check_control_loop_alive()` 가 이 검사를 하고
`startup_checklist()` 가 자동으로 부른다.

**멈춘 상태에서 읽히는 값은 전부 죽기 직전의 스냅샷이다.** `pds`, `err`,
`servo_on`, `ready` 를 그대로 믿으면 안 된다. 그 사이에 사람이 전원을 올렸든
내렸든 반영되지 않는다.

## 5. Designer 설정값

**Robot System Parameters**

| 항목 | 값 |
|---|---|
| Max Arm Reach | 650 mm |
| Gravity Vector | 연직 (−Z) |

**Actuator Mapping**

| 항목 | J1 | J2 | J3 |
|---|---|---|---|
| Rated / Peak Torque | 0.32 / 0.95 N·m | 동일 | 동일 |
| Rated Output Power | 100 W | 100 | 100 |
| No-Load / Rated Speed | 6000 / 3000 r/min | 동일 | 동일 |
| Reduction ratio | 0.01 | 0.01 | 0.019802 |
| RxPDO / TxPDO index | 0x1600 / 0x1A00 | 동일 | 동일 |
| Rotor Inertia | 5,100 g·mm² | 4,800 | 4,800 |

**Reduction ratio 의 부호가 회전 방향이다.** 반전은 여기 또는 드라이버
`607Eh Polarity` 중 **한 곳에서만** 해야 한다. 양쪽에 걸면 상쇄된다.
자세한 것은 [calibration.md](calibration.md) 참조.

PDO 엔트리는 Designer 가 고정으로 정한다 (수정 불가). Rx 5개 / Tx 6개:

| Rx | 6040h Control Word · 607Ah Target Position · 6071h Target Torque · 60B2h Torque Offset · 6060h Mode of Operation |
|---|---|
| **Tx** | 6041h Status Word · 6064h Position Actual · 606Ch Velocity Actual · 6077h Torque Actual · 6061h Mode Display · 603Fh Error Code |

Panasonic 출하 기본 0x1600 과 내용이 달라서 Designer 가 remap 을 수행한다.

## 6. RPC 규약

외부 제어는 `stellarx_remote` JSON-RPC SDK 를 쓴다. `sw/scara/client.py` 가
아래 규약을 흡수하므로 응용 코드는 신경 쓰지 않아도 된다.

- **모든 커맨드가 키워드 인자만 받는다.**
- **반환값 첫 원소가 상태 플래그, 실제 값은 index 1** 이다.
- FK/IK 두 커맨드는 응답이 **이중으로 감싸여** 온다. 그중 IK 는 안쪽 첫 원소가
  정수 결과코드이며 **0 이 성공**이라, 다른 커맨드와 규약이 반대다.
- 입력 단위는 항상 **SI(rad, m)** 다. `get_processing_unit` 이 DEGREE_MM 여도
  그렇다.
- 컨트롤러는 관절 배열을 7칸 고정으로 돌려주고 뒤를 0 으로 채운다. 앞 3개만 쓴다.
- 커맨드는 **동적 바인딩**이라 노출되는 이름이 문서와 다를 수 있다.
  `client.commands()` 로 실제 목록을 확인할 수 있다.
- SDK 는 호출마다 TCP 연결을 새로 연다. 스레드 안전하다.

### `ROBOT_NAME` 은 반드시 `"stellar_robot"`

이 이름이 실행 중인 로봇 세션에 붙는 키다. 다른 이름을 주면 컨트롤러가 빈
핸들러를 새로 만들어 준다. 설정 조회는 전역이라 되는데
`get_actual_joint_position()` 이 `[]` 로 나오고 `is_ready()` 가 False 가 된다.
manufacturer / product / model 은 무엇을 넣어도 동작한다.
