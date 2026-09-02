# SCARA

3자유도 수평 다관절 로봇(RRR)의 제어 소프트웨어. 관절각뿐 아니라 **EE 좌표를
주면 그 위치로 간다.** 최종 목표는 이 실물과 정합되는 Isaac Sim 가상 모델을
만들고, 실험으로 정합성을 증명하는 것이다.

```
                +y (왼쪽)
                 ^
                 |     + 각도 = 위에서 볼 때 반시계
    (J1 회전축)   +--------> +x   영점에서 팔이 뻗는 방향
       원점                       링크 250 + 250 + 150 = 최대 도달 650 mm
```

세 관절축이 모두 연직·평행이라 팔은 수평면 안에서만 움직인다. z 는 -44 mm 로
고정이며 제어할 수 없고, 중력은 관절 토크를 만들지 않는다.

## 구성

| | |
|---|---|
| 기구 | 링크 250 / 250 / 150 mm, 최대 도달 650 mm |
| 모터 | Panasonic MINAS A6 MSMF012L1 (100 W) x 3, 23bit 앱솔루트 엔코더 |
| 드라이버 | MADLN05BE (A6BE), EtherCAT CoE / CiA402, csp |
| 감속기 | PQ003-H 100:1 (J1, J2), PQ003-M 50.5:1 (J3) |
| 제어기 | Sodero StellaX — **EtherCAT 마스터이자 궤적 보간·기구학 담당** |

실시간 제어는 StellaX 가 한다. 이 저장소는 그 위의 응용 로직과 안전 계층이다.

## 빠른 시작

전제: 전원과 EtherCAT 이 올라와 있어야 한다. 순서를 안 지키면 지령이 먹지
않는다 — [docs/controller.md](docs/controller.md) 참조.

```
1. 드라이버 제어 전원 L1C, L2C
2. 드라이버 주회로 전원 L1, L3      <- 빠뜨리면 통신은 되는데 모터만 안 돈다
3. StellaX 전원
4. 브라우저에서 Stella Designer 연결
5. Step 4: Verification 까지 진행   <- EtherCAT OP 진입
```

```bash
python sw/main.py
```

점검 -> 영점 확인 -> 클릭 위치제어 UI 순으로 진행하고, 종료할 때 일직선
자세로 되돌린다.

코드에서:

```python
from scara.robot import SCARA

bot = SCARA()              # 접속 + 기동 전 점검
bot.move_xy(400, 100)      # mm. phi 는 자동 (한계·특이점·충돌을 피해서 고른다)
bot.home()                 # q = [0, 0, 0]
bot.close()
```

## 저장소 구조

```
sw/
  main.py          진입점 — 점검 -> 영점 확정 -> 클릭 UI
  scara/           라이브러리 (robot / motion / safety / collision / kinematics / client / config / ui)
  tools/           필요할 때 꺼내 쓰는 진단 도구
  tests/           실기 없이 도는 검증
HW/                CAD, 메시, 모터·감속기 파라미터, 오프라인 해석 모델
docs/              하드웨어·제어기·정합·실측 결과 문서
vendor/            StellaX SDK (아래 설치 참조)
```

## 설치

파이썬 3.12 기준.

```bash
mkdir -p vendor && cd vendor
git clone https://github.com/Sodero-labs/stellarx_remote.git
git clone https://github.com/Sodero-labs/stxlib.git
python -m pip install ./stxlib
```

`stellarx_remote` 는 pip 패키지가 아니라 단일 모듈이다. `client.py` 가
`vendor/stellarx_remote` 를 `sys.path` 에 넣으므로 따로 설치하지 않는다.

UI 는 matplotlib 을 쓴다. StellaX `Eth1` 은 고정 IP `10.0.0.1/24`, RPC 포트
9900 이므로 PC 를 같은 대역(예 `10.0.0.2/24`)에 두고 직결한다.

## 안전

- **A6BE 는 STO 를 지원하지 않는다.** 소프트웨어는 최종 방어선이 아니다.
  외부 E-stop 회로가 반드시 있어야 한다.
- 모든 이동이 같은 파이프라인을 지난다:
  가동범위 -> 특이점 -> CAD 자기충돌(경로 중간 샘플 포함) -> TCP 속도 상한
  -> 공진 회피 가속.
- **영점은 소프트웨어가 스스로 검증할 수 없다.** 컨트롤러는 언제나 "지령대로
  갔다" 고 보고한다. 그래서 `main.py` 는 시작할 때마다 팔이 일직선인지 사람
  눈으로 한 번 묻는다.
- 미확정 값은 `config.PENDING` 으로 두고 추정치로 채우지 않는다. 쓰려고 하면
  예외가 난다.

## 문서

| | |
|---|---|
| [hardware.md](docs/hardware.md) | 확정된 기구·구동계 파라미터와 근거 |
| [controller.md](docs/controller.md) | StellaX 설정, 기동 순서, EtherCAT, 에러 해독 |
| [calibration.md](docs/calibration.md) | 좌표 규약, 영점, 회전 방향, Isaac Sim 정합 |
| [software.md](docs/software.md) | 패키지 구조, 안전 파이프라인, 도구 |
| [findings.md](docs/findings.md) | 실측으로 알아낸 것 (공진, 토크 리플, 작업공간) |
| [CHECKLIST.md](docs/CHECKLIST.md) | 코드 쓰기 전에 훑을 함정 목록 |

## 다음

1. 엔드이펙터 확정 -> `TCP_M_RAD`, `DH_D_M[2]`, `PAYLOAD`
2. 기구학 캘리브레이션 (a1, a2, a3 와 각 축 영점 오차 실측)
3. Isaac Sim USD 작성 — **베이스 +x 축을 `config.ZERO_POSE_DEFINITION` 과
   같게 맞출 것**
4. 같은 지령을 실물과 가상에 넣고 TCP 궤적을 비교해 정합성 정량화
