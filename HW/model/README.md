# HW 모델 라이브러리 (오프라인 해석용)

팀원이 작성한 하드웨어 검증 코드를 `C:\Users\rlark\Desktop\Claude\ScaraHW` 에서 복사한 것.
**실시간 제어 코드가 아니다.** 실제 제어는 StellaX 제어기에서 수행하며,
이 폴더는 궤적·토크·수명 계산의 오프라인 레퍼런스(정답지) 역할만 한다.

## 폴더 구조 / 경로 규칙

```
SCARA/HW/
├─ scara.yaml          <- 로봇 파라미터 (DH, 질량, 관성, 기어비 ...)
├─ motors/             <- MSMF012_J1/J2/J3.yaml
├─ reducers/           <- PQ003-H.yaml, PQ003-M.yaml
└─ model/              <- (이 폴더) 파이썬 라이브러리
   ├─ define.py        상수 정의
   ├─ mathematics.py   SE(3)/SO(3), 오일러, 스크류 유틸
   ├─ robot.py         링크/툴 클래스, yaml 파라미터 로더
   ├─ dynamics.py      RNE 역동역학, 감속기·베어링 수명 계산
   ├─ traj.py          사다리꼴 / 5차 다항식 P2P 궤적 생성
   └─ example.ipynb    사용 예제
```

원본 대비 수정한 부분은 `robot.py` 의 경로 해석뿐이다.

- `HW_ROOT = <이 파일의 상위 폴더> = SCARA/HW` 상수를 추가
- `load_param_motor()` / `load_param_reducer()` 가 `HW_ROOT/motors`, `HW_ROOT/reducers` 를 참조
- `load_param_robot(..., file="scara.yaml")` 처럼 파일명만 넘기면 `HW_ROOT` 기준으로 해석

덕분에 작업 디렉터리와 무관하게 동작한다.

`example.ipynb` 의 기어비가 `[100,100,61]` 로 잘못 적혀 있어 `scara.yaml` 과 맞게
`[100,100,50.5]` 로 고쳤다.

## 설치 / 실행

```powershell
python -m pip install -r requirement_hw.txt
```

(`requirement_hw.txt` 의 pandas 3.0.2 / numpy 2.3.2 는 팀원 환경 기준 핀 버전.
버전 충돌이 나면 핀을 풀고 설치해도 무방하다. bokeh 는 노트북 시각화에만 쓰인다.)

```python
import sys, numpy as np
sys.path.insert(0, r'C:\Users\rlark\Desktop\SCARA\HW\model')

from robot import load_param_robot
from define import DEFAULT_GRAV_ACC_VEC
from dynamics import calculate_dynamics

base, links, kineType, name, axisConfig, basePose, grav, eulType, \
task_rated_speed, task_acc_time, task_dec_time = \
    load_param_robot('scara', np.zeros(6), DEFAULT_GRAV_ACC_VEC, file="scara.yaml")

data = calculate_dynamics(links, Q, QD, QDD)   # Q,QD,QDD: (N,3) rad
```

동작 확인 완료 (2026-08-11): 파라미터 로드 + `calculate_dynamics` 정상 실행.

`load_param_robot` 실행 시 `WARNING! joint ... is not defined` 가 한 번 출력되는데,
이는 모터/감속기가 없는 **BASE 링크(LINK0)** 생성 과정에서 나오는 것이며
J1~J3 는 `scara.yaml` 값을 정상적으로 사용한다. 무시해도 된다.
