"""컨트롤러에 실제로 무엇이 설정돼 있는지 전부 뽑아본다.

Stellar Designer 웹 UI에서 넣은 값이 실제로 어떻게 저장됐는지 확인하는 용도.
모터를 움직이지 않으므로 언제든 안전하게 실행 가능하다.

사용:
    python sw/tools/dump_config.py
    python sw/tools/dump_config.py --json out.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import console  # noqa: E402
console.setup()

from scara import config                  # noqa: E402
from scara.client import Scara            # noqa: E402


#: 인자 없이 부를 수 있는 조회 커맨드
SIMPLE_QUERIES = [
    "get_dh_parameters",
    "get_processing_unit",
    "get_operating_status",
    "get_error_status",
    "get_alarm",
    "get_operation_policy",
    "get_operational_mode",
    "get_total_joint_space_kinematic_limits",
    "get_task_space_kinematic_limits",
    "get_default_tcp",
    "get_default_payload",
    "get_default_motion",
    "get_feed_rate",
    "get_joint_temperature",
    "get_trajectory_status",
    "get_actual_joint_position",
    "get_actual_pose",
]

#: joint_index 가 필요한 조회
PER_JOINT_QUERIES = ["get_default_joint_limit", "get_user_joint_limit"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=str, default=None, help="결과를 JSON 파일로 저장")
    args = ap.parse_args()

    bot = Scara()
    out: dict = {"commands": list(bot.commands)}

    print("=" * 72)
    print(f"컨트롤러 설정 덤프  ({config.STELLARX_HOST}:{config.STELLARX_RPC_PORT})")
    print("=" * 72)
    print(f"바인딩된 커맨드: {len(bot.commands)}개")
    print()

    for name in SIMPLE_QUERIES:
        if not bot.has(name):
            print(f"  {name:42s} (없음)")
            continue
        try:
            r = getattr(bot.raw, name)()
            out[name] = r
            print(f"  {name:42s} = {r}")
        except Exception as exc:
            out[name] = f"ERROR: {exc}"
            print(f"  {name:42s} ! {exc}")

    print()
    print("-- 축별 --")
    limits = []
    for i in range(8):  # 실제 축 수를 모르므로 넉넉히 돌다가 범위 밖이면 멈춘다
        row = {}
        stop = False
        for name in PER_JOINT_QUERIES:
            if not bot.has(name):
                continue
            try:
                r = getattr(bot.raw, name)(joint_index=i)
                if isinstance(r, list) and r and r[0] is False:
                    if "out of range" in str(r[1]):
                        stop = True
                        break
                    row[name] = f"ERROR: {r[1]}"
                else:
                    row[name] = r[1] if isinstance(r, list) and len(r) == 2 else r
            except Exception as exc:
                row[name] = f"ERROR: {exc}"
        if stop:
            break
        limits.append(row)
        lim = row.get("get_user_joint_limit")
        if isinstance(lim, list) and len(lim) == 2:
            print(f"  J{i+1} 가동범위: [{math.degrees(lim[0]):+8.2f}, {math.degrees(lim[1]):+8.2f}] deg "
                  f"(원시값 rad: {lim})")
        else:
            print(f"  J{i+1}: {row}")
    out["joint_limits"] = limits
    print(f"  → 컨트롤러가 인식하는 축 수: {len(limits)}")

    # ---- CAD 검증값과 대조 ----
    print()
    print("=" * 72)
    print("CAD 검증값과 대조")
    print("=" * 72)
    dh = out.get("get_dh_parameters")
    if isinstance(dh, list) and len(dh) == 2 and isinstance(dh[1], dict):
        table = dh[1]
        ok = True
        for i, (key, a_expected) in enumerate(zip(sorted(table), config.LINK_LENGTHS_M)):
            link = table[key]
            a = link.get("a")
            alpha = link.get("alpha")
            d = link.get("d")
            match = abs(a - a_expected) < 1e-6 and abs(alpha) < 1e-9
            ok &= match
            print(f"  {key}: a={a} (CAD {a_expected})  alpha={alpha}  d={d}  "
                  f"{'OK' if match else '<-- 불일치'}")
        print()
        print("  [OK] DH가 CAD 검증값과 일치한다" if ok else "  [주의] DH 불일치 — 확인 필요")
    else:
        print("  DH 파라미터를 읽지 못했다")

    # get_actual_joint_position 은 [status, value] 형태.
    # 컨트롤러는 관절 배열을 7칸 고정으로 돌려주고 뒤를 0으로 채운다.
    #   (target_joint_position 의 뒤쪽 칸에는 초기화되지 않은 쓰레기값도 들어온다)
    #   따라서 "몇 칸이 왔나"가 아니라 "앞쪽 몇 칸이 실제 축인가"로 판정해야 한다.
    #   실제 축 수는 DH/가동범위 정의 수(len(limits))가 기준이다.
    raw_q = out.get("get_actual_joint_position")
    q_full = raw_q[1] if isinstance(raw_q, list) and len(raw_q) == 2 else []
    q = q_full[: config.DOF] if isinstance(q_full, list) else []

    print()
    print(f"  기구 정의 축 수 (DH/가동범위): {len(limits)}")
    print(f"  관절 배열 길이 (패딩 포함)   : {len(q_full)}  → 앞 {config.DOF}칸 사용")
    if len(q_full) == 0:
        print("  [주의] 관절 데이터가 비어 있다 = 구동계가 온라인이 아니다.")
        print("         확인 순서:")
        print("           1) config.ROBOT_NAME 이 'stellar_robot' 인지 (다르면 빈 핸들러가 잡힌다)")
        print("           2) 드라이버 3대 전원, EtherCAT 결선")
        print("           3) 웹 UI(Motion Studio)가 구동계를 점유 중인지")
    elif len(limits) == config.DOF and len(q_full) >= config.DOF:
        print(f"  [OK] 축 {config.DOF}개 온라인 — 구동계가 붙어 있다")
        print(f"       현재 관절각: {[f'{math.degrees(v):.2f}°' for v in q]}")
    else:
        print(f"  [주의] 기구 정의 {len(limits)}축 vs 데이터 {len(q_full)}칸. 매핑을 확인할 것.")

    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n저장됨: {args.json}")

    bot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
