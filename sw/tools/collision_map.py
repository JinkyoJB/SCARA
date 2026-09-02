"""자기충돌 지도 — Designer 관절 한계를 얼마까지 올려도 되는가.

모터를 움직이지 않는다. 순수 계산이다.

무엇을 그리는가
---------------
자기충돌은 q1 과 무관하다 (지지구조물이 원점 중심 회전대칭이라서).
그래서 (q2, q3) 평면 하나로 전부 표현된다.

  1. (q2, q3) 격자에서 `collision.clearance()` 를 계산해 안전/충돌 지도를 그린다
  2. 충돌까지의 여유별 경계를 표로 정리한다
  3. TCP 작업공간이 얼마나 넓어지는지 (예전 박스 vs 충돌검사) 비교한다
  4. Designer 에 넣을 관절 한계 추천값을 낸다

사용:
    python sw/tools/collision_map.py
    python sw/tools/collision_map.py --step 2     # 촘촘하게
"""

import argparse
import io
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import console  # noqa: E402
console.setup()

from scara import collision, config                    # noqa: E402
from scara import kinematics as kin                    # noqa: E402

D = math.pi / 180.0
R = 180.0 / math.pi
LOG_DIR = Path(__file__).resolve().parents[1] / "logs"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=3.0, help="격자 간격 [deg]")
    ap.add_argument("--margin", type=float, default=collision.MARGIN_M * 1000,
                    help="안전 여유 [mm]")
    args = ap.parse_args()
    step = args.step
    margin = args.margin / 1000.0

    print("=" * 86)
    print("자기충돌 지도  (모터 안 움직임 — 순수 계산)")
    print("=" * 86)
    print(f"  격자 {step:.0f}도,  안전 여유 {margin*1000:.0f} mm,  검사 쌍 {len(collision.PAIRS)}개")
    print("  자기충돌은 q1 과 무관하다 (지지구조물이 원점 중심 대칭).")

    # ---- 1) (q2, q3) 지도 ---------------------------------------------------
    lim = 180.0
    q2s = [(-lim + k * step) for k in range(int(2 * lim / step) + 1)]
    q3s = list(q2s)
    grid = {}
    for q2 in q2s:
        for q3 in q3s:
            d, pair = collision.clearance([0.0, q2 * D, q3 * D])
            grid[(q2, q3)] = (d, pair)

    print()
    print("지도  (행 q2 위->아래 +180..-180, 열 q3 좌->우 -180..+180)")
    print("  #=충돌  .=여유부족(<%dmm)  o=안전  " % (margin * 1000)
          + "[박스] = 현재 컨트롤러 한계 (J2 +-95, J3 +-140)")
    show_step = max(step, 7.5)
    rows = [q for q in sorted(set(round(v / show_step) * show_step for v in q2s),
                              reverse=True) if abs(q) <= 180]
    cols = [q for q in sorted(set(round(v / show_step) * show_step for v in q3s))
            if abs(q) <= 180]

    def cell(q2, q3):
        key = min(grid, key=lambda k: (k[0] - q2) ** 2 + (k[1] - q3) ** 2)
        d, _ = grid[key]
        inside = abs(q2) <= 95 and abs(q3) <= 140
        if d < 0:
            ch = "#"
        elif d < margin:
            ch = "-"
        else:
            ch = "o" if not inside else "O"
        return ch

    hdr = "        " + "".join(f"{int(c):>4d}" for c in cols[::3])
    print(hdr + "   (q3, 3칸마다 표시)")
    for q2 in rows:
        line = "".join(cell(q2, q3) for q3 in cols)
        print(f"  {int(q2):+5d} {line}")
    print("  (O=안전+현재 컨트롤러 박스 안, o=안전하지만 박스 밖, -=여유부족, #=충돌)")

    # ---- 2) 축별 한계 추천 --------------------------------------------------
    # q3=0 에서 q2 를 밀어보고, q2=0 에서 q3 를 밀어본다 (+ 최악 조합 확인)
    print()
    print("-" * 86)
    print("축별 안전 한계 (다른 축 = 0 일 때, 여유 %dmm 기준)" % (margin * 1000))
    print("-" * 86)

    def push(axis, sign):
        q = [0.0, 0.0, 0.0]
        deg = 0.0
        while deg <= 179.5:
            q[axis] = sign * deg * D
            d, _ = collision.clearance(q)
            if d < margin:
                return sign * (deg - 0.5)
            deg += 0.5
        return sign * 179.5

    j2n, j2p = push(1, -1), push(1, +1)
    j3n, j3p = push(2, -1), push(2, +1)
    print(f"  J2 (q3=0):  {j2n:+.1f} ~ {j2p:+.1f} 도")
    print(f"  J3 (q2=0):  {j3n:+.1f} ~ {j3p:+.1f} 도")

    # 박스 후보: 지도에서 '박스 전체가 안전' 인 최대 대칭 박스
    def box_safe(b2, b3):
        for q2 in q2s:
            if abs(q2) > b2:
                continue
            for q3 in q3s:
                if abs(q3) > b3:
                    continue
                if grid[(q2, q3)][0] < margin:
                    return False
        return True

    best = None
    for b2 in range(90, 181, 5):
        for b3 in range(90, 181, 5):
            if box_safe(b2, b3):
                area = b2 * b3
                if best is None or area > best[0]:
                    best = (area, b2, b3)
    print()
    if best:
        _, b2, b3 = best
        print(f"  ** 박스 전체가 안전한 최대 대칭 박스:  J2 +-{b2}도,  J3 +-{b3}도 **")
        print("     (Designer 한계를 이 값으로 올리면, 박스 안에서는 어떤 조합도 충돌하지 않는다.")
        print("      소프트웨어 충돌 검사는 그 위에 이중 안전으로 계속 동작한다.)")
    else:
        print("  90도 이상에서 전체 안전한 대칭 박스 없음 — 소프트웨어 검사 의존 필요")

    # ---- 3) 작업공간 비교 ---------------------------------------------------
    print()
    print("-" * 86)
    print("TCP 작업공간 비교 (phi 자유,  x 0~660 / y -660~660,  20mm 격자)")
    print("-" * 86)

    def reachable(x, y, box2, box3, use_col):
        for pd in range(-180, 180, 10):
            for sol in kin.ik_all(x / 1000, y / 1000, pd * D):
                q2d, q3d = sol.q[1] * R, sol.q[2] * R
                if abs(q2d) > box2 or abs(q3d) > box3:
                    continue
                # 특이점 하한은 동일하게 적용
                s = abs(math.sin(sol.q[1]))
                if s < math.sin(15 * D):
                    continue
                if use_col:
                    d, _ = collision.clearance(sol.q)
                    if d < margin:
                        continue
                return True
        return False

    cases = [
        ("예전 (박스 J2 90 / J3 135)", 90, 135, False),
        ("현재 (박스 94/139 + 충돌검사)", 94, 139, True),
    ]
    if best:
        cases.append((f"확장 (박스 {best[1]}/{best[2]} + 충돌검사)", best[1], best[2], True))

    xs = range(0, 661, 20)
    ys = range(-660, 661, 20)
    counts = []
    for name, b2, b3, uc in cases:
        n = sum(1 for x in xs for y in ys if reachable(x, y, b2, b3, uc))
        counts.append((name, n))
        print(f"  {name:36s} 도달 격자점 {n:5d}개")
    if len(counts) >= 2 and counts[0][1]:
        for name, n in counts[1:]:
            print(f"    -> {name.split()[0]} 은 예전 대비 {n/counts[0][1]:.2f}배")

    # ---- 저장 ---------------------------------------------------------------
    LOG_DIR.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"collision_map_{stamp}.json"
    io.open(path, "w", encoding="utf-8").write(json.dumps({
        "timestamp": stamp, "step_deg": step, "margin_mm": margin * 1000,
        "j2_range_at_q3_0": [j2n, j2p], "j3_range_at_q2_0": [j3n, j3p],
        "max_safe_box": {"j2": best[1], "j3": best[2]} if best else None,
        "workspace_counts": counts,
        "grid": [{"q2": k[0], "q3": k[1], "clear_mm": round(v[0] * 1000, 1),
                  "pair": list(v[1])} for k, v in grid.items()],
    }, ensure_ascii=False))
    print()
    print(f"  로그: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
