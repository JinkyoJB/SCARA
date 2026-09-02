"""클릭 위치제어 UI — 갈 수 있는 곳을 보여주고, 더블클릭하면 간다.

화면
----
  - 베이스(J1 축)가 화면 중앙. J1 이 +-149도라 360도 공간을 다 쓴다.
    화면 위 = 로봇 +x, 화면 왼쪽 = 로봇 +y  ->  반시계 회전이 화면에서도 반시계
  - 초록 = EE 가 갈 수 있는 영역 (가동범위·특이점·자기충돌을 전부 통과하는
    phi 가 하나라도 존재), 주황 = 못 가는 영역
  - 초록 영역 더블클릭 -> 그 위치로 이동
  - 좌상단 Home 버튼 -> 일직선 자세 q=[0,0,0] 으로
  - 우측 세로 슬라이더로 가속 배수(acc) 조절. 초록선 3.125 가 공진 여기율 0.
    회색선 6.25, 2.083 도 0 이고, 그 사이 값은 팔 공진 6.25 Hz 를 때린다.
  - 링크 3개를 직선으로 그린다 (100 ms 주기로 실기 상태 갱신)

안전
----
  - 이동은 라이브러리 파이프라인을 그대로 쓴다:
    auto_phi (한계·특이점·충돌 필터 + phi 창 60도)
    -> move_joint_safe (경로 중간 충돌 샘플링 포함)
    -> TCP 속도 자동 제한 (작업공간 한계의 85%)
  - 이동 중에는 요청을 받지 않는다
  - 실수 클릭 방지: 지도는 더블클릭만 인정하고, Home 버튼은 한 번 누르면
    무장만 되고 3초 안에 다시 눌러야 실제로 간다

도달 격자는 계산에 몇 초 걸리므로 `sw/logs/reach_cache.json` 에 캐시한다
(한계·여유·격자 간격이 바뀌면 자동 재계산).

사용:
    python sw/scara/ui.py
    python sw/scara/ui.py --grid 20      # 격자 20 mm (기본 25)
"""

import argparse
import io
import json
import logging
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scara import console  # noqa: E402
console.setup()

import matplotlib                                     # noqa: E402
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt                       # noqa: E402
import numpy as np                                    # noqa: E402

for _f in ("Malgun Gothic", "Gulim", "NanumGothic"):
    try:
        import matplotlib.font_manager as _fm
        if any(f.name == _f for f in _fm.fontManager.ttflist):
            plt.rcParams["font.family"] = _f
            break
    except Exception:
        pass
plt.rcParams["axes.unicode_minus"] = False

from scara import collision, config, motion, safety   # noqa: E402
from scara import kinematics as kin                   # noqa: E402
from scara.client import Scara                        # noqa: E402

logging.basicConfig(level=logging.ERROR, format="%(levelname)-7s %(message)s")
D = math.pi / 180.0
R = 180.0 / math.pi
LOG_DIR = Path(__file__).resolve().parents[1] / "logs"

SPEED_FRACTION = 0.15
PHI_SCAN_STEP = 15          # 도달 격자 계산용 phi 간격 [deg]

COL_OK = (0.55, 0.85, 0.55, 0.55)      # 초록 (반투명)
COL_NO = (0.95, 0.65, 0.35, 0.45)      # 주황 (반투명)


# =============================================================================
# 도달 격자
# =============================================================================

def reachable_xy(x_m, y_m):
    """이 (x,y) 로 갈 수 있는 phi 가 하나라도 있는가."""
    if math.hypot(x_m, y_m) > config.MAX_REACH_M + 1e-9:
        return False
    for pd in range(-180, 180, PHI_SCAN_STEP):
        for sol in kin.ik_all(x_m, y_m, pd * D):
            try:
                safety.check_joint_limits(sol.q)
                safety.check_singularity(sol.q)
                collision.check_self_collision(sol.q)
                return True
            except safety.SafetyViolation:
                continue
    return False


def build_grid(step_mm):
    """도달 격자. 캐시가 유효하면 그대로 쓴다."""
    lims = [[round(v * R, 2) for v in lim] for lim in config.JOINT_LIMITS_RAD]
    key = {"step_mm": step_mm, "limits_deg": lims,
           "margin_mm": collision.MARGIN_M * 1000,
           "z_margin_mm": collision.Z_MARGIN_M * 1000,
           "n_pairs": len(collision.PAIRS),
           "phi_step": PHI_SCAN_STEP}
    cache = LOG_DIR / "reach_cache.json"
    if cache.exists():
        try:
            d = json.loads(cache.read_text(encoding="utf-8"))
            if d.get("key") == key:
                print("  도달 격자: 캐시 사용")
                return d["xs"], d["ys"], np.array(d["ok"], dtype=bool)
        except Exception:
            pass

    print("  도달 격자 계산 중 (조건이 바뀌어 캐시 무효) ...", flush=True)
    lim = int(config.MAX_REACH_M * 1000) + step_mm
    xs = list(range(-lim, lim + 1, step_mm))     # 로봇 x (화면 세로)
    ys = list(range(-lim, lim + 1, step_mm))     # 로봇 y (화면 가로)
    salog = logging.getLogger("scara.safety")
    prev = salog.level
    salog.setLevel(logging.ERROR)
    t0 = time.monotonic()
    try:
        ok = np.zeros((len(xs), len(ys)), dtype=bool)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                ok[i, j] = reachable_xy(x / 1000.0, y / 1000.0)
    finally:
        salog.setLevel(prev)
    print(f"  계산 완료 ({time.monotonic()-t0:.1f}초, "
          f"도달 {int(ok.sum())}/{ok.size} 셀). 캐시 저장.")
    LOG_DIR.mkdir(exist_ok=True)
    cache.write_text(json.dumps(
        {"key": key, "xs": xs, "ys": ys, "ok": ok.tolist()}), encoding="utf-8")
    return xs, ys, ok


# =============================================================================
# 실기 상태 폴링
# =============================================================================

class Sampler(threading.Thread):
    """실기 상태 폴링 + 제어루프 감시기.

    UI 를 켠 뒤 드라이버가 Err80.x 로 떨어지는 일이 있다. 방아쇠가 확실하지
    않아서, `time_stamp` 가 멈추면 그 순간의 시각·경과·직전 동작·드라이버
    에러를 콘솔과 `sw/logs/ui_loop_death.json` 에 남긴다.
    """

    def __init__(self, bot, mover=None):
        super().__init__(daemon=True)
        self.bot = bot
        self.mover = mover              # 죽은 순간 무엇을 하고 있었는지 기록용
        self.q = [0.0] * config.DOF
        self.pose = [0.0] * 6
        self.moving = False
        self.stop_flag = threading.Event()
        self.loop_dead = False
        self.events = []                # 감지 기록

    def _report_death(self, frozen_ts, since_s):
        try:
            sv = self.bot.status()["servo_status"]
            pds = sv.get("servo_drive_coe_pds", [])[:3]
            err = ["0x%04X" % e for e in sv.get("servo_drive_error", [])[:3]]
        except Exception:
            pds, err = [], []
        what = getattr(self.mover, "state", "?") if self.mover else "?"
        msg = getattr(self.mover, "msg", "") if self.mover else ""
        ev = {"wall": time.strftime("%H:%M:%S"), "frozen_time_stamp": frozen_ts,
              "dead_after_s": round(since_s, 1), "mover_state": what,
              "mover_msg": msg, "pds": pds, "drive_err": err}
        self.events.append(ev)
        print()
        print("  " + "!" * 62)
        print(f"  ** 제어루프 정지 감지 ** {ev['wall']}")
        print(f"     time_stamp {frozen_ts} 에서 멈춤 (UI 시작 {since_s:.0f}초 후)")
        print(f"     그때 하던 일: state={what}  msg={msg[:50]}")
        print(f"     pds={pds}  drive_err={err}")
        print("     -> Designer 재연결 + Step 4 Verification 필요")
        print("  " + "!" * 62)
        try:
            LOG_DIR.mkdir(exist_ok=True)
            path = LOG_DIR / "ui_loop_death.json"
            old = json.loads(path.read_text(encoding="utf-8")) if path.exists() else []
            old.append(ev)
            path.write_text(json.dumps(old, ensure_ascii=False, indent=1),
                            encoding="utf-8")
            print(f"     기록: {path}")
        except Exception:
            pass

    def run(self):
        t_start = time.monotonic()
        last_ts = None
        last_change = time.monotonic()
        while not self.stop_flag.is_set():
            try:
                tr = self.bot.status()["trajectory_status"]
                self.q = tr["actual_joint_position"][:config.DOF]
                self.pose = tr["actual_pose"]
                ts = tr["time_stamp"]
                now = time.monotonic()
                if ts != last_ts:
                    last_ts, last_change = ts, now
                    self.loop_dead = False
                elif not self.loop_dead and now - last_change > 1.5:
                    self.loop_dead = True
                    self._report_death(ts, now - t_start)
            except Exception:
                pass
            time.sleep(0.05)


# =============================================================================
# 이동 (백그라운드)
# =============================================================================

class Mover:
    """이동 요청을 백그라운드로 처리한다.

    메시지 필드를 두 스레드가 같이 쓰면 안 된다. 이동이 막 끝나는 찰나에
    클릭하면 작업스레드가 "도착" 을 쓰고 -> 클릭 핸들러가 "이동 중" 으로 덮고
    -> 작업스레드가 busy=False 로 끝나서, 이동이 끝났는데도 "이동 중" 이 영영
    남는다. 그래서 셋으로 나눈다:

        result  작업 스레드만 쓴다 (계획/이동/도착/오류)
        hint    클릭 핸들러만 쓴다. 2초 뒤 자동으로 사라진다
        state   idle / planning / moving — 락으로 보호

    msg 는 둘을 합쳐 읽기만 하므로 경쟁이 없다.
    """

    HINT_SEC = 2.0

    def __init__(self, bot):
        self.bot = bot
        self._lock = threading.Lock()
        self.state = "idle"                 # idle / planning / moving
        self.result = "초록 영역을 더블클릭하면 이동한다"
        self._hint = ""
        self._hint_until = 0.0
        self.target = None                  # (x_m, y_m) 표시용

    @property
    def busy(self) -> bool:
        return self.state != "idle"

    @property
    def msg(self) -> str:
        if self._hint and time.monotonic() < self._hint_until:
            return self._hint
        return self.result

    def _hint_now(self, text: str) -> None:
        self._hint = text
        self._hint_until = time.monotonic() + self.HINT_SEC

    def request(self, x_m, y_m, seed_q, acc_ratio):
        with self._lock:
            if self.state != "idle":
                self._hint_now("이동 중 — 끝나면 다시 클릭")
                return
            self.state = "planning"
            self.target = (x_m, y_m)
        self._hint = ""                     # 새 이동이 시작되면 안내는 지운다
        threading.Thread(target=self._run,
                         args=(x_m, y_m, list(seed_q), acc_ratio),
                         daemon=True).start()

    def request_home(self, seed_q, acc_ratio):
        """일직선 자세 q=[0,0,0] 으로 복귀.

        좌표 이동과 **같은 상태머신/락/안전 파이프라인**을 쓴다.
        표적이 관절각이므로 IK(auto_phi/plan_joint_target)만 건너뛴다.
        """
        with self._lock:
            if self.state != "idle":
                self._hint_now("이동 중 — 끝나면 다시")
                return
            self.state = "planning"
            self.target = None              # 홈은 좌표 표적이 없다
        self._hint = ""
        threading.Thread(target=self._run_home,
                         args=(list(seed_q), acc_ratio), daemon=True).start()

    def _run_home(self, seed_q, acc_ratio):
        try:
            q_t = [0.0] * config.DOF
            if max(abs(v) for v in seed_q) * R < 0.5:
                self.result = "이미 일직선 자세다"
                return
            self.result = "홈 계획 중 — 일직선 자세 q=[0,0,0]"
            vel = min(safety.rated_joint_speeds()) * SPEED_FRACTION
            vel = min(vel, motion.tcp_speed_cap(seed_q, q_t))
            if not self.bot.status().get("operation_status", {}).get("manual_mode"):
                self.bot.set_operational_mode("MANUAL")
            self.bot.servo(True)
            self.state = "moving"
            self.result = f"홈(일직선)으로 이동 중 — {vel*R:.0f} deg/s"
            # 좌표 이동과 동일: 가동범위 + 특이점 + 충돌(경로 중간 샘플 포함)
            motion.move_joint_safe(self.bot, q_t, vel=vel, acc=vel * acc_ratio)
            p = self.bot.pose()
            self.result = (f"홈 도착 ({p[0]*1000:.1f}, {p[1]*1000:.1f}) mm "
                           f"(기대 {config.MAX_REACH_M*1000:.0f}, 0) — "
                           "팔이 실제로 일직선인지 눈으로 확인할 것")
        except safety.SafetyViolation as exc:
            self.result = f"홈 거부: {str(exc)[:70]}"
            try:
                self.bot.stop()
            except Exception:
                pass
        except Exception as exc:
            self.result = f"홈 오류: {type(exc).__name__}: {str(exc)[:55]}"
            try:
                self.bot.stop()
            except Exception:
                pass
        finally:
            with self._lock:
                self.state = "idle"
                self.target = None
            self._hint = ""
            self._hint_until = 0.0

    def _run(self, x, y, seed_q, acc_ratio):
        try:
            self.result = f"계획 중: ({x*1000:.0f}, {y*1000:.0f}) mm"
            phi, _ = motion.auto_phi(x, y, seed_q, "both")
            if phi is None:
                self.result = (f"({x*1000:.0f},{y*1000:.0f}) 갈 수 있는 phi 없음 "
                               "— 다른 곳을 더블클릭")
                return
            q_t = motion.plan_joint_target(x, y, phi, seed_q=seed_q)
            vel = min(safety.rated_joint_speeds()) * SPEED_FRACTION
            vel = min(vel, motion.tcp_speed_cap(seed_q, q_t))
            if not self.bot.status().get("operation_status", {}).get("manual_mode"):
                self.bot.set_operational_mode("MANUAL")
            self.bot.servo(True)
            self.state = "moving"
            self.result = (f"이동 중 -> ({x*1000:.0f}, {y*1000:.0f}), "
                           f"phi {phi*R:+.0f}도, {vel*R:.0f} deg/s")
            motion.move_joint_safe(self.bot, q_t, vel=vel, acc=vel * acc_ratio)
            p = self.bot.pose()
            err = math.hypot(p[0] - x, p[1] - y) * 1000
            self.result = (f"도착 ({p[0]*1000:.1f}, {p[1]*1000:.1f}) mm, "
                           f"오차 {err:.2f} mm — 다음 목표를 더블클릭")
        except safety.SafetyViolation as exc:
            self.result = f"거부: {str(exc)[:70]}"
            try:
                self.bot.stop()
            except Exception:
                pass
        except Exception as exc:
            self.result = f"오류: {type(exc).__name__}: {str(exc)[:60]}"
            try:
                self.bot.stop()
            except Exception:
                pass
        finally:
            with self._lock:
                self.state = "idle"
                self.target = None
            # 이동이 끝나면 "이동 중" 안내는 거짓이 되므로 지운다.
            # 안 지우면 힌트 2초가 도착 메시지를 가려서 완료됐는데도
            # "이동 중" 으로 보인다.
            self._hint = ""
            self._hint_until = 0.0


# =============================================================================

def run_ui(bot, grid_mm: int = 25, confirm: bool = True,
           own_client: bool = True) -> int:
    """클릭 UI 를 띄운다.

    Args:
        bot:        저수준 클라이언트 (client.Scara). SCARA 클래스에서는 `.raw`.
        grid_mm:    도달 격자 간격 [mm]
        confirm:    시작 전 Enter 확인을 받을지 (main.py 처럼 이미 확인했으면 False)
        own_client: 종료 시 클라이언트를 닫을지. main.py 가 계속 쓸 거면 False.
    """
    print("=" * 70)
    print("클릭 위치제어 UI")
    print("=" * 70)
    xs, ys, ok = build_grid(grid_mm)
    args = argparse.Namespace(grid=grid_mm)

    problems = safety.startup_checklist(bot)
    if problems:
        print("[기동 전 점검 실패]")
        for p in problems:
            print("  -", p)
        return 1

    if confirm:
        print()
        print("  더블클릭하면 팔이 실제로 움직인다. 주변을 비울 것.")
        try:
            if input("  준비되면 Enter (취소 n) > ").strip().lower().startswith("n"):
                return 130
        except (EOFError, KeyboardInterrupt):
            return 130

    sampler = Sampler(bot)
    mover = Mover(bot)
    sampler.mover = mover          # 루프가 죽은 순간 무엇을 하고 있었는지 기록하려고
    sampler.start()

    # ---- 그림 ----------------------------------------------------------
    # 화면 좌표: 가로 = -로봇y (오른쪽이 로봇 -y), 세로 = 로봇 x (위가 +x)
    #   -> 반시계 회전이 화면에서도 반시계로 보인다.
    fig = plt.figure(figsize=(10.5, 9.2))
    fig.canvas.manager.set_window_title("SCARA 클릭 위치제어 — 초록 더블클릭 = 이동")
    ax = fig.add_axes([0.06, 0.07, 0.76, 0.845])

    # ---- 우측 세로 acc 슬라이더 -------------------------------------------
    # 가속 배수. 3.125 가 공진(6.25 Hz) 여기율 0 이 되는 최적값이라 초록으로 표시.
    from matplotlib.widgets import Slider
    ax_s = fig.add_axes([0.90, 0.15, 0.035, 0.68])
    acc_slider = Slider(ax_s, "acc", 1.0, 6.5,
                        valinit=motion.ACC_RATIO_ZERO_VIB, orientation="vertical",
                        color="#9dbfdd")
    # 여기율 0 이 되는 값들 (가속시간 = 공진주기의 정수배)
    f0 = config.RESONANCE_HZ
    for k, (lw, col) in {1: (2.0, "#b8b8b8"), 2: (3.5, "#0a9a3c"),
                         3: (2.0, "#b8b8b8")}.items():
        ax_s.axhline(f0 / k, lw=lw, color=col, zorder=5)
    ax_s.text(0.5, f0 / 2, "  3.125 최적", fontsize=8, color="#0a9a3c",
              va="center", transform=ax_s.get_yaxis_transform())
    acc_txt = fig.text(0.895, 0.085, "", fontsize=8, ha="left")

    def _acc_info(v):
        n = f0 / v
        exc = abs(math.sin(math.pi * n))
        nl = chr(10)
        return (f"acc {v:.2f}{nl}가속 {1/v:.2f}s{nl}공진주기 {n:.2f}배{nl}여기율 {exc:.2f}"
                + ("  OK" if exc < 0.15 else " !"))

    acc_txt.set_text(_acc_info(acc_slider.val))
    acc_slider.on_changed(lambda v: acc_txt.set_text(_acc_info(v)))

    # 좌상단 Home 버튼 — 일직선 자세(q=[0,0,0])로.
    # 좌표 이동과 **같은 상태머신·락·안전 파이프라인**을 탄다 (Mover.request_home).
    from matplotlib.widgets import Button
    ax_home = fig.add_axes([0.06, 0.945, 0.14, 0.042])
    btn_home = Button(ax_home, "Home (일직선)", color="#dfe9f3",
                      hovercolor="#c3d8ef")
    # 지도는 "더블클릭만 명령으로 인정" 해서 실수 클릭을 막는다. 버튼은 한 번
    # 클릭이라 스치기만 해도 팔이 움직이므로, 같은 취지로 **두 번 눌러야** 간다.
    HOME_ARM_SEC = 3.0
    home_arm = {"until": 0.0}

    def _home_label(text):
        btn_home.label.set_text(text)

    def _on_home(ev):
        now = time.monotonic()
        if now > home_arm["until"]:                 # 1차: 무장만 한다
            home_arm["until"] = now + HOME_ARM_SEC
            _home_label("한 번 더 = 이동")
            mover._hint_now("Home: 한 번 더 누르면 일직선 자세로 이동")
            return
        home_arm["until"] = 0.0                     # 2차: 실행
        _home_label("Home (일직선)")
        mover.request_home(sampler.q, float(acc_slider.val))

    btn_home.on_clicked(_on_home)

    img = np.zeros((len(xs), len(ys), 4))
    img[ok] = COL_OK
    img[~ok] = COL_NO
    # imshow: 행=세로. 행 = xs(로봇 x), 열 = ys(로봇 y). 화면 가로는 -y 이므로 열 뒤집기
    # origin="lower" 라 array[0,0] 이 (왼쪽, 아래) = (로봇 y 최대, 로봇 x 최소).
    # 행(로봇 x)은 그대로, 열(로봇 y)만 뒤집는다. (행까지 뒤집으면 상하 반전 버그)
    img_show = img[:, ::-1]
    extent = [-max(ys) - args.grid / 2, -min(ys) + args.grid / 2,
              min(xs) - args.grid / 2, max(xs) + args.grid / 2]
    ax.imshow(img_show, extent=extent, origin="lower", interpolation="nearest",
              zorder=0)

    (arm_ln,) = ax.plot([], [], "-", lw=5, color="#1f3b57",
                        solid_capstyle="round", zorder=5)
    # scatter 는 빈 좌표에 크기 4개를 주면 에러가 난다 -> 더미 4점으로 초기화
    joints_sc = ax.scatter([0, 0, 0, 0], [0, 0, 0, 0],
                           s=[90, 70, 70, 110], zorder=6,
                           c=["#12253a", "#12253a", "#12253a", "#c40060"])
    (target_ln,) = ax.plot([], [], "x", ms=14, mew=3, color="#c40060", zorder=7)

    # 베이스를 화면 **중앙**에 — J1 이 +-149도라 사실상 360도를 쓴다
    ax.set_xlim(-700, 700)
    ax.set_ylim(-700, 700)
    ax.set_aspect("equal")
    ax.set_xlabel("← +y      화면 가로 [mm]      -y →")
    ax.set_ylabel("로봇 +x [mm]  (베이스에서 팔이 뻗는 방향)")
    ax.grid(alpha=0.25, zorder=1)
    status = ax.set_title(mover.msg, fontsize=11)

    # 중앙 상단 사람 표시 — 화면 방향을 가늠하는 기준점.
    # Malgun Gothic 에는 이모지가 없어 Segoe UI Emoji 를 따로 지정한다.
    ax.text(0.5, 0.965, "🧍", transform=ax.transAxes,
            ha="center", va="top", fontsize=22, zorder=8,
            fontfamily=["Segoe UI Emoji", "Segoe UI Symbol"])

    def to_screen(x_m, y_m):
        return -y_m * 1000, x_m * 1000

    def to_robot(sx, sy):
        return sy / 1000.0, -sx / 1000.0

    def cell_ok(x_m, y_m):
        i = round((x_m * 1000 - xs[0]) / args.grid)
        j = round((y_m * 1000 - ys[0]) / args.grid)
        if 0 <= i < len(xs) and 0 <= j < len(ys):
            return bool(ok[i, j])
        return False

    def on_click(ev):
        if not ev.dblclick or ev.inaxes is not ax:
            return
        x_m, y_m = to_robot(ev.xdata, ev.ydata)
        if not cell_ok(x_m, y_m):
            mover.msg = (f"({x_m*1000:.0f}, {y_m*1000:.0f}) 는 주황(불가) 영역이다")
            return
        mover.request(x_m, y_m, sampler.q, float(acc_slider.val))

    fig.canvas.mpl_connect("button_press_event", on_click)
    running = {"on": True}
    fig.canvas.mpl_connect("close_event", lambda e: running.update(on=False))

    try:
        while running["on"]:
            q = sampler.q
            pts = kin.joint_positions(q)
            sx = [to_screen(p[0], p[1])[0] for p in pts]
            sy = [to_screen(p[0], p[1])[1] for p in pts]
            arm_ln.set_data(sx, sy)
            joints_sc.set_offsets(np.column_stack([sx, sy]))
            if mover.target:
                tx, ty = to_screen(*mover.target)
                target_ln.set_data([tx], [ty])
            else:
                target_ln.set_data([], [])
            status.set_text(mover.msg)
            if home_arm["until"] and time.monotonic() > home_arm["until"]:
                home_arm["until"] = 0.0
                _home_label("Home (일직선)")
            plt.pause(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        sampler.stop_flag.set()
        try:
            bot.stop(); time.sleep(0.3); bot.servo(False)
        except Exception:
            pass
        if own_client:
            bot.close()
        print("UI 종료")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=int, default=25, help="격자 간격 [mm]")
    args = ap.parse_args()
    bot = Scara()
    try:
        return run_ui(bot, grid_mm=args.grid, confirm=True, own_client=True)
    except Exception:
        try:
            bot.close()
        except Exception:
            pass
        raise


if __name__ == "__main__":
    raise SystemExit(main())
