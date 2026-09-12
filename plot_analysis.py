"""Групповой анализ plot_recordings/ по «сетам» коэффициентов.

Один набор = префикс в label до последнего «W_WW_rad_s». Например:
   6_28_rad_s        → set = "base"  (без буквенного префикса)
   A_6_28_rad_s      → set = "A"
   B_3_14_rad_s      → set = "B"

Для каждого сета отдельная кривая на Bode-графике (амплитуда, фаза vs
частота) и отдельная колонка на time-series картинке.
"""
import csv
import glob
import json
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DIR = os.path.join(os.path.dirname(__file__), "plot_recordings")
OUT = os.path.join(DIR, "_analysis")
os.makedirs(OUT, exist_ok=True)

# Регекс: <set-prefix>_<W>_<WW>_rad_s. set-prefix = «base» если только
# скорость (нет буквенного тега). Из имени файла берём часть после
# «plot_YYYYMMDD_HHMMSS_».
# Разбор label: последние <W>_<WW>_rad_s = скорость, всё что перед — имя
# сета. Non-greedy prefix + backtrack гарантирует, что для «B1_1_3_14»
# префикс станет «B1_1», а не «B1» (иначе скорость получилась бы 1.3).
# Пустой префикс → сет 'base'. Trailing '_' в префиксе стрипаем (артефакт
# санитайзера апострофов в ui.py).
LABEL_RE = re.compile(r"^(?:(.+?)_)?(\d+)_(\d+)_rad_s$")


def load(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(x["t_s"]) for x in rows])
    nx = np.array([float(x["nx_px"]) for x in rows])
    pot = np.array([float(x["pot_px"]) for x in rows])
    pred = np.array([float(x["err_pred_px"]) for x in rows])
    om = np.array([float(x["omega_out"]) for x in rows])
    return t, nx, pot, pred, om


def parse_label(fname):
    """Возвращает (set_name, omega_rad_s). None если не парсится."""
    tag = "_".join(os.path.basename(fname).replace(".csv", "").split("_")[3:])
    m = LABEL_RE.match(tag)
    if not m:
        return None
    set_name = (m.group(1) or "base").strip("_") or "base"
    w = float(f"{m.group(2)}.{m.group(3)}")
    return set_name, w


def sinefit(t, y, w):
    s, c, o = np.sin(w * t), np.cos(w * t), np.ones_like(t)
    a, b, C = np.linalg.lstsq(np.column_stack([s, c, o]), y, rcond=None)[0]
    resid = y - (a * s + b * c + C)
    rms_y = float(np.sqrt(np.mean(y ** 2)))
    rms_r = float(np.sqrt(np.mean(resid ** 2)))
    coh = 1.0 - (rms_r / rms_y if rms_y > 1e-9 else 1.0)
    return float(np.hypot(a, b)), float(np.arctan2(b, a)), float(C), coh


def build_dataset():
    files = sorted(glob.glob(os.path.join(DIR, "plot_*.csv")))
    by_set: dict = {}
    for f in files:
        parsed = parse_label(f)
        if not parsed:
            continue
        set_name, w = parsed
        t, nx, pot, pred, om = load(f)
        # Отрезаем первые 2с (переходной после нажатия REC).
        m = t >= t[0] + 2.0
        if m.sum() < 30:
            continue
        t = t[m] - t[m][0]
        nx, pot, pred, om = nx[m], pot[m], pred[m], om[m]
        A_nx, phi_nx, _, coh_nx = sinefit(t, nx, w)
        A_pot, phi_pot, C_pot, coh_pot = sinefit(t, pot, w)
        A_pred, phi_pred, _, _ = sinefit(t, pred, w)
        # Явно: pot двигает камеру, nx = ball_world - camera_world.
        # Значит ball_world = pot + nx (в правильных фазорах).
        A_ball = float(abs(A_pot * np.exp(1j * phi_pot) +
                           A_nx * np.exp(1j * phi_nx)))
        # Загружаем метапараметры (для легенды и sat-порога).
        with open(f.replace(".csv", ".json")) as fp:
            meta = json.load(fp)["params"]
        max_om = float(meta["max_omega"])
        sat_frac = float(np.mean(np.abs(om) >= 0.98 * max_om))
        rec = dict(t=t, nx=nx, pot=pot, pred=pred, om=om, w=w,
                   freq=w / (2 * np.pi), A_nx=A_nx, A_pot=A_pot,
                   A_pred=A_pred, A_ball=A_ball, phi_nx=phi_nx,
                   phi_pot=phi_pot, phi_pred=phi_pred, coh_nx=coh_nx,
                   coh_pot=coh_pot, sat_frac=sat_frac, meta=meta,
                   fname=os.path.basename(f))
        by_set.setdefault(set_name, []).append(rec)
    # По каждой скорости в сете может быть несколько дублей — оставим
    # последний по времени (то есть более свежий).
    for name in by_set:
        by_freq = {}
        for r in by_set[name]:
            by_freq[r["w"]] = r  # последняя запись перепишет более раннюю
        by_set[name] = sorted(by_freq.values(), key=lambda r: r["w"])
    return by_set


def print_table(by_set):
    def h(s): return f"── {s} " + "─" * max(0, 90 - len(s))
    for name in sorted(by_set.keys()):
        recs = by_set[name]
        p = recs[0]["meta"]
        print()
        print(h(f"set '{name}':  kp={p['kp']:.2f} td={p['td']:.3f} "
                f"maxΩ={p['max_omega']:.0f} kdpot={p['kd_pot']:.2f}"
                f" pg={p['predict_gain']:.1f} off={p['latency_offset_ms']:.0f}ms"))
        print(" ω,r/s │ f,Hz │ |nx| │|pot|│|ball│ err%│ dφ(pot-nx)│dφ(pred-nx)│ ωmax│ sat│ coh_nx│coh_pot")
        for r in recs:
            def wrap(x): return (x + np.pi) % (2 * np.pi) - np.pi
            dph_pot = wrap(r["phi_pot"] - r["phi_nx"]) * 180 / np.pi
            dph_pred = wrap(r["phi_pred"] - r["phi_nx"]) * 180 / np.pi
            err_pct = r["A_nx"] / max(1e-3, r["A_ball"]) * 100
            om_max = float(np.max(np.abs(r["om"])))
            print(f" {r['w']:5.2f} │{r['freq']:5.2f} │ {r['A_nx']:4.1f} │"
                  f"{r['A_pot']:4.1f} │{r['A_ball']:4.1f} │{err_pct:4.0f}%│ "
                  f"{dph_pot:+7.1f}°  │ {dph_pred:+6.1f}°  │ "
                  f"{om_max:4.0f}│ {r['sat_frac']*100:2.0f}%│ "
                  f"{r['coh_nx']:5.2f} │ {r['coh_pot']:5.2f}")


def plot_bode(by_set):
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    colors = {"base": "C7", "A": "C0", "B": "C2", "C": "C3"}

    def wrap(x): return (x + np.pi) % (2 * np.pi) - np.pi
    for name in sorted(by_set.keys()):
        recs = by_set[name]
        p = recs[0]["meta"]
        lbl = f"{name}: Kp={p['kp']:.1f} MaxΩ={p['max_omega']:.0f} Kdpot={p['kd_pot']:.2f}"
        col = colors.get(name, None)
        f = [r["freq"] for r in recs]
        axes[0].plot(f, [r["A_nx"] / max(1e-3, r["A_ball"]) * 100 for r in recs],
                     "o-", color=col, lw=2, label=lbl)
        axes[1].plot(f, [wrap(r["phi_pot"] - r["phi_nx"]) * 180 / np.pi
                         for r in recs], "s-", color=col, lw=2, label=lbl)
        axes[2].plot(f, [r["coh_nx"] for r in recs], "d-", color=col, lw=1.5,
                     label=f"{name} nx")
        axes[2].plot(f, [r["coh_pot"] for r in recs], "d:", color=col, lw=1.0,
                     label=f"{name} pot")

    axes[0].set_ylabel("Tracking error |nx| / |ball|, %")
    axes[0].set_title("Closed-loop tracking error vs ball frequency")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8, loc="best")
    axes[0].set_ylim(0, 100)

    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("φ(pot) − φ(nx), deg")
    axes[1].set_title("Phase between shaft and camera-error")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8, loc="best")

    axes[2].set_ylabel("Coherence (1=clean sine)")
    axes[2].set_title("Signal cleanliness — <0.7 = harmonics/limit-cycle/noise")
    axes[2].set_xlabel("Ball frequency (Hz)")
    axes[2].set_ylim(0, 1.05)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(fontsize=8, loc="best", ncol=2)
    axes[2].set_xscale("log")

    fig.tight_layout()
    p = os.path.join(OUT, "bode_compare.png")
    fig.savefig(p, dpi=110)
    print("saved", p)


def plot_traces_for_set(by_set, name):
    recs = by_set.get(name, [])
    if not recs:
        return
    fig, axes = plt.subplots(len(recs), 1, figsize=(11, 2.2 * len(recs)),
                             sharex=False)
    if len(recs) == 1:
        axes = [axes]
    p = recs[0]["meta"]
    fig.suptitle(f"Set '{name}':  Kp={p['kp']:.1f}  Td={p['td']:.3f}  "
                 f"MaxΩ={p['max_omega']:.0f}  Kdpot={p['kd_pot']:.2f}  "
                 f"pg={p['predict_gain']:.1f}  offset={p['latency_offset_ms']:.0f}ms",
                 fontsize=11)
    for ax, r in zip(axes, recs):
        m = r["t"] <= 8.0
        ax.plot(r["t"][m], r["nx"][m], color="C0", lw=1.6, label="nx (camera)")
        ax.plot(r["t"][m], r["pot"][m], color="C3", lw=1.4, label="pot (shaft)")
        ax.plot(r["t"][m], r["pred"][m], color="C2", lw=0.9, alpha=0.7,
                label="err → STM32")
        ax.plot(r["t"][m], r["om"][m], color="C4", lw=0.6, alpha=0.6,
                label="ω_out")
        ax.axhline(r["meta"]["max_omega"], color="C4", ls=":", lw=0.4)
        ax.axhline(-r["meta"]["max_omega"], color="C4", ls=":", lw=0.4)
        ax.axhline(0, color="k", lw=0.3)
        err_pct = r["A_nx"] / max(1e-3, r["A_ball"]) * 100
        ax.set_title(f"ω={r['w']:.2f} rad/s ({r['freq']:.2f} Hz)   "
                     f"|nx|={r['A_nx']:.0f}px  |pot|={r['A_pot']:.0f}px  "
                     f"|ball|~{r['A_ball']:.0f}px  err={err_pct:.0f}%  "
                     f"coh_nx={r['coh_nx']:.2f}", fontsize=10)
        ax.set_ylim(-160, 160)
        ax.grid(True, alpha=0.3)
        if ax is axes[0]:
            ax.legend(fontsize=8, loc="upper right", ncol=4)
    axes[-1].set_xlabel("t, s")
    fig.tight_layout()
    p = os.path.join(OUT, f"traces_{name}.png")
    fig.savefig(p, dpi=110)
    print("saved", p)


if __name__ == "__main__":
    ds = build_dataset()
    print_table(ds)
    plot_bode(ds)
    for name in ds:
        plot_traces_for_set(ds, name)
