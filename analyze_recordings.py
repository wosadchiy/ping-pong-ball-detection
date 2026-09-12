"""Спектральный + временной анализ серии записей plot_recordings/.

Задача: понять, как одна и та же настройка регулятора ведёт себя на
разных частотах вращения мяча. Извлекаем для каждой записи:

* Амплитуду и фазу трёх каналов (nx, pot_px, err_pred_px) на
  фундаментальной частоте вращения (метка в имени в rad/s).
* Модуль «замкнутого контура» |pot/err_pred| — насколько шафт вообще
  отрабатывает командный сигнал.
* Модуль ошибки регулирования |nx|/|pot| — во сколько раз мы «промахнулись»
  относительно того, что вал сделал.
* Долю времени, которую привод провёл в насыщении (|ω|≥0.98·MaxΩ).
* RMS каждого канала — грубая проверка формы.
"""

import csv
import glob
import json
import os
import sys
import numpy as np

DIR = os.path.join(os.path.dirname(__file__), "plot_recordings")


def load(csv_path):
    with open(csv_path) as f:
        r = csv.DictReader(f)
        rows = list(r)
    t = np.array([float(x["t_s"]) for x in rows])
    nx = np.array([float(x["nx_px"]) for x in rows])
    pot = np.array([float(x["pot_px"]) for x in rows])
    pred = np.array([float(x["err_pred_px"]) for x in rows])
    omega = np.array([float(x["omega_out"]) for x in rows])
    return t, nx, pot, pred, omega


def parse_omega_from_label(label: str) -> float:
    # "6_28_rad_s" -> 6.28
    parts = label.replace("rad_s", "").strip("_").split("_")
    try:
        return float(parts[0] + "." + parts[1])
    except (IndexError, ValueError):
        return float("nan")


def sine_fit(t, y, omega):
    """Fit y ≈ A·sin(ωt + φ) + C via least-squares.

    y_hat = A·sin(ωt)·cos(φ) + A·cos(ωt)·sin(φ) + C
          = a·sin + b·cos + c;  A=√(a²+b²), φ=atan2(b,a)
    """
    s = np.sin(omega * t)
    c = np.cos(omega * t)
    ones = np.ones_like(t)
    M = np.column_stack([s, c, ones])
    coef, *_ = np.linalg.lstsq(M, y, rcond=None)
    a, b, C = coef
    A = float(np.hypot(a, b))
    phi = float(np.arctan2(b, a))
    resid = y - (a * s + b * c + C)
    rms_signal = float(np.sqrt(np.mean(y ** 2)))
    rms_resid = float(np.sqrt(np.mean(resid ** 2)))
    coherence = 1.0 - (rms_resid / rms_signal if rms_signal > 1e-9 else 1.0)
    return A, phi, C, coherence


def analyze_one(csv_path):
    label = os.path.basename(csv_path).replace(".csv", "")
    # берём часть после plot_YYYYMMDD_HHMMSS_
    tag = "_".join(label.split("_")[3:])
    ball_omega = parse_omega_from_label(tag)
    if not np.isfinite(ball_omega):
        return None

    t, nx, pot, pred, omega_cmd = load(csv_path)
    if len(t) < 30:
        return None
    # Отрезаем первые 2 с (переходной процесс после команды REC).
    mask = t >= (t[0] + 2.0)
    t = t[mask] - t[mask][0]
    nx, pot, pred, omega_cmd = nx[mask], pot[mask], pred[mask], omega_cmd[mask]

    with open(csv_path.replace(".csv", ".json")) as f:
        meta = json.load(f)
    params = meta["params"]
    max_omega = float(params["max_omega"])

    A_nx, phi_nx, C_nx, coh_nx = sine_fit(t, nx, ball_omega)
    A_pot, phi_pot, C_pot, coh_pot = sine_fit(t, pot, ball_omega)
    A_pred, phi_pred, _, _ = sine_fit(t, pred, ball_omega)

    # Насыщение
    sat_frac = float(np.mean(np.abs(omega_cmd) >= 0.98 * max_omega))
    peak_omega = float(np.max(np.abs(omega_cmd)))
    rms_omega = float(np.sqrt(np.mean(omega_cmd ** 2)))

    # Фазовые сдвиги (relative to nx)
    def wrap(x):
        return (x + np.pi) % (2 * np.pi) - np.pi
    d_phi_pot_nx = wrap(phi_pot - phi_nx) * 180 / np.pi
    d_phi_pred_nx = wrap(phi_pred - phi_nx) * 180 / np.pi

    # Простейшая оценка эффективной задержки от вала относительно nx:
    # τ ≈ -Δφ / ω. Отрицательный сдвиг = вал отстаёт.
    tau_pot_ms = -d_phi_pot_nx / 360 * (2 * np.pi / ball_omega) * 1000

    return {
        "label": tag,
        "omega_rad_s": ball_omega,
        "freq_hz": ball_omega / (2 * np.pi),
        "period_s": 2 * np.pi / ball_omega,
        "duration_s": float(t[-1]),
        "A_nx": A_nx, "A_pot": A_pot, "A_pred": A_pred,
        "coh_nx": coh_nx, "coh_pot": coh_pot,
        "dphi_pot_nx_deg": d_phi_pot_nx,
        "dphi_pred_nx_deg": d_phi_pred_nx,
        "tau_pot_lag_ms": tau_pot_ms,
        "sat_frac": sat_frac,
        "peak_omega": peak_omega,
        "rms_omega": rms_omega,
        "rms_nx": float(np.sqrt(np.mean(nx ** 2))),
        "rms_pot": float(np.sqrt(np.mean(pot ** 2))),
        "C_nx": C_nx, "C_pot": C_pot,
    }


def main():
    files = sorted(glob.glob(os.path.join(DIR, "*.csv")))
    results = [r for r in (analyze_one(f) for f in files) if r]
    results.sort(key=lambda r: r["omega_rad_s"])

    print()
    print(" ω, rad/s │ f, Hz │  |nx| │ |pot| │ |pred│  φ(pot-nx) │ φ(pred-nx)│  τlag  │ ωmax │ sat%│ coh_nx│ coh_pot│ bias_nx│ bias_pot")
    print("──────────┼───────┼───────┼───────┼──────┼────────────┼───────────┼────────┼──────┼─────┼───────┼────────┼────────┼─────────")
    for r in results:
        print(f"  {r['omega_rad_s']:5.2f}   │ {r['freq_hz']:5.2f} │ "
              f"{r['A_nx']:5.1f} │ {r['A_pot']:5.1f} │ "
              f"{r['A_pred']:4.1f} │  {r['dphi_pot_nx_deg']:+7.1f}°  │  "
              f"{r['dphi_pred_nx_deg']:+6.1f}°  │ "
              f"{r['tau_pot_lag_ms']:+5.0f}ms │ {r['peak_omega']:4.0f} │ "
              f"{r['sat_frac']*100:3.0f}% │ {r['coh_nx']:5.2f} │ "
              f"{r['coh_pot']:5.2f}  │ {r['C_nx']:+5.1f} │ {r['C_pot']:+5.1f}")

    print()
    print("Заметки к таблице:")
    print("  |nx|, |pot|, |pred|  — амплитуды на фундаментальной частоте (px).")
    print("  φ(pot-nx)   — фазовый сдвиг вала относительно nx. Отрицательный ⇒ вал отстаёт.")
    print("  τlag        — та же величина в мс.")
    print("  ωmax        — пиковое |omega_out|; MaxΩ=100. Если ≥98 → SAT.")
    print("  sat%        — доля времени в насыщении.")
    print("  coh_nx/pot  — «когерентность» на фунд. частоте: 1.0=чистая синусоида,")
    print("                <0.9 значит много шума/гармоник в сигнале.")
    print("  bias_nx/pot — среднее (DC-компонента). Идеально 0; заметный сдвиг")
    print("                = смещение центра pot либо камеры.")


if __name__ == "__main__":
    main()
