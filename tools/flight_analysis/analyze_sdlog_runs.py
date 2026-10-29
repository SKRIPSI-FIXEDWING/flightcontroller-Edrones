"""Metrik per run dari log SD (format v2, fc::FlightLog) + uji statistik L1 vs L1+fuzzy.

Pemakaian (dari tools/flight_analysis, dengan venv):
    .venv/Scripts/python.exe analyze_sdlog_runs.py LOG007.CSV LOG008.CSV --out hasil/
    .venv/Scripts/python.exe analyze_sdlog_runs.py *.CSV --compare          # Mann-Whitney antar metode
    .venv/Scripts/python.exe analyze_sdlog_runs.py *.CSV --compare --paired # Wilcoxon (run berpasangan)

Definisi (docs/flight-log-csv.md, "Metrik"):
  run          satu rentang berurutan mode == AUTO.
  segmen eval  baris di dalam run dengan leg_from_wp == 1 (garis acuan sudah
               dimulai di WP, bukan di posisi masuk AUTO) dan GPS 3D.
  pulih        |cte| <= --band (m) bertahan >= --hold (s), dihitung dari DIST_OFF.
  overshoot    simpangan terbesar di sisi SEBERANG garis setelah gangguan
               (sisi gangguan = tanda cte di puncak |cte| selama/segera setelah gangguan).
Hanya numpy.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys

import numpy as np

from sdlog_io import SdLog, read_sdlog

# ----------------------------------------------------------------------------
# Run & window detection
# ----------------------------------------------------------------------------


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """[(awal, akhir_eksklusif)] untuk tiap rentang True berurutan."""
    runs: list[tuple[int, int]] = []
    start = None
    for i, flag in enumerate(mask):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i))
            start = None
    if start is not None:
        runs.append((start, len(mask)))
    return runs


def disturbance_windows(log: SdLog, t0_ms: float, t1_ms: float) -> list[dict]:
    """Pasangkan DIST_ON ... DIST_OFF di dalam [t0, t1]; DIST_ON berulang = ganti level."""
    windows: list[dict] = []
    open_win: dict | None = None
    for ev in log.events:
        if ev.t_ms < t0_ms or ev.t_ms > t1_ms:
            continue
        if ev.kind == "DIST_ON":
            level = int(float(ev.args[0])) if ev.args else 0
            if open_win is None:
                open_win = dict(start_ms=ev.t_ms, end_ms=None, level_us=abs(level), first_us=level)
            else:
                open_win["level_us"] = max(open_win["level_us"], abs(level))
        elif ev.kind == "DIST_OFF" and open_win is not None:
            open_win["end_ms"] = ev.t_ms
            windows.append(open_win)
            open_win = None
    if open_win is not None:  # masih aktif saat run berakhir
        open_win["end_ms"] = t1_ms
        windows.append(open_win)
    return windows


def recovery_metrics(t_s: np.ndarray, cte: np.ndarray, win: dict, band: float, hold_s: float,
                     horizon_s: float = 30.0) -> dict:
    t_start = win["start_ms"] / 1000.0
    t_end = win["end_ms"] / 1000.0
    during = (t_s >= t_start) & (t_s <= t_end + 2.0)
    out = dict(peak_abs_cte=float("nan"), recovered=False, recovery_s=float("nan"), overshoot_m=float("nan"))
    if not during.any():
        return out
    idx_peak = np.flatnonzero(during)[np.argmax(np.abs(cte[during]))]
    out["peak_abs_cte"] = float(abs(cte[idx_peak]))
    side = 1.0 if cte[idx_peak] >= 0 else -1.0

    after = np.flatnonzero((t_s >= t_end) & (t_s <= t_end + horizon_s))
    if len(after) == 0:
        return out
    inside = np.abs(cte) <= band
    for k in after:
        window = (t_s >= t_s[k]) & (t_s <= t_s[k] + hold_s)
        if t_s[k] + hold_s > t_s[after[-1]] + 1e-9:
            break  # tidak cukup data untuk memastikan bertahan selama hold
        if inside[k] and inside[window].all():
            out["recovered"] = True
            out["recovery_s"] = float(t_s[k] - t_end)
            break
    end_idx = after[-1]
    if out["recovered"]:
        end_idx = np.flatnonzero(t_s <= t_end + out["recovery_s"] + hold_s)[-1]
    tail = np.arange(after[0], end_idx + 1)
    out["overshoot_m"] = float(max(0.0, np.max(-side * cte[tail]))) if len(tail) else float("nan")
    return out


# ----------------------------------------------------------------------------
# Per-run metrics
# ----------------------------------------------------------------------------


def zero_crossings(cte: np.ndarray, hysteresis: float = 2.0) -> int:
    """Perpotongan nol CTE; baru dihitung bila sisi berganti melewati +-hysteresis (buang noise di garis)."""
    side = 0
    count = 0
    for value in cte:
        if value > hysteresis:
            new = 1
        elif value < -hysteresis:
            new = -1
        else:
            continue
        if side != 0 and new != side:
            count += 1
        side = new
    return count


def analyze_log(log: SdLog, band: float, hold_s: float, limit_m: float) -> tuple[list[dict], list[dict]]:
    col = log.col
    if log.n_rows == 0:
        return [], []
    t_s = col["time_boot_ms"] / 1000.0
    runs: list[dict] = []
    dist_rows: list[dict] = []

    for run_no, (i0, i1) in enumerate(contiguous_runs(col["mode"] == "AUTO"), start=1):
        sl = slice(i0, i1)
        eff = log.effective_at(col["time_boot_ms"][i0])
        roll_limit = eff.get("roll_limit_deg", 45.0)
        ev_mask = (col["leg_from_wp"][sl] == 1) & (col["gps_fix"][sl] >= 2)
        cte = col["cte_m"][sl][ev_mask]
        tt = t_s[sl][ev_mask]
        row = dict(
            file=os.path.basename(log.path), run=run_no,
            t_start_s=t_s[i0], t_end_s=t_s[i1 - 1], duration_s=t_s[i1 - 1] - t_s[i0],
            # format 3 (2026-10-29) tidak punya kolom fuzzy_on: firmware tanpa fuzzy = L1 tetap
            fuzzy_on=int(np.mean(col["fuzzy_on"][sl]) >= 0.5) if "fuzzy_on" in col else 0,
            l1_base_period=eff.get("l1_period_now", float("nan")),
            l1_damping=eff.get("l1_damping", float("nan")),
            wp_radius_m=eff.get("wp_radius_m", float("nan")),
            roll_limit_deg=roll_limit,
            n_eval=int(ev_mask.sum()),
            eval_duration_s=float(tt[-1] - tt[0]) if len(tt) > 1 else 0.0,
        )
        if len(cte) > 0:
            a = np.abs(cte)
            row.update(
                cte_abs_mean=float(a.mean()), cte_rms=float(np.sqrt(np.mean(cte ** 2))),
                cte_abs_max=float(a.max()), pct_within_limit=float(100.0 * np.mean(a <= limit_m)),
                zero_crossings=zero_crossings(cte),
                roll_rms_deg=float(np.sqrt(np.mean(col["roll_deg"][sl][ev_mask] ** 2))),
                nav_roll_saturated_frac=float(np.mean(np.abs(col["nav_roll_deg"][sl][ev_mask]) >= roll_limit - 0.5)),
                l1_period_mean=float(col["l1_period_s"][sl][ev_mask].mean()),
                airspeed_mean=float(col["airspeed_mps"][sl][ev_mask].mean()),
                gs_mean=float(col["gs_mps"][sl][ev_mask].mean()),
                throttle_ch3_mean=float(col["ch3"][sl][ev_mask].mean()),
            )
        wp_events = [e for e in log.events if e.kind == "WP_REACHED" and
                     col["time_boot_ms"][i0] <= e.t_ms <= col["time_boot_ms"][i1 - 1]]
        row["wp_claims"] = ";".join(f"{e.args[0]}:{e.args[1]}" for e in wp_events if len(e.args) >= 2)

        windows = disturbance_windows(log, col["time_boot_ms"][i0], col["time_boot_ms"][i1 - 1])
        row["dist_windows"] = len(windows)
        row["dist_level_us"] = max([w["level_us"] for w in windows], default=0)
        runs.append(row)

        for w_no, win in enumerate(windows, start=1):
            rec = recovery_metrics(t_s[sl], col["cte_m"][sl], win, band, hold_s)
            dist_rows.append(dict(file=row["file"], run=run_no, window=w_no,
                                  fuzzy_on=row["fuzzy_on"], level_us=win["level_us"],
                                  start_s=win["start_ms"] / 1000.0, end_s=win["end_ms"] / 1000.0, **rec))
    return runs, dist_rows


# ----------------------------------------------------------------------------
# Statistics (tanpa scipy)
# ----------------------------------------------------------------------------


def _midranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values))
    sorted_vals = values[order]
    i = 0
    while i < len(values):
        j = i
        while j + 1 < len(values) and sorted_vals[j + 1] == sorted_vals[i]:
            j += 1
        ranks[order[i:j + 1]] = 0.5 * (i + j) + 1.0
        i = j + 1
    return ranks


def mann_whitney_u(a, b) -> tuple[float, float]:
    """(U_a, p dua sisi). Tepat (permutasi) bila n1+n2 <= 20, selain itu aproksimasi normal + koreksi ties."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    a, b = a[~np.isnan(a)], b[~np.isnan(b)]
    n1, n2 = len(a), len(b)
    if n1 == 0 or n2 == 0:
        return float("nan"), float("nan")
    allv = np.concatenate([a, b])
    ranks = _midranks(allv)
    u_a = ranks[:n1].sum() - n1 * (n1 + 1) / 2.0
    mean_u = n1 * n2 / 2.0
    if n1 + n2 <= 20:
        count = total = 0
        for combo in itertools.combinations(range(n1 + n2), n1):
            u = ranks[list(combo)].sum() - n1 * (n1 + 1) / 2.0
            total += 1
            if abs(u - mean_u) >= abs(u_a - mean_u) - 1e-9:
                count += 1
        return float(u_a), count / total
    n = n1 + n2
    _, counts = np.unique(allv, return_counts=True)
    tie = (counts ** 3 - counts).sum()
    sigma = math.sqrt(n1 * n2 / 12.0 * ((n + 1) - tie / (n * (n - 1)))) if n > 1 else 0.0
    if sigma == 0:
        return float(u_a), 1.0
    z = (abs(u_a - mean_u) - 0.5) / sigma
    return float(u_a), float(math.erfc(max(z, 0.0) / math.sqrt(2.0)))


def wilcoxon_signed_rank(a, b) -> tuple[float, float]:
    """(W+, p dua sisi) untuk data berpasangan. Tepat bila n <= 16, selain itu aproksimasi normal."""
    d = np.asarray(a, float) - np.asarray(b, float)
    d = d[~np.isnan(d)]
    d = d[d != 0]
    n = len(d)
    if n == 0:
        return float("nan"), float("nan")
    ranks = _midranks(np.abs(d))
    w_plus = ranks[d > 0].sum()
    mean_w = n * (n + 1) / 4.0
    if n <= 16:
        total = 1 << n
        count = 0
        for mask in range(total):
            signs = np.array([(mask >> k) & 1 for k in range(n)], bool)
            w = ranks[signs].sum()
            if abs(w - mean_w) >= abs(w_plus - mean_w) - 1e-9:
                count += 1
        return float(w_plus), count / total
    sigma = math.sqrt(n * (n + 1) * (2 * n + 1) / 24.0)
    z = (abs(w_plus - mean_w) - 0.5) / sigma
    return float(w_plus), float(math.erfc(max(z, 0.0) / math.sqrt(2.0)))


# ----------------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------------

COMPARE_METRICS = ["cte_abs_mean", "cte_rms", "cte_abs_max", "pct_within_limit", "zero_crossings",
                   "roll_rms_deg", "nav_roll_saturated_frac"]


def write_csv(path: str, rows: list[dict]) -> None:
    if not rows:
        return
    keys: list[str] = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value) -> str:
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def compare_methods(runs: list[dict], dist_rows: list[dict], paired: bool) -> None:
    print("\n== Perbandingan metode: L1 tetap (fuzzy_on=0) vs L1 + fuzzy (fuzzy_on=1) ==")
    print("   uji:", "Wilcoxon signed-rank (berpasangan, urut waktu)" if paired else "Mann-Whitney U (tak berpasangan)",
          "| p dua sisi; p < 0,05 = beda signifikan; efek = selisih rata-rata fuzzy terhadap L1 tetap (%)")
    levels = sorted({r["dist_level_us"] for r in runs})
    for level in levels:
        group = [r for r in runs if r["dist_level_us"] == level]
        a_runs = [r for r in group if r["fuzzy_on"] == 0]
        b_runs = [r for r in group if r["fuzzy_on"] == 1]
        print(f"\n-- level gangguan {level} us: L1 tetap n={len(a_runs)}, fuzzy n={len(b_runs)}")
        if not a_runs or not b_runs:
            print("   (butuh minimal satu run di tiap metode)")
            continue
        for metric in COMPARE_METRICS:
            a = [r.get(metric, float("nan")) for r in a_runs]
            b = [r.get(metric, float("nan")) for r in b_runs]
            if paired and len(a) == len(b):
                stat, p = wilcoxon_signed_rank(b, a)
            else:
                stat, p = mann_whitney_u(a, b)
            ma, mb = np.nanmean(a), np.nanmean(b)
            effect = 100.0 * (mb - ma) / ma if ma not in (0.0,) and not math.isnan(ma) else float("nan")
            print(f"   {metric:26s} L1={ma:9.3f}±{np.nanstd(a):7.3f}  fuzzy={mb:9.3f}±{np.nanstd(b):7.3f}"
                  f"  efek={effect:+7.1f}%  p={p:.4f}")
        rec = {0: [], 1: []}
        for r in dist_rows:
            if r["level_us"] == level and r["recovered"]:
                rec[r["fuzzy_on"]].append(r["recovery_s"])
        if rec[0] and rec[1]:
            _, p = mann_whitney_u(rec[0], rec[1])
            print(f"   {'recovery_s':26s} L1={np.mean(rec[0]):9.3f}  fuzzy={np.mean(rec[1]):9.3f}  p={p:.4f}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="file LOGnnn.CSV dari SD card")
    parser.add_argument("--band", type=float, default=5.0, help="pita pulih |cte| (m), default 5")
    parser.add_argument("--hold", type=float, default=3.0, help="waktu tahan di pita pulih (s), default 3")
    parser.add_argument("--limit", type=float, default=45.0, help="toleransi CTE (m), default 45")
    parser.add_argument("--out", default=None, help="folder keluaran CSV (opsional)")
    parser.add_argument("--compare", action="store_true", help="uji statistik L1 tetap vs fuzzy")
    parser.add_argument("--paired", action="store_true", help="Wilcoxon berpasangan (jumlah run sama)")
    args = parser.parse_args()

    all_runs: list[dict] = []
    all_dist: list[dict] = []
    for path in args.files:
        log = read_sdlog(path)
        runs, dist_rows = analyze_log(log, args.band, args.hold, args.limit)
        print(f"{os.path.basename(path)}: {log.n_rows} baris, {len(log.events)} event, "
              f"{len(log.snapshots)} snapshot, {log.bad_rows} baris rusak, {len(runs)} run AUTO")
        all_runs += runs
        all_dist += dist_rows

    for r in all_runs:
        print("  run", r["file"], r["run"], " ".join(f"{k}={fmt(v)}" for k, v in r.items()
                                                    if k not in ("file", "run")))
    for d in all_dist:
        print("  gangguan", d["file"], d["run"], d["window"], " ".join(f"{k}={fmt(v)}" for k, v in d.items()
                                                                     if k not in ("file", "run", "window")))
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        write_csv(os.path.join(args.out, "run_metrics.csv"), all_runs)
        write_csv(os.path.join(args.out, "disturbance_windows.csv"), all_dist)
        print("ditulis ke", args.out)
    if args.compare:
        compare_methods(all_runs, all_dist, args.paired)
    return 0


if __name__ == "__main__":
    sys.exit(main())
