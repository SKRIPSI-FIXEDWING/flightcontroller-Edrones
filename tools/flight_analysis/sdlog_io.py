"""Pembaca log penerbangan SD card (format v2, fc::FlightLog).

Format: docs/flight-log-csv.md. Ringkas:
  - baris diawali '#' = komentar/metadata (header, '# EFFECTIVE', '# PARAM',
    '# WP', ...), '#EVENT,<ms>,<jenis>,...' = event, '#ARM_SNAPSHOT,<ms>' /
    '#SNAPSHOT,BOOT,<ms>' = awal blok snapshot setelan;
  - baris pertama tanpa '#' = nama kolom; sesudahnya data 20 Hz.

TANDA cte_m di file: POSITIF = pesawat di KIRI garis wp_prev->wp_next (dilihat
searah garis). Skrip tlog lama memakai positif = kanan. read_sdlog() tidak
mengubah tanda; pakai `log.cte_right_positive()` bila butuh konvensi lama.

Hanya butuh numpy. Pemakaian:
    from sdlog_io import read_sdlog
    log = read_sdlog("LOG007.CSV")
    log.col["cte_m"], log.events, log.snapshots
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

# Kolom bertipe teks; sisanya float.
STRING_COLUMNS = {"mode"}


@dataclass
class Event:
    t_ms: int
    kind: str  # ARM, DISARM, MODE, WP_REACHED, PARAM, GPS_LOST, GPS_OK, DIST_ON, DIST_OFF, ...
    args: list[str]

    @property
    def t_s(self) -> float:
        return self.t_ms / 1000.0


@dataclass
class Snapshot:
    """Setelan yang tercatat: di header file (tag BOOT) atau setiap ARM."""

    tag: str  # "BOOT" atau "ARM"
    t_ms: int
    effective: dict[str, float] = field(default_factory=dict)  # yang benar-benar dipakai
    params: dict[str, float] = field(default_factory=dict)  # isi tabel parameter
    home: tuple[int, int, int] | None = None  # lat_e7, lon_e7, alt_cm
    waypoints: list[dict] = field(default_factory=list)  # seq, lat_e7, lon_e7, alt_cm, radius_m


@dataclass
class SdLog:
    path: str
    meta: dict[str, str]
    columns: list[str]
    col: dict[str, np.ndarray]
    events: list[Event]
    snapshots: list[Snapshot]
    bad_rows: int

    @property
    def n_rows(self) -> int:
        return len(next(iter(self.col.values()))) if self.col else 0

    def time_s(self) -> np.ndarray:
        """Detik sejak baris pertama (dari time_boot_ms)."""
        t = self.col["time_boot_ms"]
        return (t - t[0]) / 1000.0 if len(t) else t

    def cte_right_positive(self) -> np.ndarray:
        """cte_m dengan konvensi skrip tlog lama (positif = kanan garis)."""
        return -self.col["cte_m"]

    def events_of(self, *kinds: str) -> list[Event]:
        return [e for e in self.events if e.kind in kinds]

    def effective_at(self, t_ms: float) -> dict[str, float]:
        """Setelan efektif dari snapshot terakhir yang tercatat sebelum t_ms."""
        best = None
        for snap in self.snapshots:
            if snap.t_ms <= t_ms and snap.effective:
                best = snap
        if best is None and self.snapshots:
            best = self.snapshots[0]
        return dict(best.effective) if best else {}


_KV = re.compile(r"([A-Za-z0-9_]+)=(\S+)")


def _to_float(text: str) -> float:
    try:
        return float(text)
    except ValueError:
        return float("nan")


def read_sdlog(path: str) -> SdLog:
    meta: dict[str, str] = {}
    events: list[Event] = []
    snapshots: list[Snapshot] = []
    current: Snapshot | None = None
    columns: list[str] | None = None
    rows: list[list[str]] = []
    bad_rows = 0

    with open(path, "r", encoding="utf-8", errors="replace", newline="") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("#"):
                if line.startswith("#EVENT,"):
                    parts = line.split(",")
                    try:
                        events.append(Event(int(parts[1]), parts[2], parts[3:]))
                    except (ValueError, IndexError):
                        pass
                elif line.startswith("#ARM_SNAPSHOT,"):
                    try:
                        current = Snapshot("ARM", int(line.split(",")[1]))
                        snapshots.append(current)
                    except (ValueError, IndexError):
                        current = None
                elif line.startswith("#SNAPSHOT,"):
                    parts = line.split(",")
                    try:
                        current = Snapshot(parts[1], int(parts[2]))
                        snapshots.append(current)
                    except (ValueError, IndexError):
                        current = None
                elif line.startswith("# EFFECTIVE "):
                    if current is not None:
                        for key, value in _KV.findall(line):
                            current.effective[key] = _to_float(value)
                elif line.startswith("# PARAM "):
                    if current is not None:
                        for key, value in _KV.findall(line):
                            current.params[key] = _to_float(value)
                elif line.startswith("# HOME,"):
                    parts = line.split(",")
                    if current is not None and len(parts) >= 4:
                        try:
                            current.home = (int(parts[1]), int(parts[2]), int(parts[3]))
                        except ValueError:
                            pass
                elif line.startswith("# WP,"):
                    parts = line.split(",")
                    if current is not None and len(parts) >= 6:
                        try:
                            current.waypoints.append(
                                dict(seq=int(parts[1]), lat_e7=int(parts[2]), lon_e7=int(parts[3]),
                                     alt_cm=int(parts[4]), radius_m=float(parts[5])))
                        except ValueError:
                            pass
                else:
                    # Metadata bebas: '# fw_tag=... build=...', '# file=... boot_ms=...'
                    for key, value in _KV.findall(line):
                        meta.setdefault(key, value)
                continue

            if columns is None:
                columns = line.split(",")
                continue
            fields = line.split(",")
            if len(fields) != len(columns):
                bad_rows += 1  # baris terpotong (daya putus) atau rusak
                continue
            rows.append(fields)

    if columns is None:
        raise ValueError(f"{path}: tidak ada baris nama kolom (bukan log format v2?)")

    col: dict[str, np.ndarray] = {}
    for index, name in enumerate(columns):
        values = [r[index] for r in rows]
        if name in STRING_COLUMNS:
            col[name] = np.array(values, dtype="<U4")
        else:
            col[name] = np.array([_to_float(v) for v in values], dtype=float)

    return SdLog(path=path, meta=meta, columns=columns, col=col, events=events,
                 snapshots=snapshots, bad_rows=bad_rows)

