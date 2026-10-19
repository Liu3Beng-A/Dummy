"""Collect the no-motion F405 CBENCH diagnostic; never flash or send motion.

Without --port this script only shows help or parses a saved capture. Opening a
USB port requires --port; running computation additionally requires --run.
Examples (COM numbers are examples, not an instruction to connect now):
  python tools/tests/run_cartesian_benchmark.py --port COM9
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --op 2 --work 1
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --op 3 --work 5 --profile
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --profile --oled-mode skip-render --work 5
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --profile --oled-mode rate-10hz --work 5
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --profile --oled-mode split-same --work 5
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --profile --oled-mode split-low --work 5
  python tools/tests/run_cartesian_benchmark.py --parse PATH/raw.jsonl
  python tools/tests/run_cartesian_benchmark.py --port COM9 --cancel

This measures the diagnostic kernel, not a complete online MoveL follower.
Optional profiling reports IRQ-inclusive task residence, not exclusive CPU time.
Host parsing/tests are not hardware evidence. A timed-out job is not silently
cancelled. Use a separate explicit --cancel invocation if cancellation is wanted.
Exit 0 means capture/parsing completed (or IDLE screening), not that timing or
the benchmark job passed; inspect summary.json job_success and phase.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import sys
import time
from typing import Any


REPO = Path(__file__).resolve().parents[2]
TERMINAL = frozenset({"IDLE", "DONE", "CANCELLED", "ABORTED", "BUDGET", "CLOCK_ERROR"})
QUERIES = ("#FW_VERSION 0", "#KIN_CONFIG", "#TOOL", "#MOTION_STATUS", "#STACK", "#CBENCH")
STATUS_RE = re.compile(r"CBENCH v=1 id=(\d+) phase=(IDLE|PENDING|RUNNING|DONE|CANCELLED|ABORTED|BUDGET|CLOCK_ERROR)")
DATA_RE = re.compile(r"CBENCH DATA id=(\d+) i=(\d+) cycles=(\d+) ticks=(\d+) work=(\d+) flag=(\d+)")
RESULT_RE = re.compile(r"CBENCH RESULT id=(\d+) i=(\d+) fk=(\d+) ik=(\d+) valid=(\d+) selected=(\d+) failed=(\d+) checksum=(\d+)")
# Compact wire records fit the existing 64-byte Respond buffer even at uint32
# maximum. The keyed spellings above remain accepted for saved synthetic data.
DATA_CSV_RE = re.compile(r"CBENCH DATA (\d+),(\d+),(\d+),(\d+),(\d+),(\d+)")
RESULT_CSV_RE = re.compile(r"CBENCH RESULT (\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)")
CONFIG_RE = re.compile(r"CBENCH config op=(\d+) fixture=(\d+) work=(\d+) samples=(\d+)")
CLOCK_RE = re.compile(r"CBENCH clock_hz=(\d+) budget_us=(\d+) n=(\d+) period_ms=(\d+)")
CLOCK_SHORT_RE = re.compile(r"CBENCH clock_hz=(\d+) budget_us=(\d+) n=(\d+)")
PERIOD_RE = re.compile(r"CBENCH period_ms=(\d+)")
PROBE_RE = re.compile(r"CBENCH probe_cycles=(\d+) probe_ticks=(\d+)")
SETUP_RE = re.compile(r"CBENCH setup_cycles=(\d+)")
CONTRACT_RE = re.compile(r"CBENCH no_motion=(\d+) scope=(\S+)")
DWT_RE = re.compile(r"CBENCH dwt_ctrl=(\d+) demcr=(\d+)")
UINT32_MAX = (1 << 32) - 1
PROFILE_TASKS = ("command", "realtime", "oled", "rgb", "usb", "other")
PROFILE_TASKS_V2 = ("command", "realtime", "oled_worker", "rgb", "usb", "display", "other")
PROFILE_HEADER_RE = re.compile(r"CBPROF v=([12]) id=([0-9]+) mode=([01]) n=([0-9]+)")
PROFILE_CAPS_RE = re.compile(r"CBPROF caps samples=([0-9]+) work=([0-9]+) tasks=([0-9]+)")
PROFILE_MAP_RE = re.compile(r"CBPROF mapmask=([0-9]+) irq_included=([0-9]+)")
PROFILE_END_RE = re.compile(r"CBPROF end id=([0-9]+)")
PROFILE_RECORD_RE = re.compile(r"CBPROF ([SHDWTE]) ([0-9]+(?:,[0-9]+){3,4})")
OLED_STAGES = ("imu", "service", "draw", "send", "other")
OLED_MODES = {"normal": 0, "skip-render": 1, "rate-10hz": 2, "split-same": 3, "split-low": 4}
OLED_HEADER_RE = re.compile(r"CBOLED v=([123]) id=([0-9]+) enabled=([01]) skip=([01])(?: mode=([0-4]))?")
OLED_CAPS_RE = re.compile(r"CBOLED caps samples=([0-9]+) stages=([0-9]+)(?: lanes=([0-9]+))?")
OLED_RATE_RE = re.compile(r"CBOLED rate period_ticks=([0-9]+) tick_hz=([0-9]+)")
OLED_COUNTS_RE = re.compile(r"CBOLED ([FLMRGDJNUHSACK]) ([0-9]+(?:,[0-9]+){2,3})")
OLED_END_RE = re.compile(r"CBOLED end id=([0-9]+)")
OLED_RECORD_RE = re.compile(r"CBOLED ([PWQVE]) ([0-9]+(?:,[0-9]+){2,3})")


def uint32_values(values: Any) -> tuple[int, ...]:
    values = tuple(values)
    if any(isinstance(value, str) and (len(value) > 10 or not value.isascii() or not value.isdigit()) for value in values):
        raise ProtocolError("Profile number is not a bounded unsigned decimal")
    numbers = tuple(map(int, values))
    if any(value < 0 or value > UINT32_MAX for value in numbers):
        raise ProtocolError("Profile value exceeds uint32")
    return numbers


class ProfileMetadata:
    def __init__(self, expected_id: int):
        self.expected_id = expected_id
        self.seen: set[str] = set()
        self.values: dict[str, int] = {}

    def feed(self, line: str) -> bool:
        for key, pattern in (("header", PROFILE_HEADER_RE), ("caps", PROFILE_CAPS_RE),
                             ("map", PROFILE_MAP_RE), ("end", PROFILE_END_RE)):
            match = pattern.fullmatch(line)
            if not match:
                continue
            numbers = uint32_values(match.groups())
            if key in self.seen:
                raise ProtocolError(f"Duplicate profile metadata {key}")
            if key == "header":
                version, job_id, mode, count = numbers
                if job_id != self.expected_id or count > 128:
                    raise ProtocolError("Profile metadata id/count mismatch")
                self.values.update(version=version, id=job_id, mode=mode, n=count)
            elif "header" not in self.seen:
                raise ProtocolError("Profile metadata arrived before id")
            elif key == "caps":
                tasks = 6 if self.values["version"] == 1 else 7
                if numbers != (128, 5, tasks):
                    raise ProtocolError("Unsupported profile capacities")
                self.values.update(samples=128, work=5, tasks=tasks)
            elif key == "map":
                if numbers[0] > (31 if self.values["version"] == 1 else 63):
                    raise ProtocolError("Profile mapmask exceeds mapped task bits")
                if numbers[1] != 1:
                    raise ProtocolError("Profile must disclose IRQ-inclusive residence")
                self.values.update(mapmask=numbers[0], irq_included=1)
            elif numbers != (self.expected_id,):
                raise ProtocolError("Profile end id mismatch")
            self.seen.add(key)
            if key == "end":
                if self.seen != {"header", "caps", "map", "end"}:
                    raise ProtocolError("Incomplete profile metadata at end")
                return True
            return False
        raise ProtocolError(f"Malformed profile metadata: {line}")


class ProfileRecords:
    """Strict per-sample accounting, reused by live boundaries and saved logs."""
    def __init__(self, expected_id: int, base_data: dict[int, dict[str, int]], hz: int, version: int = 1):
        if not base_data or hz <= 0:
            raise ProtocolError("PREAD requires base DATA and a known clock")
        uint32_values((expected_id, hz))
        for index, data in base_data.items():
            uint32_values((index, data["work"], data["cycles"]))
            if index >= 128 or data["work"] > 5:
                raise ProtocolError("PREAD base DATA exceeds profile capacities")
        if version not in (1, 2):
            raise ProtocolError("Unsupported profile record version")
        self.expected_id, self.base_data, self.hz = expected_id, base_data, hz
        self.version, self.task_count = version, 6 if version == 1 else 7
        self.records: dict[int, dict[str, Any]] = {}

    def feed(self, line: str) -> bool:
        match = PROFILE_RECORD_RE.fullmatch(line)
        if not match:
            raise ProtocolError(f"Malformed profile record: {line}")
        kind = match[1]
        values = uint32_values(match[2].split(","))
        expected_length = 5 if kind in "SHD" else 4
        if len(values) != expected_length:
            raise ProtocolError(f"Wrong profile {kind} field count")
        job_id, index, *payload = values
        if job_id != self.expected_id or index not in self.base_data:
            raise ProtocolError("PREAD id/index differs from request or base DATA")
        base = self.base_data[index]
        actual = base["work"]
        if not 0 <= actual <= 5:
            raise ProtocolError("Profile base work exceeds five slots")
        sample = self.records.setdefault(index, {"w": {}, "t": {}, "ended": False})
        if sample["ended"]:
            raise ProtocolError("Profile data after sample end")
        if kind in "WT":
            slot, cycles = payload
            limit = actual if kind == "W" else self.task_count
            bucket = sample[kind.lower()]
            if slot >= limit or slot in bucket:
                raise ProtocolError("Duplicate or out-of-range profile work/task slot")
            bucket[slot] = cycles
        else:
            if kind.lower() in sample:
                raise ProtocolError(f"Duplicate profile {kind} record")
            sample[kind.lower()] = payload
        if kind != "E":
            return False
        if set(sample) != {"w", "t", "ended", "s", "h", "d", "e"}:
            raise ProtocolError("Profile sample missing S/H/D records at E")
        if set(sample["w"]) != set(range(actual)) or set(sample["t"]) != set(range(self.task_count)):
            raise ProtocolError("Profile sample missing work/task slots at E")
        if payload[0] != actual:
            raise ProtocolError("Profile actualwork differs from base DATA")
        plan, start, end = sample["s"]
        start_hal, end_hal, late = sample["h"]
        start_dwt, end_dwt, window = sample["d"]
        if late != ((start - plan) & UINT32_MAX):
            raise ProtocolError("Profile lateness arithmetic mismatch")
        if ((end - start) & UINT32_MAX) > 10000 or ((end_hal - start_hal) & UINT32_MAX) > 10000 or late > 10000:
            raise ProtocolError("Profile time window exceeds ten seconds")
        if ((end_dwt - start_dwt) & UINT32_MAX) != window:
            raise ProtocolError("Profile DWT window arithmetic mismatch")
        if window * 1000 > self.hz * 10000 or base["cycles"] > window:
            raise ProtocolError("Profile kernel/window duration is inconsistent")
        if sum(sample["t"].values()) != window:
            raise ProtocolError("Profile task residence does not conserve window cycles")
        previous = 0
        for slot in range(actual):
            cycles = sample["w"][slot]
            if cycles < previous or cycles > base["cycles"] or cycles > window:
                raise ProtocolError("Profile cumulative work cycles are invalid")
            previous = cycles
        sample["ended"] = True
        return True


class OledMetadata:
    """Whole-job frame counters; they are not kernel-window stage durations."""
    def __init__(self, expected_id: int):
        self.expected_id = expected_id
        self.seen: set[str] = set()
        self.values: dict[str, int] = {}

    def validate(self) -> None:
        v = self.values
        if v["ended"] > v["begun"] or v["skipped"] > v["ended"] or v["late_frames"] > v["begun"] or v["overrun_frames"] > v["ended"]:
            raise ProtocolError("OLED frame counters are inconsistent")
        if v["version"] == 1:
            if not v["skip"] and v["skipped"]:
                raise ProtocolError("Normal OLED mode reported skipped render frames")
            return
        if v["tick_hz"] != 1000 or v["period_ticks"] != (100 if v["enabled"] and v["mode"] >= 2 else 0):
            raise ProtocolError("Unsupported OLED rate period/tick frequency")
        if v["skip"] != int(v["enabled"] and v["mode"] == 1):
            raise ProtocolError("OLED skip flag differs from mode")
        if not v["enabled"] and v["mode"] != 0:
            raise ProtocolError("Disabled OLED metadata must report normal mode")
        if v["begun"] - v["ended"] > 1:
            raise ProtocolError("OLED v2 has more than one incomplete frame")
        if v["render_begun"] > v["begun"] or v["rate_limited"] > v["ended"]:
            raise ProtocolError("OLED render/rate counters exceed frame counters")
        if v["render_begun"] < 2:
            if v["min_render_gap_ticks"] or v["max_render_gap_ticks"]:
                raise ProtocolError("OLED gap requires two render opportunities")
        elif not 0 <= v["min_render_gap_ticks"] <= v["max_render_gap_ticks"]:
            raise ProtocolError("OLED render gap ordering is inconsistent")
        if not v["enabled"] and any(v[name] for name in ("begun", "ended", "skipped", "late_frames", "max_late_ticks", "overrun_frames", "max_frame_cycles", "render_begun", "rate_limited", "min_render_gap_ticks", "max_render_gap_ticks")):
            raise ProtocolError("Disabled OLED accounting reported frame counters")
        if v["version"] == 2 and v["mode"] > 2:
            raise ProtocolError("OLED v2 does not support split modes")
        if v["mode"] == 0 and (v["skipped"] or v["rate_limited"] or v["render_begun"] != v["begun"]):
            raise ProtocolError("Normal OLED mode has rate/skip counters")
        if v["mode"] == 1 and (v["render_begun"] or v["rate_limited"] or v["skipped"] != v["ended"]):
            raise ProtocolError("Skip OLED mode has inconsistent render counters")
        if v["mode"] == 2 and (v["skipped"] != v["rate_limited"] or not v["ended"] <= v["render_begun"] + v["rate_limited"] <= v["begun"]):
            raise ProtocolError("Rate OLED decisions do not cover ended frames")
        if v["version"] != 3:
            return
        d = {key: v["display_" + key] for key in ("begun", "ended", "skipped", "late_frames", "max_late_ticks", "overrun_frames", "max_frame_cycles", "render_begun", "rate_limited", "min_render_gap_ticks", "max_render_gap_ticks")}
        if (d["ended"] > d["begun"] or d["begun"] - d["ended"] > 1 or d["skipped"] > d["ended"]
                or d["late_frames"] > d["begun"] or d["overrun_frames"] > d["ended"]
                or d["render_begun"] > d["begun"] or d["rate_limited"] > d["ended"]):
            raise ProtocolError("OLED display frame counters are inconsistent")
        if d["render_begun"] < 2:
            if d["min_render_gap_ticks"] or d["max_render_gap_ticks"]:
                raise ProtocolError("OLED display gap requires two render opportunities")
        elif d["min_render_gap_ticks"] > d["max_render_gap_ticks"]:
            raise ProtocolError("OLED display render gap ordering is inconsistent")
        if v["mode"] < 3 and any(d.values()):
            raise ProtocolError("Non-split OLED reported display frame counters")
        if v["mode"] >= 3:
            if v["skipped"] != v["ended"] or v["render_begun"] or v["rate_limited"]:
                raise ProtocolError("Split OLED worker must omit rendering")
            if (d["skipped"] != d["rate_limited"]
                    or not d["ended"] <= d["render_begun"] + d["rate_limited"] <= d["begun"]):
                raise ProtocolError("Split OLED display rate decisions do not cover ended frames")
        if v["restored"] not in (0, 1) or v["display_stack_size"] != 2048 or v["display_stack_free"] > 2048:
            raise ProtocolError("OLED runtime/stack metadata is inconsistent")

    def feed(self, line: str) -> bool:
        if match := OLED_HEADER_RE.fullmatch(line):
            key = "header"
            version, job_id, enabled, skip = uint32_values(match.groups()[:4])
            if (version == 1) != (match[5] is None):
                raise ProtocolError("OLED version/mode schema mismatch")
            mode = int(match[5]) if version >= 2 else skip
            if version == 2 and mode > 2:
                raise ProtocolError("OLED v2 mode exceeds its schema")
            if job_id != self.expected_id or (not enabled and skip):
                raise ProtocolError("OLED metadata id/mode mismatch")
            values = {"version": version, "id": job_id, "enabled": enabled, "skip": skip, "mode": mode}
        elif match := OLED_CAPS_RE.fullmatch(line):
            key = "caps"
            expected = (128, 5, 2) if self.values.get("version") == 3 else (128, 5)
            if uint32_values(group for group in match.groups() if group is not None) != expected:
                raise ProtocolError("Unsupported OLED capacities")
            values = {"samples": 128, "stages": 5}
            if self.values.get("version") == 3:
                values["lanes"] = 2
        elif match := OLED_RATE_RE.fullmatch(line):
            key = "rate"
            period, tick_hz = uint32_values(match.groups())
            values = {"period_ticks": period, "tick_hz": tick_hz}
        elif match := OLED_COUNTS_RE.fullmatch(line):
            key = match[1]
            numbers = uint32_values(match[2].split(","))
            names = {"F": ("begun", "ended", "skipped"),
                     "L": ("late_frames", "max_late_ticks"),
                     "M": ("overrun_frames", "max_frame_cycles"),
                     "R": ("render_begun", "rate_limited"),
                     "G": ("min_render_gap_ticks", "max_render_gap_ticks"),
                     "D": ("display_begun", "display_ended", "display_skipped"),
                     "J": ("display_late_frames", "display_max_late_ticks"),
                     "N": ("display_overrun_frames", "display_max_frame_cycles"),
                     "U": ("display_render_begun", "display_rate_limited"),
                     "H": ("display_min_render_gap_ticks", "display_max_render_gap_ticks"),
                     "S": ("worker_max_loop_gap_ticks", "display_max_loop_gap_ticks"),
                     "A": ("max_snapshot_age_ticks", "overwritten", "max_send_gap_ticks"),
                     "C": ("runtime_flags", "restored", "applied_priority"),
                     "K": ("display_stack_free", "display_stack_size")}[key]
            if len(numbers) != len(names) + 1 or numbers[0] != self.expected_id:
                raise ProtocolError("OLED frame metadata id/field count mismatch")
            values = dict(zip(names, numbers[1:]))
        elif match := OLED_END_RE.fullmatch(line):
            key, values = "end", {}
            if uint32_values(match.groups()) != (self.expected_id,):
                raise ProtocolError("OLED end id mismatch")
        else:
            raise ProtocolError(f"Malformed OLED metadata: {line}")
        if key in self.seen:
            raise ProtocolError(f"Duplicate OLED metadata {key}")
        if key != "header" and "header" not in self.seen:
            raise ProtocolError("OLED metadata arrived before id")
        order = (("header", "caps", "rate", "F", "L", "M", "R", "G", "D", "J", "N", "U", "H", "S", "A", "C", "K", "end")
                 if self.values.get("version") == 3 else ("header", "caps", "rate", "F", "L", "M", "R", "G", "end"))
        if self.values.get("version", values.get("version", 0)) >= 2 and (len(self.seen) >= len(order) or key != order[len(self.seen)]):
            raise ProtocolError("OLED metadata arrived out of order")
        if self.values.get("version") == 1 and key not in ("header", "caps", "F", "L", "M", "end"):
            raise ProtocolError("OLED v1 has newer-only metadata")
        self.seen.add(key)
        self.values.update(values)
        if key == "end":
            required = set(order) if self.values["version"] >= 2 else {"header", "caps", "F", "L", "M", "end"}
            if self.seen != required:
                raise ProtocolError("Incomplete OLED metadata at end")
            self.validate()
            return True
        return False


class OledRecords:
    """Disjoint P residence and W elapsed partitions of a saved profile window."""
    def __init__(self, expected_id: int, profile_records: dict[int, dict[str, Any]], version: int = 1):
        if version not in (1, 2, 3):
            raise ProtocolError("Unsupported OLED record version")
        if not profile_records or any(not record.get("ended") for record in profile_records.values()):
            raise ProtocolError("OREAD requires complete profile records")
        uint32_values((expected_id,))
        for index, record in profile_records.items():
            if index >= 128 or set(record.get("t", {})) != set(range(7 if version == 3 else 6)) or len(record.get("d", ())) != 3:
                raise ProtocolError("OREAD requires bounded complete profile task/window data")
            uint32_values((index, *record["t"].values(), *record["d"]))
        self.expected_id, self.profile_records = expected_id, profile_records
        self.version = version
        self.records: dict[int, dict[str, Any]] = {}

    def feed(self, line: str) -> bool:
        match = OLED_RECORD_RE.fullmatch(line)
        if not match:
            raise ProtocolError(f"Malformed OLED record: {line}")
        kind = match[1]
        values = uint32_values(match[2].split(","))
        if len(values) != (3 if kind == "E" else 4):
            raise ProtocolError(f"Wrong OLED {kind} field count")
        job_id, index, *payload = values
        if job_id != self.expected_id or index not in self.profile_records:
            raise ProtocolError("OREAD id/index differs from request or profile")
        if self.version < 3 and kind in "QV":
            raise ProtocolError("OLED single-lane version has display records")
        record = self.records.setdefault(index, {**{key: {} for key in ("p", "w", "q", "v") if self.version == 3 or key in ("p", "w")}, "ended": False})
        if record["ended"]:
            raise ProtocolError("OLED data after sample end")
        if self.version == 3:
            sequence = [(name, stage) for stage in range(5) for name in "PWQV"] + [("E", None)]
            position = sum(len(record[name]) for name in ("p", "w", "q", "v"))
            expected_kind, expected_stage = sequence[position]
            if kind != expected_kind or (kind != "E" and payload[0] != expected_stage):
                raise ProtocolError("OLED v3 stage records arrived out of order")
        if kind != "E":
            stage, cycles = payload
            bucket = record[kind.lower()]
            if stage >= 5 or stage in bucket:
                raise ProtocolError("Duplicate or out-of-range OLED stage")
            bucket[stage] = cycles
            return False
        if any(set(record[key]) != set(range(5)) for key in (("p", "w", "q", "v") if self.version == 3 else ("p", "w"))):
            raise ProtocolError("OLED sample missing stage slots at E")
        profile = self.profile_records[index]
        if sum(record["p"].values()) != profile["t"][2]:
            raise ProtocolError("OLED residence does not conserve profile OLED cycles")
        if sum(record["w"].values()) != profile["d"][2]:
            raise ProtocolError("OLED elapsed stages do not conserve profile window cycles")
        if any(record["p"][stage] > record["w"][stage] for stage in range(5)):
            raise ProtocolError("OLED stage residence exceeds elapsed stage window")
        if self.version == 3:
            if sum(record["q"].values()) != profile["t"][5]:
                raise ProtocolError("OLED display residence does not conserve profile display cycles")
            if sum(record["v"].values()) != profile["d"][2]:
                raise ProtocolError("OLED display elapsed stages do not conserve profile window cycles")
            if any(record["q"][stage] > record["v"][stage] for stage in range(5)):
                raise ProtocolError("OLED display residence exceeds elapsed stage window")
        record["flags"], record["ended"] = payload[0], True
        return True


class ProtocolError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def command_allowed(command: str) -> bool:
    """A closed allowlist; no arbitrary passthrough or newline injection."""
    if not command.isascii():
        return False
    if command in QUERIES or command in ("#CBENCH CANCEL", "#CBENCH PROFILE", "#CBENCH OLED"):
        return True
    match = re.fullmatch(r"#CBENCH RUN (\d+),(\d+),(\d+),(\d+),(\d+)(?:,(1)(?:,([0-4]))?)?", command)
    if match:
        op, fixture, work, samples, budget = map(int, match.groups()[:5])
        return (0 <= op <= 5 and 0 <= fixture <= 2 and 1 <= work <= (5 if match[6] else 192)
                and 1 <= samples <= 128 and 100 <= budget <= 20000)
    match = re.fullmatch(r"#CBENCH ([PO]?READ) (\d+),(\d+)", command)
    if match:
        offset, count = map(int, match.groups()[1:])
        return 0 <= offset < 128 and 1 <= count <= (8 if match[1] == "READ" else 4) and offset + count <= 128
    return False


class ResponseBoundary:
    """Command-specific schemas from firmware, which has NO generic end.

    A known final record must arrive after all mandatory records. Silence,
    an arbitrary line, or a standalone end never completes a response.
    """
    def __init__(self, command: str, expected_id: int | None = None,
                 base_data: dict[int, dict[str, int]] | None = None, clock_hz: int = 0,
                 profile_records: dict[int, dict[str, Any]] | None = None,
                 profile_version: int = 1, oled_version: int = 1):
        self.command = command
        self.expected_id = expected_id
        self.request: int | None = None
        self.version_fields: set[str] = set()
        self.early_version: list[str] = []
        self.run_values: tuple[int, ...] | None = None
        self.seen: set[Any] = set()
        self.error: str | None = None
        self.response_id: int | None = None
        self.rules: list[tuple[str, re.Pattern[str], bool]] = []
        self.read_range: set[int] = set()
        self.read_final: int | None = None
        self.required: set[Any] = set()
        self.profile_metadata: ProfileMetadata | None = None
        self.profile_records: ProfileRecords | None = None
        self.oled_metadata: OledMetadata | None = None
        self.oled_records: OledRecords | None = None
        if not command_allowed(command):
            self._fail(f"Unsupported response boundary: {command!r}")
        if command == "#FW_VERSION 0":
            self.prefixes = ("FW_VERSION [0] ", "ok FW_VERSION [0] ")
            self.required = {"pending", "v", "epoch", "group", "h0", "h1", "end"}
        elif command == "#CBENCH OLED":
            if expected_id is None:
                self._fail("OLED requires a known benchmark id")
            self.prefixes = ("CBOLED ",)
            self.oled_metadata = OledMetadata(expected_id)
        elif command.startswith("#CBENCH OREAD "):
            if expected_id is None:
                self._fail("OREAD requires a known benchmark id")
            offset, count = map(int, command.split(" ", 2)[2].split(","))
            self.read_range = set(range(offset, offset + count))
            self.read_final = offset + count - 1
            if profile_records is None or not self.read_range <= profile_records.keys():
                self._fail("OREAD requires known profile records for every requested index")
            self.oled_records = OledRecords(expected_id, {i: profile_records[i] for i in self.read_range}, oled_version)
            self.required = set(self.read_range)
            self.prefixes = ("CBOLED ",)
        elif command == "#CBENCH PROFILE":
            if expected_id is None:
                self._fail("PROFILE requires a known benchmark id")
            self.prefixes = ("CBPROF ",)
            self.profile_metadata = ProfileMetadata(expected_id)
        elif command.startswith("#CBENCH PREAD "):
            if expected_id is None:
                self._fail("PREAD requires a known benchmark id")
            offset, count = map(int, command.split(" ", 2)[2].split(","))
            self.read_range = set(range(offset, offset + count))
            self.read_final = offset + count - 1
            if base_data is None or not self.read_range <= base_data.keys():
                self._fail("PREAD requires known base DATA for every requested index")
            self.profile_records = ProfileRecords(expected_id, {i: base_data[i] for i in self.read_range}, clock_hz, profile_version)
            self.required = set(self.read_range)
            self.prefixes = ("CBPROF ",)
        elif command.startswith("#CBENCH READ "):
            if expected_id is None:
                self._fail("READ requires a known benchmark id")
            offset, count = map(int, command.split(" ", 2)[2].split(","))
            self.read_range = set(range(offset, offset + count))
            self.read_final = offset + count - 1
            self.required = {(kind, i) for kind in ("data", "result") for i in self.read_range}
            self.prefixes = ("CBENCH ",)
        elif command.startswith("#CBENCH"):
            self.prefixes = ("CBENCH ",)
            self.required = {"status", "config", "clock", "period", "dwt", "probe", "setup", "contract"}
            if command.startswith("#CBENCH RUN "):
                self.run_values = tuple(map(int, command.split(" ", 2)[2].split(",")))
        else:
            schemas = {
                "#KIN_CONFIG": ("KIN ", [
                    ("v", r"KIN v=2 xyz=mm abc=deg joints=deg rail=mm", False),
                    ("base", r"KIN base=\S+ db=\S+ arm=\S+ forearm=\S+", False),
                    ("elbow", r"KIN elbow=\S+ wrist=\S+ tool_default=\S+", False),
                    ("rail", r"KIN rail_axis=\S+ rail_min=\S+ rail_max=\S+", False),
                    ("tcp", r"KIN TCP=@T x,y,z,a,b,c,speed,rail_auto", True),
                ]),
                "#TOOL": ("TOOL ", [
                    ("header", r"TOOL v=1 rev=\d+ dirty=\d+ storage=\d+", False),
                    ("xyz", r"TOOL XYZ \S+ \S+ \S+", False),
                    ("abc", r"TOOL ABC \S+ \S+ \S+", True),
                ]),
                "#STACK": ("ok STACK ", [
                    ("usb", r"ok STACK usb=\d+ uart=\d+ heap=\d+", False),
                    ("cmd", r"ok STACK cmd=\d+ size=\d+ heap_min=\d+", False),
                    ("rt", r"ok STACK rt=\d+ oled=\d+ rgb=\d+", True),
                ]),
                "#MOTION_STATUS": ("MOTION ", [
                    ("header", r"MOTION id=\d+ kind=\S+ phase=\d+ mode=\d+ ms=\d+", False),
                    ("ready", r"MOTION ready=[0-9A-Fa-f]+ confirmed=[0-9A-Fa-f]+ progress=[0-9A-Fa-f]+", False),
                    ("send", r"MOTION send=\d+ query=\d+ reject=\d+", False),
                    ("plan", r"MOTION plan_ms=\S+ moving=[0-9A-Fa-f]+ txok=[0-9A-Fa-f]+ pending=[0-9A-Fa-f]+", False),
                    ("spread", r"MOTION spread tx=-?\d+ settled_obs=-?\d+", False),
                ]),
            }
            prefix, rules = schemas[command]
            if command == "#MOTION_STATUS":
                for node in range(1, 8):
                    rules += [
                        (f"goal{node}", rf"MOTION J{node} goal=\S+ real=\S+ f=[0-9A-Fa-f]+ age=\d+", False),
                        (f"tx{node}", rf"MOTION J{node} tx=-?\d+ ready=-?\d+ try=\d+ fail=\d+", False),
                        (f"plan{node}", rf"MOTION J{node} plan=-?\d+ pos=-?\d+ settled=-?\d+", False),
                        (f"v{node}", rf"MOTION J{node} v=\S+ a=\S+ d=\S+", node == 7),
                    ]
            self.prefixes = (prefix,)
            self.rules = [(key, re.compile(pattern), terminal) for key, pattern, terminal in rules]
            self.required = {key for key, _, _ in rules}

    def _fail(self, message: str) -> None:
        self.error = message
        raise ProtocolError(message)

    def _accept(self, keys: set[Any], terminal: bool = False) -> bool:
        if self.seen & keys:
            self._fail(f"Duplicate {self.command} record: {self.seen & keys}")
        self.seen.update(keys)
        if terminal:
            missing = self.required - self.seen
            if missing:
                self._fail(f"Incomplete {self.command} response at final record: {missing}")
            return True
        return False

    def related(self, line: str) -> bool:
        return line.startswith(self.prefixes)

    def trailing(self, line: str) -> None:
        if line.startswith("error ") or self.related(line):
            self._fail(f"Unexpected trailing response after {self.command}: {line}")

    def feed(self, line: str) -> bool:
        if line.startswith("error "):
            self.error = line
            self._fail(line)
        if self.oled_metadata is not None:
            return self.oled_metadata.feed(line) if self.related(line) else False
        if self.oled_records is not None:
            if self.related(line) and self.oled_records.feed(line):
                index = int(line.split(" ", 2)[2].split(",")[1])
                return self._accept({index}, index == self.read_final)
            return False
        if self.profile_metadata is not None:
            if self.related(line):
                return self.profile_metadata.feed(line)
            return False
        if self.profile_records is not None:
            if self.related(line):
                ended = self.profile_records.feed(line)
                if ended:
                    index = int(line.split(" ", 2)[2].split(",")[1])
                    return self._accept({index}, index == self.read_final)
            return False
        if self.command == "#FW_VERSION 0":
            match = re.fullmatch(r"ok FW_VERSION \[0\] req=(\d+) pending", line)
            if match:
                if self.request is not None and self.request != int(match[1]):
                    self._fail("FW_VERSION request changed within one response")
                self.request = int(match[1])
                self._accept({"pending"})
                # Firmware publishes DIAG_READY before printing pending. The
                # same-priority output task can deliver fields first.
                earlier, self.early_version = self.early_version, []
                for record in earlier:
                    self.feed(record)
                return self.required <= self.seen
            match = re.fullmatch(r"FW_VERSION \[0\] req=(\d+) (.+)", line)
            if match and self.request is None:
                if len(self.early_version) >= 16:
                    self._fail("Too many FW_VERSION fields before pending context")
                self.early_version.append(line)
                return False
            if match and self.request is not None and int(match[1]) == self.request:
                payload = match[2]
                patterns = {
                    "v": r"v=\d{8}\.\d{2} type=CORE", "epoch": r"epoch=\d+",
                    "group": r"group=[0-9A-Fa-f]{16}", "h0": r"h0=[0-9A-Fa-f]{32}",
                    "h1": r"h1=[0-9A-Fa-f]{32}", "end": r"end",
                }
                for field, pattern in patterns.items():
                    if re.fullmatch(pattern, payload):
                        self.version_fields.add(field)
                        return self._accept({field}, field == "end")
                self._fail(f"Malformed FW_VERSION field: {line}")
            if self.related(line) and match is None:
                self._fail(f"Malformed FW_VERSION response: {line}")
            return False
        if self.read_range:
            for regex, kind in ((DATA_CSV_RE, "data"), (RESULT_CSV_RE, "result"),
                                (DATA_RE, "data"), (RESULT_RE, "result")):
                match = regex.fullmatch(line)
                if match:
                    job_id, index = int(match[1]), int(match[2])
                    if job_id != self.expected_id or index not in self.read_range:
                        self._fail(f"READ id/index differs from request: {line}")
                    return self._accept({(kind, index)}, kind == "result" and index == self.read_final)
        elif self.command.startswith("#CBENCH"):
            match = STATUS_RE.fullmatch(line)
            if match:
                self.response_id = int(match[1])
                if self.expected_id is not None:
                    if self.run_values is None and self.response_id != self.expected_id:
                        self._fail("CBENCH response changed the expected job id")
                    if self.run_values is not None and self.response_id == self.expected_id:
                        self._fail("CBENCH RUN did not return a new job id")
                return self._accept({"status"})
            for regex, keys in (
                (CONFIG_RE, {"config"}), (CLOCK_SHORT_RE, {"clock"}),
                (CLOCK_RE, {"clock", "period"}), (PERIOD_RE, {"period"}),
                (DWT_RE, {"dwt"}), (PROBE_RE, {"probe"}), (SETUP_RE, {"setup"}),
                (CONTRACT_RE, {"contract"}),
            ):
                match = regex.fullmatch(line)
                if match:
                    if self.response_id is None:
                        self._fail("CBENCH metadata arrived before status/id")
                    if keys == {"contract"} and match.groups() != ("1", "components"):
                        self._fail(f"Unexpected CBENCH contract: {line}")
                    if self.run_values is not None:
                        if keys == {"config"} and tuple(map(int, match.groups())) != self.run_values[:4]:
                            self._fail("CBENCH RUN configuration differs from request")
                        if "clock" in keys and int(match[2]) != self.run_values[4]:
                            self._fail("CBENCH RUN budget differs from request")
                    return self._accept(keys, keys == {"contract"})
        else:
            for key, pattern, terminal in self.rules:
                if pattern.fullmatch(line):
                    return self._accept({key}, terminal)
            if self.command == "#MOTION_STATUS" and (
                re.fullmatch(r"MOTION CAN alst=\d+ terr=\d+ busy=\d+", line)
                or re.fullmatch(r"MOTION cancelled_drop=\d+", line)
            ):
                return False
        if self.related(line):
            self._fail(f"Malformed {self.command} response: {line}")
        return False


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def timing_summary(values: list[float]) -> dict[str, Any]:
    return {"count": len(values), "min_us": min(values) if values else None,
            "p50_us": percentile(values, .50), "p95_us": percentile(values, .95),
            "p99_us": percentile(values, .99), "max_us": max(values) if values else None,
            "over_20000us": sum(v > 20000 for v in values),
            "percentile_method": "linear interpolation of ordered observations; not a population-tail guarantee"}


class CaptureParser:
    """Retain partial jobs; never turn missing or mismatched records into passes."""
    def __init__(self):
        self.jobs: dict[int, dict[str, Any]] = {}
        self.current_id: int | None = None
        self.errors: list[str] = []
        self.auxiliary_errors: list[str] = []

    def job(self, job_id: int) -> dict[str, Any]:
        return self.jobs.setdefault(job_id, {"id": job_id, "phase": None, "config": {},
                                            "clock": {}, "dwt": {}, "probe": {}, "contract": {}, "data": {}, "results": {},
                                            "errors": [], "phases": [], "profile_meta": {},
                                            "profile_pending_meta": None, "profile_records": None,
                                            "profile_expected": False, "profile_errors": [],
                                            "oled_meta": {}, "oled_pending_meta": None, "oled_records": None,
                                            "oled_expected": False, "oled_request_mode": None, "oled_errors": []})

    def _oled(self, line: str) -> None:
        if self.current_id is None:
            self.auxiliary_errors.append(f"Orphan OLED record: {line}")
            return
        job = self.job(self.current_id)
        try:
            if OLED_HEADER_RE.fullmatch(line):
                if job["oled_pending_meta"] is not None:
                    raise ProtocolError("Incomplete previous OLED metadata block")
                job["oled_pending_meta"] = OledMetadata(self.current_id)
            pending = job["oled_pending_meta"]
            if pending is not None:
                if pending.feed(line):
                    if (job["oled_meta"].get("enabled") == 1
                            and any(pending.values[name] != job["oled_meta"].get(name) for name in ("version", "mode", "skip", "enabled"))):
                        raise ProtocolError("OLED job mode changed for the same id")
                    job["oled_meta"] = dict(pending.values)
                    job["oled_expected"] = job["oled_expected"] or bool(pending.values["enabled"])
                    job["oled_pending_meta"] = None
                return
            if not job["oled_expected"] or job["oled_meta"].get("enabled") != 1:
                raise ProtocolError("OLED data without enabled OLED metadata")
            if job["phase"] not in TERMINAL:
                raise ProtocolError("OLED data arrived before terminal base status")
            if job["oled_records"] is None:
                if job["profile_records"] is None:
                    raise ProtocolError("OLED data requires complete profile records first")
                job["oled_records"] = OledRecords(self.current_id, job["profile_records"].records, job["oled_meta"]["version"])
            job["oled_records"].feed(line)
        except ProtocolError as exc:
            job["oled_errors"].append(str(exc))

    def oled_summary(self, job: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]], list[str]]:
        meta, expected = job["oled_meta"], job["oled_expected"]
        errors = list(job["oled_errors"])
        if job["oled_pending_meta"] is not None:
            errors.append("Incomplete OLED metadata block")
        records = job["oled_records"].records if job["oled_records"] else {}
        count = job["clock"].get("n", 0)
        if expected:
            if meta.get("enabled") != 1 or meta.get("id") != job["id"]:
                errors.append("Missing or mismatched terminal OLED metadata")
            if job["oled_request_mode"] is not None and meta.get("mode") != OLED_MODES[job["oled_request_mode"]]:
                errors.append("OLED mode differs from requested mode")
            if set(records) != set(range(count)) or any(not r["ended"] for r in records.values()):
                errors.append("Missing or incomplete OLED samples")
        profile, _, _ = self.profile_summary(job)
        rows = []
        for index, record in sorted(records.items()):
            row = {"id": job["id"], "i": index, "record_complete": record["ended"],
                   "oled_flags": record.get("flags"), "mode": next((name for name, value in OLED_MODES.items() if value == meta.get("mode")), None)}
            for stage, name in enumerate(OLED_STAGES):
                for key, label in (("p", "resident"), ("w", "elapsed")):
                    if stage in record[key]:
                        row[f"{name}_{label}_cycles"] = record[key][stage]
                if meta.get("version") == 3:
                    for key, label in (("q", "resident"), ("v", "elapsed")):
                        if stage in record[key]:
                            row[f"display_{name}_{label}_cycles"] = record[key][stage]
            row["accounting_valid"] = bool(record["ended"] and record.get("flags") == 0 and profile["accounting_valid"])
            rows.append(row)
        scale = 1_000_000 / job["clock"].get("clock_hz", 1) if job["clock"].get("clock_hz") else 0
        valid = [r for r in rows if r["accounting_valid"]]
        split = meta.get("mode", 0) >= 3
        runtime_valid = bool(meta.get("runtime_flags", 0) == 0 and meta.get("restored") == 1
                             and meta.get("applied_priority") == (16 if meta.get("mode") == 4 else 24))
        observed = bool(meta.get("skipped") if meta.get("skip") else meta.get("ended"))
        if split:
            observed = bool(observed and meta.get("display_ended") and runtime_valid)
        policy_observed = None
        policy_status = "not_applicable"
        if meta.get("mode", 0) >= 2:
            prefix = "display_" if split else ""
            enough = bool(meta.get(prefix + "render_begun", 0) >= 2 and meta.get(prefix + "rate_limited", 0) > 0)
            gap_valid = meta.get(prefix + "min_render_gap_ticks", 0) >= meta.get("period_ticks", 0)
            accounting_valid = bool(valid and len(valid) == len(rows) and profile["accounting_valid"]) if meta.get("version") == 3 else True
            runtime_ok = runtime_valid if meta.get("version") == 3 else True
            policy_observed = bool(expected and not errors and enough and gap_valid and accounting_valid and runtime_ok)
            policy_status = ("observed" if policy_observed else "violated" if enough and (not gap_valid or not runtime_ok) else "insufficient_evidence")
        return {"requested": expected, "requested_mode": job["oled_request_mode"], "metadata": meta,
                "sample_count": len(rows), "records_complete": expected and not errors,
                "accounting_valid": expected and not errors and bool(rows) and all(r["accounting_valid"] for r in rows),
                "application_observed": expected and observed,
                "policy_observed": policy_observed, "policy_status": policy_status,
                "runtime_valid": runtime_valid if meta.get("version") == 3 else None,
                "nonzero_flags": [{"i": r["i"], "flags": r["oled_flags"]} for r in rows if r.get("oled_flags")],
                "stage_residence": {name: timing_summary([r[f"{name}_resident_cycles"] * scale for r in valid]) for name in OLED_STAGES},
                "stage_elapsed": {name: timing_summary([r[f"{name}_elapsed_cycles"] * scale for r in valid]) for name in OLED_STAGES},
                "display_stage_residence": ({name: timing_summary([r[f"display_{name}_resident_cycles"] * scale for r in valid]) for name in OLED_STAGES} if meta.get("version") == 3 else None),
                "display_stage_elapsed": ({name: timing_summary([r[f"display_{name}_elapsed_cycles"] * scale for r in valid]) for name in OLED_STAGES} if meta.get("version") == 3 else None),
                "note": "P partitions IRQ-inclusive OLED worker residence; W partitions the full window by its current stage, including preemption, and is not exclusive OLED CPU time. V3 Q/V independently do the same for display; W and V each sum to the window and must not be added as disjoint CPU time. Frame counters cover the whole job, including probe/waits. Render gaps describe permitted opportunities, not physical panel FPS; loop gaps are separate. Runtime flags/restoration/priority are policy evidence. Display stack free is a historical unused watermark for that task, not full system stack acceptance. Zero frames or too few opportunities gives insufficient evidence. Terminal mode is saved configuration; normal behavior resumes after the job.",
                "errors": errors}, rows, errors

    def _profile(self, line: str) -> None:
        if self.current_id is None:
            self.auxiliary_errors.append(f"Orphan profile record: {line}")
            return
        job = self.job(self.current_id)
        try:
            if PROFILE_HEADER_RE.fullmatch(line):
                if job["profile_pending_meta"] is not None:
                    raise ProtocolError("Incomplete previous profile metadata block")
                job["profile_pending_meta"] = ProfileMetadata(self.current_id)
            pending = job["profile_pending_meta"]
            if pending is not None:
                if pending.feed(line):
                    if (job["profile_meta"].get("mode") == 1
                            and any(pending.values[name] != job["profile_meta"].get(name)
                                    for name in ("version", "mode", "tasks"))):
                        raise ProtocolError("Profile job schema/mode changed for the same id")
                    job["profile_meta"] = dict(pending.values)
                    job["profile_expected"] = bool(pending.values["mode"])
                    job["profile_pending_meta"] = None
                return
            if not job["profile_expected"] or job["profile_meta"].get("mode") != 1:
                raise ProtocolError("Profile data without enabled profile metadata")
            if job["phase"] not in TERMINAL:
                raise ProtocolError("Profile data arrived before terminal base status")
            if job["profile_records"] is None:
                job["profile_records"] = ProfileRecords(self.current_id, job["data"], job["clock"].get("clock_hz", 0), job["profile_meta"]["version"])
            job["profile_records"].feed(line)
        except ProtocolError as exc:
            job["profile_errors"].append(str(exc))

    def profile_summary(self, job: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]], list[str]]:
        meta = job["profile_meta"]
        expected = job["profile_expected"]
        errors = list(job["profile_errors"])
        if job["profile_pending_meta"] is not None:
            errors.append("Incomplete profile metadata block")
        records = job["profile_records"].records if job["profile_records"] else {}
        count = job["clock"].get("n", 0)
        if expected:
            if meta.get("mode") != 1 or meta.get("id") != job["id"] or meta.get("n") != count:
                errors.append("Missing or mismatched terminal profile metadata")
            if set(records) != set(range(count)) or any(not r["ended"] for r in records.values()):
                errors.append("Missing or incomplete profile samples")
        hz = job["clock"].get("clock_hz", 0)
        task_names = PROFILE_TASKS_V2 if meta.get("version") == 2 else PROFILE_TASKS
        expected_mapmask = 63 if meta.get("version") == 2 else 31
        rows = []
        for index, record in sorted(records.items()):
            row: dict[str, Any] = {"id": job["id"], "i": index, "record_complete": record["ended"],
                                   "kernel_cycles": job["data"][index]["cycles"], "base_work": job["data"][index]["work"]}
            for kind, names in (("s", ("plan_rtos", "start_rtos", "end_rtos")),
                                ("h", ("start_hal", "end_hal", "late_ticks")),
                                ("d", ("start_dwt", "end_dwt", "window_cycles")),
                                ("e", ("actual_work", "profile_flags"))):
                row.update(zip(names, record.get(kind, ())))
            if hz and "window_cycles" in row:
                row["window_us"] = row["window_cycles"] * 1_000_000 / hz
                row["window_minus_kernel_cycles"] = row["window_cycles"] - row["kernel_cycles"]
                row["window_minus_kernel_us"] = row["window_minus_kernel_cycles"] * 1_000_000 / hz
            previous = 0
            for slot, cumulative in sorted(record["w"].items()):
                row[f"work{slot}_cumulative_cycles"] = cumulative
                row[f"work{slot}_interval_cycles"] = cumulative - previous
                previous = cumulative
            for bucket, cycles in sorted(record["t"].items()):
                row[f"task_{task_names[bucket]}_cycles"] = cycles
            row["accounting_valid"] = bool(record["ended"] and row.get("profile_flags") == 0 and meta.get("mapmask") == expected_mapmask)
            rows.append(row)
        valid = [row for row in rows if row["accounting_valid"]]
        def metric(key: str, scale: float = 1.0) -> dict[str, Any]:
            return timing_summary([row[key] * scale for row in valid if key in row])
        scale = 1_000_000 / hz if hz else 0
        return {"requested": expected, "metadata": meta, "sample_count": len(rows),
                "records_complete": expected and not errors,
                "accounting_valid": expected and not errors and bool(rows) and meta.get("mapmask") == expected_mapmask and all(row["accounting_valid"] for row in rows),
                "nonzero_flags": [{"i": row["i"], "flags": row.get("profile_flags")} for row in rows if row.get("profile_flags", 0)],
                "late_ticks": {"count": len(valid), "max": max((r["late_ticks"] for r in valid), default=None),
                               "late_samples": sum(r["late_ticks"] != 0 for r in valid)},
                "window": metric("window_us"), "window_minus_kernel": metric("window_minus_kernel_us"),
                "work_intervals": {str(slot): metric(f"work{slot}_interval_cycles", scale) for slot in range(5)},
                "task_residence": {name: metric(f"task_{name}_cycles", scale) for name in task_names},
                "note": "DWT window includes the wrapper and Measure. Task residence includes IRQ time; it is not exclusive task CPU time. Window minus kernel is an additional interval, not pure measurement overhead.",
                "errors": errors}, rows, errors

    def feed(self, line: str) -> None:
        if line.startswith("CBOLED "):
            self._oled(line)
            return
        if line.startswith("CBPROF "):
            self._profile(line)
            return
        match = STATUS_RE.fullmatch(line)
        if match:
            job_id = int(match[1])
            self.current_id = job_id
            job = self.job(job_id)
            job["phase"] = match[2]
            job["phases"].append(match[2])
            return
        match = CONTRACT_RE.fullmatch(line)
        if match:
            if self.current_id is None:
                self.errors.append(f"Orphan metadata: {line}")
                return
            job = self.job(self.current_id)
            job["contract"] = {"no_motion": int(match[1]), "scope": match[2]}
            if job["contract"] != {"no_motion": 1, "scope": "components"}:
                job["errors"].append("Unexpected no-motion/components contract")
            return
        for regex, name, keys in (
            (DATA_RE, "data", ("cycles", "ticks", "work", "flag")),
            (RESULT_RE, "results", ("fk", "ik", "valid", "selected", "failed", "checksum")),
            (DATA_CSV_RE, "data", ("cycles", "ticks", "work", "flag")),
            (RESULT_CSV_RE, "results", ("fk", "ik", "valid", "selected", "failed", "checksum")),
        ):
            match = regex.fullmatch(line)
            if match:
                numbers = list(map(int, match.groups()))
                job_id, index, *values = numbers
                job = self.job(job_id)
                if self.current_id is not None and job_id != self.current_id:
                    self.errors.append(f"{name} id {job_id} differs from current status id {self.current_id}")
                if index >= 128:
                    job["errors"].append(f"{name} index out of range: {index}")
                if index in job[name]:
                    job["errors"].append(f"Duplicate {name} index {index}")
                else:
                    job[name][index] = dict(zip(keys, values))
                return
        for regex, name, keys in (
            (CONFIG_RE, "config", ("op", "fixture", "work", "samples")),
            (CLOCK_RE, "clock", ("clock_hz", "budget_us", "n", "period_ms")),
            (CLOCK_SHORT_RE, "clock", ("clock_hz", "budget_us", "n")),
            (PERIOD_RE, "clock", ("period_ms",)),
            (DWT_RE, "dwt", ("dwt_ctrl", "demcr")),
            (PROBE_RE, "probe", ("probe_cycles", "probe_ticks")),
            (SETUP_RE, "probe", ("setup_cycles",)),
        ):
            match = regex.fullmatch(line)
            if match:
                if self.current_id is None:
                    self.errors.append(f"Orphan metadata: {line}")
                    return
                job = self.job(self.current_id)
                values = dict(zip(keys, map(int, match.groups())))
                if name == "config" and job[name] and job[name] != values:
                    job["errors"].append("Configuration changed for same id")
                job[name].update(values)
                return
        if line.startswith("CBENCH "):
            self.errors.append(f"Unrecognized CBENCH record: {line}")

    def summarize(self, job_id: int | None = None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        if job_id is None:
            job_id = self.current_id
        if job_id is None or job_id not in self.jobs:
            return {"collection_complete": False, "job_success": False,
                    "kernel_collection_complete": False, "kernel_success": False,
                    "errors": self.errors + self.auxiliary_errors + ["No CBENCH job status"], "hardware_evidence": False}, []
        job = self.jobs[job_id]
        config, clock, dwt = job["config"], job["clock"], job["dwt"]
        errors = self.errors + list(job["errors"])
        phase = job["phase"]
        if phase not in TERMINAL:
            errors.append(f"Job is not terminal: {phase}")
        if not config:
            errors.append("Missing configuration")
        if not clock:
            errors.append("Missing clock metadata")
        if not dwt:
            errors.append("Missing DWT metadata")
        n = clock.get("n")
        if n is None or n > 128:
            errors.append("Invalid sample count")
            n = 0
        expected = set(range(n))
        if set(job["data"]) != expected:
            errors.append("DATA indices differ from final n")
        if set(job["results"]) != expected:
            errors.append("RESULT indices differ from final n")
        if phase == "DONE" and n != config.get("samples"):
            errors.append("DONE sample count differs from requested samples")
        hz = clock.get("clock_hz", 0)
        if n and (hz <= 0 or not (dwt.get("dwt_ctrl", 0) & 1)
                  or not (dwt.get("demcr", 0) & (1 << 24))):
            errors.append("Timing clock was not enabled")
        if clock and clock.get("period_ms") != 20:
            errors.append("Unexpected benchmark period")
        rows = []
        for index in sorted(set(job["data"]) | set(job["results"])):
            data, result = job["data"].get(index), job["results"].get(index)
            row = {"id": job_id, "i": index, "phase": phase, "clock_hz": hz,
                   "requested_work": config.get("work"), "pair_complete": data is not None and result is not None}
            if data:
                row.update(data)
                row["us"] = data["cycles"] * 1_000_000 / hz if hz > 0 else None
                if data["flag"] not in range(4):
                    errors.append(f"Unknown sample flag at {index}")
                if data["flag"] == 0 and data["work"] > 0 and data["cycles"] == 0:
                    errors.append(f"Completed nonzero work has zero DWT cycles at {index}")
                if data["work"] > config.get("work", 192):
                    errors.append(f"Work exceeds requested amount at {index}")
                row["full_work"] = data["work"] == config.get("work") and data["flag"] == 0
                if phase == "DONE" and not row["full_work"]:
                    errors.append(f"DONE has partial work or failure flag at {index}")
            if result:
                row.update(result)
            rows.append(row)
        durations = [r["us"] for r in rows if r.get("us") is not None]
        full_durations = [r["us"] for r in rows if r.get("full_work") and r.get("pair_complete") and r.get("us") is not None]
        kernel_complete = phase in TERMINAL and not errors
        errors.extend(self.auxiliary_errors)
        profile, _, profile_errors = self.profile_summary(job)
        errors.extend(profile_errors)
        oled, _, oled_errors = self.oled_summary(job)
        errors.extend(oled_errors)
        complete = phase in TERMINAL and not errors
        oled["rate10_test_passed"] = (bool(complete and phase == "DONE" and oled["accounting_valid"] and oled["policy_observed"])
                                      if oled["metadata"].get("mode", 0) >= 2 else None)
        oled["split_test_passed"] = (bool(complete and phase == "DONE" and oled["accounting_valid"] and oled["policy_observed"] and oled["runtime_valid"])
                                     if oled["metadata"].get("mode", 0) >= 3 else None)
        return {"id": job_id, "phase": phase, "config": config, "clock": clock, "dwt": dwt, "probe": job["probe"],
                "contract": job["contract"],
                "collection_complete": complete, "job_success": complete and phase == "DONE",
                "kernel_collection_complete": kernel_complete, "kernel_success": kernel_complete and phase == "DONE",
                "errors": errors, "timing_all_observed": timing_summary(durations),
                "timing_full_work": timing_summary(full_durations),
                "partial_work_samples": sum(not r.get("full_work", False) for r in rows),
                "profile": profile, "oled": oled,
                "hardware_evidence": False,
                "evidence_note": "Parser output alone does not establish hardware provenance; inspect capture and session metadata. Timings cover the diagnostic kernel, not the full online controller."}, rows


class Recorder:
    def __init__(self, path: Path):
        self.stream = path.open("x", encoding="utf-8", newline="\n")

    def record(self, direction: str, **values: Any) -> None:
        self.stream.write(json.dumps({"utc": utc_now(), "monotonic_ns": time.monotonic_ns(),
                                      "direction": direction, **values}, ensure_ascii=False) + "\n")
        self.stream.flush()

    def close(self) -> None:
        self.stream.close()


class SafeSession:
    def __init__(self, port: Any, recorder: Recorder, timeout: float, parser: CaptureParser):
        self.port, self.recorder, self.timeout, self.parser = port, recorder, timeout, parser
        self.buffer = bytearray()
        self.pending: list[str] = []
        self.last_boundary: ResponseBoundary | None = None

    def _background(self, line: str, boundary: ResponseBoundary) -> None:
        # Every byte/line is already retained in raw.jsonl.
        boundary.trailing(line)
        self.recorder.record("background", line=line, after=boundary.command)

    def _line(self, deadline: float) -> str:
        while time.monotonic() < deadline:
            if self.pending:
                return self.pending.pop(0)
            chunk = self.port.read(4096)
            if not chunk:
                continue
            self.recorder.record("rx_chunk", hex=chunk.hex())
            self.buffer.extend(chunk)
            if len(self.buffer) > 65536:
                raise ProtocolError("Receive fragment exceeds 64 KiB")
            while b"\n" in self.buffer:
                raw, _, tail = self.buffer.partition(b"\n")
                self.buffer = bytearray(tail)
                line = raw.rstrip(b"\r").decode("utf-8", errors="replace")
                self.recorder.record("rx", line=line)
                self.pending.append(line)
        raise TimeoutError("Response deadline expired; job was not automatically cancelled")

    def request(self, command: str) -> list[str]:
        if not command_allowed(command):
            raise ProtocolError(f"Command denied by allowlist: {command!r}")
        if self.pending or self.buffer:
            if self.last_boundary is None:
                raise ProtocolError("Unconsumed response/fragment before next request")
            # Resolve only bytes already buffered. Do not transmit over an old
            # fragment, or guess a response boundary from a quiet interval.
            deadline = time.monotonic() + self.timeout
            while self.pending or self.buffer:
                self._background(self._line(deadline), self.last_boundary)
        # Validate response context (in particular READ's job id) before writing.
        base_data = None
        clock_hz = 0
        profile_records = None
        if command.startswith("#CBENCH OREAD "):
            job = self.parser.jobs.get(self.parser.current_id)
            if (job is None or job["phase"] not in TERMINAL or job["oled_meta"].get("enabled") != 1
                    or job["oled_meta"].get("id") != self.parser.current_id
                    or job["profile_meta"].get("mode") != 1
                    or job["profile_meta"].get("n") != job["clock"].get("n")
                    or job["profile_records"] is None):
                raise ProtocolError("OREAD requires terminal enabled OLED and profile metadata")
            profile_records = job["profile_records"].records
            if set(profile_records) != set(range(job["clock"].get("n", 0))):
                raise ProtocolError("OREAD requires all profile samples first")
        if command.startswith("#CBENCH PREAD "):
            job = self.parser.jobs.get(self.parser.current_id)
            if (job is None or job["phase"] not in TERMINAL or job["profile_meta"].get("mode") != 1
                    or job["profile_meta"].get("id") != self.parser.current_id
                    or job["profile_meta"].get("n") != job["clock"].get("n")):
                raise ProtocolError("PREAD requires terminal enabled profile metadata")
            base_data, clock_hz = job["data"], job["clock"].get("clock_hz", 0)
        boundary = ResponseBoundary(command, expected_id=self.parser.current_id,
                                    base_data=base_data, clock_hz=clock_hz, profile_records=profile_records,
                                    profile_version=(self.parser.jobs.get(self.parser.current_id, {}).get("profile_meta", {}).get("version", 1)),
                                    oled_version=(self.parser.jobs.get(self.parser.current_id, {}).get("oled_meta", {}).get("version", 1)))
        encoded = (command + "\n").encode("ascii")
        self.recorder.record("tx", command=command, hex=encoded.hex(), stage="attempt")
        written = self.port.write(encoded)
        self.recorder.record("tx_result", command=command, bytes_written=written, bytes_expected=len(encoded))
        if written != len(encoded):
            raise ProtocolError("Partial serial write; no retry performed")
        rows = []
        deadline = time.monotonic() + self.timeout
        while True:
            line = self._line(deadline)
            rows.append(line)
            finished = boundary.feed(line)
            # Background CBENCH text during another query is evidence, not a
            # new accepted job. Validate id/index/schema before mutating context.
            if command.startswith("#CBENCH") and boundary.related(line):
                self.parser.feed(line)
            if finished:
                self.last_boundary = boundary
                while self.pending:
                    self._background(self.pending.pop(0), boundary)
                return rows


def capture_lines(path: Path) -> list[str]:
    """Read this collector's JSONL rx lines, or a plain line-oriented capture."""
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    if path.suffix.lower() != ".jsonl":
        return lines
    result = []
    for number, line in enumerate(lines, 1):
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ProtocolError(f"Malformed JSONL at line {number}") from exc
        if event.get("direction") == "rx":
            result.append(str(event["line"]))
    return result


def output_directory(root: Path) -> Path:
    path = root / ("cartesian-benchmark-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    path.mkdir(parents=True, exist_ok=False)
    return path


def export(output: Path, parser: CaptureParser, job_id: int | None, metadata: dict[str, Any]) -> dict[str, Any]:
    summary, rows = parser.summarize(job_id)
    summary["session"] = metadata
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    names = ["id", "i", "phase", "requested_work", "work", "flag", "cycles", "ticks", "clock_hz", "us", "pair_complete", "full_work", "fk", "ik", "valid", "selected", "failed", "checksum"]
    with (output / "samples.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, names)
        writer.writeheader()
        writer.writerows(rows)
    selected_id = parser.current_id if job_id is None else job_id
    profile_rows = parser.profile_summary(parser.jobs[selected_id])[1] if selected_id in parser.jobs else []
    profile_names = ["id", "i", "record_complete", "accounting_valid", "base_work", "actual_work", "profile_flags",
                     "plan_rtos", "start_rtos", "end_rtos", "start_hal", "end_hal", "late_ticks",
                     "start_dwt", "end_dwt", "window_cycles", "window_us", "kernel_cycles",
                     "window_minus_kernel_cycles", "window_minus_kernel_us"]
    profile_names += [name for slot in range(5) for name in (f"work{slot}_cumulative_cycles", f"work{slot}_interval_cycles")]
    profile_names += [f"task_{name}_cycles" for name in dict.fromkeys(PROFILE_TASKS + PROFILE_TASKS_V2)]
    with (output / "profile.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, profile_names)
        writer.writeheader()
        writer.writerows(profile_rows)
    oled_rows = parser.oled_summary(parser.jobs[selected_id])[1] if selected_id in parser.jobs else []
    oled_names = ["id", "i", "mode", "record_complete", "accounting_valid", "oled_flags"]
    oled_names += [f"{stage}_{kind}_cycles" for stage in OLED_STAGES for kind in ("resident", "elapsed")]
    oled_names += [f"display_{stage}_{kind}_cycles" for stage in OLED_STAGES for kind in ("resident", "elapsed")]
    with (output / "oled.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, oled_names)
        writer.writeheader()
        writer.writerows(oled_rows)
    return summary


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Explicit USB virtual COM port. Omit to avoid opening hardware.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--parse", type=Path, help="Offline raw.jsonl or plain capture; never opens a port")
    parser.add_argument("--id", type=int, help="Offline job id; default is the last status id")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--run", action="store_true", help="Explicitly request a no-motion compute job after read-only screening")
    action.add_argument("--cancel", action="store_true", help="Explicitly cancel an existing compute job")
    parser.add_argument("--profile", action="store_true", help="With --run, explicitly request supported task-residence/work profiling (work<=5)")
    parser.add_argument("--oled-mode", choices=tuple(OLED_MODES),
                        help="With --run --profile, request OLED accounting. rate-10hz caps worker drawing at 10Hz; split-same/split-low use a separate 10Hz display task at same/lower priority. All retain IMU/services. Split requires CBOLED v3 and CBPROF v2")
    parser.add_argument("--op", type=int, choices=range(6), default=2, help="0 FK; 1 IK; 2 FK+IK; 3 TCP/select; 4 rail candidates; 5 unreachable IK")
    parser.add_argument("--fixture", type=int, choices=range(3), default=0)
    parser.add_argument("--work", type=int, default=1)
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--budget-us", type=int, default=20000)
    parser.add_argument("--timeout", type=float, default=8.0, help="Per-response deadline seconds")
    parser.add_argument("--job-timeout", type=float, default=15.0, help="Overall compute-job wait seconds; no implicit cancel")
    parser.add_argument("--poll-ms", type=int, default=200)
    parser.add_argument("--output-root", type=Path, default=REPO / "firmware_artifacts")
    return parser


def main(argv: list[str] | None = None) -> int:
    cli = argument_parser()
    args = cli.parse_args(argv)
    if args.profile and not args.run:
        cli.error("--profile requires --run; saved profile records are detected automatically by --parse")
    if args.oled_mode is not None and not (args.run and args.profile):
        cli.error("--oled-mode requires --run --profile; saved OLED records are detected automatically by --parse")
    if args.parse and (args.port or args.run or args.cancel):
        cli.error("--parse cannot be combined with hardware options")
    if not args.port and not args.parse:
        if args.run or args.cancel:
            cli.error("--run/--cancel requires explicit --port")
        cli.print_help()
        return 0
    run_command = f"#CBENCH RUN {args.op},{args.fixture},{args.work},{args.samples},{args.budget_us}"
    if args.profile:
        run_command += ",1"
    if args.oled_mode is not None:
        run_command += "," + str(OLED_MODES[args.oled_mode])
    if not command_allowed(run_command):
        cli.error("work=1..192 (1..5 with --profile), samples=1..128, budget-us=100..20000 required")
    if args.timeout <= 0 or args.job_timeout <= 0 or not 20 <= args.poll_ms <= 5000 or args.baud <= 0:
        cli.error("positive timeout/baud and poll-ms=20..5000 required")
    output = output_directory(args.output_root)
    metadata = {"started_utc": utc_now(), "argv": sys.argv if argv is None else argv,
                "python": sys.version, "platform": platform.platform(),
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "script_path": str(Path(__file__).resolve()), "mode": "offline_parse" if args.parse else "usb_capture",
                "port": args.port, "baud": args.baud, "commands": [],
                "hardware_opened": False, "compute_requested": False, "motion_commands_permitted": False,
                "profile_requested": args.profile,
                "oled_requested_mode": args.oled_mode,
                "timeout_policy": "Preserve incomplete data; no implicit cancel/reconnect/reset/motion"}
    (output / "collector_source.py").write_bytes(Path(__file__).read_bytes())
    parsed = CaptureParser()
    recorder: Recorder | None = None
    port = None
    selected_id = args.id
    error = None
    try:
        if args.parse:
            metadata["input_path"] = str(args.parse.resolve())
            metadata["input_sha256"] = hashlib.sha256(args.parse.read_bytes()).hexdigest()
            for line in capture_lines(args.parse):
                parsed.feed(line)
        else:
            # Import is intentionally lazy: offline parsing/help need no pyserial.
            import serial
            metadata["pyserial_version"] = getattr(serial, "__version__", "unknown")
            recorder = Recorder(output / "raw.jsonl")
            recorder.record("session", note="Only fixed read queries and explicitly requested CBENCH computation/cancellation are permitted",
                            profile_requested=args.profile, oled_requested_mode=args.oled_mode)
            # No reset pulse is requested. Driver/device behavior on opening USB
            # is platform-dependent, so this is not a no-reset guarantee.
            port = serial.Serial(port=None, baudrate=args.baud, timeout=.05, write_timeout=2)
            port.dtr = False
            port.rts = False
            port.port = args.port
            port.open()
            metadata["hardware_opened"] = True
            session = SafeSession(port, recorder, args.timeout, parsed)

            def request(command: str) -> list[str]:
                metadata["commands"].append(command)
                return session.request(command)

            for command in QUERIES:
                request(command)
            selected_id = parsed.current_id
            if selected_id is None:
                raise ProtocolError("Firmware did not advertise CBENCH v=1")
            if parsed.jobs[selected_id]["contract"] != {"no_motion": 1, "scope": "components"}:
                raise ProtocolError("Firmware did not confirm CBENCH no_motion=1 scope=components")
            if args.cancel:
                request("#CBENCH CANCEL")
                selected_id = parsed.current_id
            if args.run:
                if parsed.jobs[selected_id]["phase"] not in TERMINAL:
                    raise ProtocolError("Existing benchmark is active; no implicit cancel")
                previous_id = selected_id
                if args.profile:
                    request("#CBENCH PROFILE")
                if args.oled_mode is not None:
                    request("#CBENCH OLED")
                    capability = parsed.jobs[selected_id]["oled_meta"]
                    metadata["oled_capability"] = dict(capability)
                    if args.oled_mode == "rate-10hz" and capability.get("version", 0) < 2:
                        raise ProtocolError("rate-10hz requires CBOLED v2; RUN was not sent")
                    if OLED_MODES[args.oled_mode] >= 3 and (capability.get("version") != 3
                            or capability.get("lanes") != 2 or parsed.jobs[selected_id]["profile_meta"].get("version") != 2
                            or parsed.jobs[selected_id]["profile_meta"].get("tasks") != 7):
                        raise ProtocolError("split modes require CBOLED v3/two lanes and CBPROF v2/seven tasks; RUN was not sent")
                request(run_command)
                metadata["compute_requested"] = True
                selected_id = parsed.current_id
                if selected_id == previous_id:
                    raise ProtocolError("RUN did not return a new job id")
                if args.profile:
                    parsed.jobs[selected_id]["profile_expected"] = True
                if args.oled_mode is not None:
                    parsed.jobs[selected_id]["oled_expected"] = True
                    parsed.jobs[selected_id]["oled_request_mode"] = args.oled_mode
            if args.run or args.cancel:
                deadline = time.monotonic() + args.job_timeout
                while parsed.jobs[selected_id]["phase"] not in TERMINAL:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Compute-job deadline expired; job was not automatically cancelled")
                    time.sleep(args.poll_ms / 1000)
                    request("#CBENCH")
                    if parsed.current_id != selected_id:
                        raise ProtocolError("Benchmark id changed during polling")
            # Also export an already-terminal previous job in read-only mode.
            if parsed.jobs[selected_id]["phase"] in TERMINAL:
                n = parsed.jobs[selected_id]["clock"].get("n", 0)
                if not 0 <= n <= 128:
                    raise ProtocolError("Invalid final sample count")
                for offset in range(0, n, 8):
                    request(f"#CBENCH READ {offset},{min(8, n-offset)}")
                if args.profile:
                    request("#CBENCH PROFILE")
                    meta = parsed.jobs[selected_id]["profile_meta"]
                    if meta.get("mode") != 1 or meta.get("n") != n:
                        raise ProtocolError("Profile RUN was not enabled or final count changed")
                    for offset in range(0, n, 4):
                        request(f"#CBENCH PREAD {offset},{min(4, n-offset)}")
                if args.oled_mode is not None:
                    request("#CBENCH OLED")
                    meta = parsed.jobs[selected_id]["oled_meta"]
                    if meta.get("enabled") != 1 or meta.get("mode") != OLED_MODES[args.oled_mode]:
                        raise ProtocolError("OLED RUN was not enabled in the requested mode")
                    for offset in range(0, n, 4):
                        request(f"#CBENCH OREAD {offset},{min(4, n-offset)}")
            request("#STACK")
    except (Exception, KeyboardInterrupt) as exc:
        error = f"{type(exc).__name__}: {exc}"
        if recorder:
            recorder.record("error", detail=error)
    finally:
        if port is not None:
            try:
                port.close()
            except Exception as exc:
                metadata["close_error"] = str(exc)
        if recorder:
            recorder.close()
        metadata["finished_utc"] = utc_now()
        metadata["error"] = error
        (output / "session.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        summary = export(output, parsed, selected_id, metadata)
    print(output)
    if error:
        print(error, file=sys.stderr)
        return 2
    # Read-only screening can legitimately return IDLE without collected data.
    if args.port and not args.run and not args.cancel and summary.get("phase") == "IDLE":
        return 0
    return 0 if summary.get("collection_complete") else 3


if __name__ == "__main__":
    raise SystemExit(main())
