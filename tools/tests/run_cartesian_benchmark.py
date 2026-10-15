"""Collect the no-motion F405 CBENCH diagnostic; never flash or send motion.

Without --port this script only shows help or parses a saved capture. Opening a
USB port requires --port; running computation additionally requires --run.
Examples (COM numbers are examples, not an instruction to connect now):
  python tools/tests/run_cartesian_benchmark.py --port COM9
  python tools/tests/run_cartesian_benchmark.py --port COM9 --run --op 2 --work 1
  python tools/tests/run_cartesian_benchmark.py --parse PATH/raw.jsonl
  python tools/tests/run_cartesian_benchmark.py --port COM9 --cancel

This measures the diagnostic kernel, not a complete online MoveL follower.
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


class ProtocolError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def command_allowed(command: str) -> bool:
    """A closed allowlist; no arbitrary passthrough or newline injection."""
    if not command.isascii():
        return False
    if command in QUERIES or command == "#CBENCH CANCEL":
        return True
    match = re.fullmatch(r"#CBENCH RUN (\d+),(\d+),(\d+),(\d+),(\d+)", command)
    if match:
        op, fixture, work, samples, budget = map(int, match.groups())
        return (0 <= op <= 5 and 0 <= fixture <= 2 and 1 <= work <= 192
                and 1 <= samples <= 128 and 100 <= budget <= 20000)
    match = re.fullmatch(r"#CBENCH READ (\d+),(\d+)", command)
    if match:
        offset, count = map(int, match.groups())
        return 0 <= offset < 128 and 1 <= count <= 8 and offset + count <= 128
    return False


class ResponseBoundary:
    """Wait for the ASCII end AND the asynchronous core-version end, if used."""
    def __init__(self, command: str):
        self.command = command
        self.outer_end = False
        self.request: int | None = None
        self.version_fields: set[str] = set()
        self.error: str | None = None

    def feed(self, line: str) -> bool:
        if line == "end":
            self.outer_end = True
        if line.startswith("error "):
            self.error = line
        if self.command == "#FW_VERSION 0":
            match = re.fullmatch(r"ok FW_VERSION \[0\] req=(\d+) pending", line)
            if match:
                if self.request is not None and self.request != int(match[1]):
                    raise ProtocolError("FW_VERSION request changed within one response")
                self.request = int(match[1])
            match = re.fullmatch(r"FW_VERSION \[0\] req=(\d+) (.+)", line)
            if match and self.request is not None and int(match[1]) == self.request:
                field = match[2].split("=", 1)[0]
                if field in {"v", "epoch", "group", "h0", "h1", "end"}:
                    if field in self.version_fields:
                        raise ProtocolError("Duplicate FW_VERSION field")
                    self.version_fields.add(field)
            if self.error:
                return self.outer_end
            return self.outer_end and self.version_fields == {"v", "epoch", "group", "h0", "h1", "end"}
        return self.outer_end


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

    def job(self, job_id: int) -> dict[str, Any]:
        return self.jobs.setdefault(job_id, {"id": job_id, "phase": None, "config": {},
                                            "clock": {}, "dwt": {}, "probe": {}, "contract": {}, "data": {}, "results": {},
                                            "errors": [], "phases": []})

    def feed(self, line: str) -> None:
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
                    "errors": self.errors + ["No CBENCH job status"], "hardware_evidence": False}, []
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
        complete = phase in TERMINAL and not errors
        return {"id": job_id, "phase": phase, "config": config, "clock": clock, "dwt": dwt, "probe": job["probe"],
                "contract": job["contract"],
                "collection_complete": complete, "job_success": complete and phase == "DONE",
                "errors": errors, "timing_all_observed": timing_summary(durations),
                "timing_full_work": timing_summary(full_durations),
                "partial_work_samples": sum(not r.get("full_work", False) for r in rows),
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
                self.parser.feed(line)
                self.pending.append(line)
        raise TimeoutError("Response deadline expired; job was not automatically cancelled")

    def request(self, command: str) -> list[str]:
        if not command_allowed(command):
            raise ProtocolError(f"Command denied by allowlist: {command!r}")
        if self.pending or self.buffer:
            raise ProtocolError("Unconsumed response/fragment before next request")
        encoded = (command + "\n").encode("ascii")
        self.recorder.record("tx", command=command, hex=encoded.hex(), stage="attempt")
        written = self.port.write(encoded)
        self.recorder.record("tx_result", command=command, bytes_written=written, bytes_expected=len(encoded))
        if written != len(encoded):
            raise ProtocolError("Partial serial write; no retry performed")
        boundary = ResponseBoundary(command)
        rows = []
        deadline = time.monotonic() + self.timeout
        while True:
            line = self._line(deadline)
            rows.append(line)
            if boundary.feed(line):
                if boundary.error:
                    raise ProtocolError(boundary.error)
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
    if args.parse and (args.port or args.run or args.cancel):
        cli.error("--parse cannot be combined with hardware options")
    if not args.port and not args.parse:
        if args.run or args.cancel:
            cli.error("--run/--cancel requires explicit --port")
        cli.print_help()
        return 0
    run_command = f"#CBENCH RUN {args.op},{args.fixture},{args.work},{args.samples},{args.budget_us}"
    if not command_allowed(run_command):
        cli.error("work=1..192, samples=1..128, budget-us=100..20000 required")
    if args.timeout <= 0 or args.job_timeout <= 0 or not 20 <= args.poll_ms <= 5000 or args.baud <= 0:
        cli.error("positive timeout/baud and poll-ms=20..5000 required")
    output = output_directory(args.output_root)
    metadata = {"started_utc": utc_now(), "argv": sys.argv if argv is None else argv,
                "python": sys.version, "platform": platform.platform(),
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "script_path": str(Path(__file__).resolve()), "mode": "offline_parse" if args.parse else "usb_capture",
                "port": args.port, "baud": args.baud, "commands": [],
                "hardware_opened": False, "compute_requested": False, "motion_commands_permitted": False,
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
            recorder.record("session", note="Only fixed read queries and explicitly requested CBENCH computation/cancellation are permitted")
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
                request(run_command)
                metadata["compute_requested"] = True
                selected_id = parsed.current_id
                if selected_id == previous_id:
                    raise ProtocolError("RUN did not return a new job id")
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
