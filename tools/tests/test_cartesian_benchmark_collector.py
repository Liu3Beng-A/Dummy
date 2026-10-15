"""Offline collector protocol/allowlist tests; no serial port is opened."""
from __future__ import annotations

import importlib.util
import contextlib
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location("cartesian_benchmark", Path(__file__).with_name("run_cartesian_benchmark.py"))
bench = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bench)


def metadata(job_id=7, phase="DONE", n=2, samples=2, work=3, hz=168000000):
    return [f"CBENCH v=1 id={job_id} phase={phase}",
            f"CBENCH config op=2 fixture=0 work={work} samples={samples}",
            f"CBENCH clock_hz={hz} budget_us=20000 n={n} period_ms=20",
            "CBENCH dwt_ctrl=1 demcr=16777216"]


def sample(index, job_id=7, work=3, flag=0, cycles=168000):
    return [f"CBENCH DATA id={job_id} i={index} cycles={cycles} ticks=1 work={work} flag={flag}",
            f"CBENCH RESULT id={job_id} i={index} fk=3 ik=3 valid=24 selected=0 failed=0 checksum=42"]


def parse(lines):
    parser = bench.CaptureParser()
    for line in lines:
        parser.feed(line)
    return parser.summarize()


def native_status(job_id=7, phase="DONE", n=2, hz=168000000):
    """All lines emitted by current Native BenchStatus, including its contract."""
    return [f"CBENCH v=1 id={job_id} phase={phase}",
            "CBENCH config op=2 fixture=0 work=3 samples=2",
            f"CBENCH clock_hz={hz} budget_us=20000 n={n}",
            "CBENCH period_ms=20",
            f"CBENCH dwt_ctrl={1 if hz else 0} demcr={16777216 if hz else 0}",
            f"CBENCH probe_cycles={168000 if hz else 0} probe_ticks={1 if hz else 0}",
            f"CBENCH setup_cycles={240 if hz else 0}",
            "CBENCH no_motion=1 scope=components"]


class AllowlistTests(unittest.TestCase):
    def test_only_diagnostic_queries_and_valid_bounded_jobs(self):
        for command in bench.QUERIES + ("#CBENCH CANCEL", "#CBENCH RUN 0,0,1,1,100",
                                       "#CBENCH RUN 5,2,192,128,20000", "#CBENCH READ 120,8"):
            self.assertTrue(bench.command_allowed(command), command)

    def test_motion_configuration_injection_and_out_of_range_are_denied(self):
        forbidden = ["!START", "!DISABLE", "!STOP", "#CMDMODE 2", "#TOOL_SAVE", "#TOOL_SET 0,0,114,0,0,0",
                     "@ 1,2,3,0,0,0", "> 0,0,0,0,0,0", "#FW_VERSION 1", "#FW_VERSION 0\n!START",
                     "#CBENCH\r", "#CBENCH RUN 6,0,1,1,100", "#CBENCH RUN 0,3,1,1,100",
                     "#CBENCH RUN 0,0,0,1,100", "#CBENCH RUN 0,0,193,1,100",
                     "#CBENCH RUN 0,0,1,129,100", "#CBENCH RUN 0,0,1,1,99",
                     "#CBENCH RUN 0,0,1,1,20001", "#CBENCH READ 127,2", "#CBENCH READ 0,9",
                     "#CBENCH READ -1,1", "#CBENCH CANCEL\n!STOP", "#CBENCH RUN ٠,٠,١,١,١٠٠"]
        for command in forbidden:
            self.assertFalse(bench.command_allowed(command), command)


class VersionBoundaryTests(unittest.TestCase):
    def test_outer_end_does_not_finish_async_version(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        self.assertFalse(boundary.feed("ok FW_VERSION [0] req=123 pending"))
        self.assertFalse(boundary.feed("end"))
        for field in ["v=20261015.06 type=CORE", "epoch=1", "group=ABC", "h0=111", "h1=222"]:
            self.assertFalse(boundary.feed("FW_VERSION [0] req=123 " + field))
        self.assertTrue(boundary.feed("FW_VERSION [0] req=123 end"))

    def test_version_end_does_not_hide_truncation_or_stale_request(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        boundary.feed("ok FW_VERSION [0] req=123 pending")
        boundary.feed("end")
        for field in ["v=x", "epoch=1", "group=x", "h0=x", "end"]:
            self.assertFalse(boundary.feed("FW_VERSION [0] req=123 " + field))
        self.assertFalse(boundary.feed("FW_VERSION [0] req=122 h1=x"))
        self.assertTrue(boundary.feed("FW_VERSION [0] req=123 h1=x"))

    def test_version_error_requires_outer_end(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        self.assertFalse(boundary.feed("error FW_VERSION busy"))
        self.assertTrue(boundary.feed("end"))
        self.assertEqual(boundary.error, "error FW_VERSION busy")


class ParserTests(unittest.TestCase):
    def test_completed_nonzero_work_with_zero_cycles_is_preserved_but_invalid(self):
        summary, rows = parse(metadata(n=1, samples=1) + sample(0, cycles=0))
        self.assertFalse(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertIn("Completed nonzero work has zero DWT cycles at 0", summary["errors"])
        self.assertEqual(rows[0]["cycles"], 0)
        self.assertEqual(rows[0]["us"], 0)

    def test_full_native_status_idle_then_done_and_csv_integration(self):
        parser = bench.CaptureParser()
        for line in native_status(job_id=0, phase="IDLE", n=0, hz=0):
            parser.feed(line)
        idle, rows = parser.summarize()
        self.assertTrue(idle["collection_complete"])
        self.assertFalse(idle["job_success"])
        self.assertEqual(rows, [])
        for line in native_status(job_id=1, phase="PENDING", n=0, hz=0) + native_status(job_id=1):
            parser.feed(line)
        for line in ["CBENCH DATA 1,0,168000,1,3,0", "CBENCH RESULT 1,0,3,3,24,0,0,42",
                     "CBENCH DATA 1,1,336000,2,3,0", "CBENCH RESULT 1,1,3,3,24,0,0,43"]:
            parser.feed(line)
        done, rows = parser.summarize()
        self.assertTrue(done["collection_complete"])
        self.assertTrue(done["job_success"])
        self.assertEqual(done["contract"], {"no_motion": 1, "scope": "components"})
        self.assertEqual(done["probe"], {"probe_cycles": 168000, "probe_ticks": 1, "setup_cycles": 240})
        self.assertEqual(len(rows), 2)

    def test_invalid_no_motion_or_scope_cannot_be_marked_complete(self):
        for contract in ["CBENCH no_motion=0 scope=components", "CBENCH no_motion=1 scope=controller"]:
            summary, _ = parse(native_status()[:-1] + [contract] + sample(0) + sample(1))
            self.assertFalse(summary["collection_complete"])
            self.assertIn("Unexpected no-motion/components contract", summary["errors"])

    def test_compact_wire_format_split_metadata_and_uint32_maximum(self):
        job_id = 4294967295
        maximum = 4294967295
        lines = [f"CBENCH v=1 id={job_id} phase=DONE",
                 "CBENCH config op=4 fixture=2 work=192 samples=128",
                 "CBENCH clock_hz=168000000 budget_us=20000 n=128",
                 "CBENCH period_ms=20",
                 "CBENCH dwt_ctrl=4294967295 demcr=4294967295",
                 f"CBENCH probe_cycles={maximum} probe_ticks={maximum}",
                 f"CBENCH setup_cycles={maximum}"]
        for index in range(128):
            lines += [f"CBENCH DATA {job_id},{index},{maximum},{maximum},192,0",
                      f"CBENCH RESULT {job_id},{index},384,192,1536,192,192,{maximum}"]
        summary, rows = parse(lines)
        self.assertTrue(summary["collection_complete"])
        self.assertEqual(summary["id"], maximum)
        self.assertEqual(summary["clock"]["period_ms"], 20)
        self.assertEqual(summary["probe"]["setup_cycles"], maximum)
        self.assertEqual(summary["timing_full_work"]["count"], 128)
        self.assertEqual(rows[-1]["i"], 127)
        self.assertEqual(rows[-1]["cycles"], maximum)
        self.assertEqual(rows[-1]["ticks"], maximum)
        self.assertEqual(rows[-1]["checksum"], maximum)
        # Diagnostic success means the job ran; over-budget observations remain
        # visible and do not become a real-time-performance pass.
        self.assertEqual(summary["timing_all_observed"]["over_20000us"], 128)

    def test_every_maximum_legal_wire_line_fits_respond_payload_limit(self):
        respond = (bench.REPO / "firmware_new/dummy-ref-core-fw/Bsp/communication/ascii_processor.hpp").read_text(encoding="utf-8")
        # Tie the limit to the actual emitter rather than assuming an enlarged
        # buffer or silently accepting its "error response too long" fallback.
        self.assertIn("char response[64];", respond)
        self.assertIn("needed >= static_cast<int>(sizeof(response) - 2)", respond)
        lines = ["CBENCH v=1 id=4294967295 phase=CLOCK_ERROR",
                 "CBENCH config op=5 fixture=2 work=192 samples=128",
                 "CBENCH clock_hz=4294967295 budget_us=20000 n=128",
                 "CBENCH period_ms=20",
                 "CBENCH dwt_ctrl=4294967295 demcr=4294967295",
                 "CBENCH probe_cycles=4294967295 probe_ticks=4294967295",
                 "CBENCH setup_cycles=4294967295",
                 "CBENCH no_motion=1 scope=components",
                 "CBENCH DATA 4294967295,127,4294967295,4294967295,192,3",
                 "CBENCH RESULT 4294967295,127,384,192,1536,192,192,4294967295"]
        for line in lines:
            # Existing Respond has a 64-byte buffer and rejects needed >= 62.
            self.assertLess(len(line.encode("ascii")), 62, line)
        self.assertEqual(len(lines[-1]), 60)
        self.assertEqual(len(lines[-2]), 54)

    def test_compact_split_clock_missing_period_is_not_complete(self):
        lines = ["CBENCH v=1 id=7 phase=DONE",
                 "CBENCH config op=2 fixture=0 work=3 samples=1",
                 "CBENCH clock_hz=168000000 budget_us=20000 n=1",
                 "CBENCH dwt_ctrl=1 demcr=16777216",
                 "CBENCH DATA 7,0,168000,1,3,0",
                 "CBENCH RESULT 7,0,3,3,24,0,0,42"]
        summary, _ = parse(lines)
        self.assertFalse(summary["collection_complete"])
        self.assertIn("Unexpected benchmark period", summary["errors"])

    def test_complete_job_and_quantiles(self):
        summary, rows = parse(metadata() + sample(0) + sample(1, cycles=336000))
        self.assertTrue(summary["collection_complete"])
        self.assertTrue(summary["job_success"])
        self.assertEqual(summary["timing_full_work"]["p50_us"], 1500)
        self.assertEqual(summary["timing_full_work"]["p95_us"], 1950)
        self.assertEqual(summary["timing_full_work"]["p99_us"], 1990)
        self.assertEqual(len(rows), 2)
        self.assertFalse(summary["hardware_evidence"])

    def test_terminal_budget_preserves_partial_work_without_success(self):
        summary, rows = parse(metadata(phase="BUDGET", n=1, samples=16) + sample(0, work=2, flag=1, cycles=3500000))
        self.assertTrue(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertEqual(summary["partial_work_samples"], 1)
        self.assertEqual(summary["timing_all_observed"]["over_20000us"], 1)
        self.assertEqual(summary["timing_full_work"]["count"], 0)
        self.assertFalse(rows[0]["full_work"])

    def test_missing_result_mismatch_and_duplicate_fail_closed(self):
        cases = [metadata() + sample(0) + sample(1)[:1],
                 metadata() + sample(0) + sample(1, job_id=8),
                 metadata() + sample(0) + sample(1) + sample(0, job_id=8),
                 metadata() + sample(0) + sample(1) + sample(1),
                 metadata(n=1) + sample(0)]
        for lines in cases:
            summary, _ = parse(lines)
            self.assertFalse(summary["collection_complete"], lines)
            self.assertFalse(summary["job_success"])
            self.assertTrue(summary["errors"])

    def test_nonterminal_and_disabled_clock_are_not_passes(self):
        for lines in [metadata(phase="RUNNING") + sample(0) + sample(1),
                      metadata(hz=0) + sample(0) + sample(1),
                      metadata()[:-1] + ["CBENCH dwt_ctrl=0 demcr=16777216"] + sample(0) + sample(1),
                      metadata() + sample(0, flag=1) + sample(1)]:
            summary, _ = parse(lines)
            self.assertFalse(summary["collection_complete"])

    def test_clock_error_zero_samples_is_a_recorded_failure_not_success(self):
        summary, rows = parse(metadata(phase="CLOCK_ERROR", n=0, hz=0))
        self.assertTrue(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertEqual(rows, [])

    def test_pending_metadata_is_replaced_by_terminal_measurement(self):
        summary, _ = parse(metadata(phase="PENDING", n=0, hz=0) + metadata() + sample(0) + sample(1))
        self.assertTrue(summary["job_success"])

    def test_malformed_record_and_configuration_change_are_errors(self):
        for extra in ["CBENCH DATA id=7 i=0 cycles=abc ticks=1 work=3 flag=0",
                      "CBENCH config op=2 fixture=0 work=4 samples=2"]:
            summary, _ = parse(metadata() + sample(0) + sample(1) + [extra])
            self.assertFalse(summary["collection_complete"])


class FakePort:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []

    def read(self, count):
        return self.chunks.pop(0) if self.chunks else b""

    def write(self, data):
        self.writes.append(data)
        return len(data)


class SessionTests(unittest.TestCase):
    def test_full_fake_usb_capture_uses_current_native_protocol(self):
        class FakeUsb(FakePort):
            def __init__(self):
                super().__init__([])
                self.started = False
                self.opened = False

            def open(self):
                self.opened = True

            def close(self):
                self.opened = False

            def write(self, encoded):
                command = encoded.decode("ascii").strip()
                self.writes.append(encoded)
                if command == "#FW_VERSION 0":
                    lines = ["ok FW_VERSION [0] req=12 pending", "end"]
                    lines += ["FW_VERSION [0] req=12 " + field for field in
                              ["v=20261015.06 type=CORE", "epoch=1", "group=AAA", "h0=BBB", "h1=CCC", "end"]]
                elif command == "#KIN_CONFIG":
                    lines = ["KIN v=2 xyz=mm abc=deg joints=deg rail=mm", "end"]
                elif command == "#TOOL":
                    lines = ["TOOL v=1 rev=1 dirty=0 storage=0", "TOOL XYZ 0.000 0.000 114.000", "TOOL ABC 0.000 0.000 0.000", "end"]
                elif command == "#MOTION_STATUS":
                    lines = ["MOTION enabled=0", "end"]
                elif command == "#STACK":
                    lines = ["ok STACK usb=6000 uart=6000 heap=32000", "ok STACK cmd=7000 size=8192 heap_min=31000", "ok STACK rt=3900 oled=1800 rgb=1800", "end"]
                elif command == "#CBENCH":
                    lines = (native_status(job_id=1) if self.started else native_status(job_id=0, phase="IDLE", n=0, hz=0)) + ["end"]
                elif command == "#CBENCH RUN 2,0,3,2,20000":
                    self.started = True
                    lines = native_status(job_id=1, phase="PENDING", n=0, hz=0) + ["end"]
                elif command == "#CBENCH READ 0,2":
                    lines = ["CBENCH DATA 1,0,168000,1,3,0", "CBENCH RESULT 1,0,3,3,24,0,0,42",
                             "CBENCH DATA 1,1,336000,2,3,0", "CBENCH RESULT 1,1,3,3,24,0,0,43", "end"]
                else:
                    raise AssertionError(f"Unexpected command: {command}")
                self.chunks.append(("\r\n".join(lines) + "\r\n").encode("ascii"))
                return len(encoded)

        port = FakeUsb()
        fake_serial = types.SimpleNamespace(Serial=lambda **kwargs: port, __version__="offline_fake")
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.dict("sys.modules", {"serial": fake_serial}), contextlib.redirect_stdout(io.StringIO()) as printed:
                code = bench.main(["--port", "OFFLINE_FAKE", "--run", "--work", "3", "--samples", "2",
                                   "--poll-ms", "20", "--output-root", temporary])
            self.assertEqual(code, 0)
            self.assertFalse(port.opened)
            output = Path(printed.getvalue().strip())
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertTrue(summary["job_success"])
            self.assertEqual(summary["contract"], {"no_motion": 1, "scope": "components"})
            self.assertEqual(summary["timing_full_work"]["count"], 2)
            self.assertEqual(summary["session"]["pyserial_version"], "offline_fake")
            self.assertFalse(summary["hardware_evidence"])
            self.assertTrue(all(bench.command_allowed(command.decode("ascii").strip()) for command in port.writes))

    def test_default_mode_does_not_import_or_open_serial(self):
        original_import = __import__

        def guarded_import(name, *args, **kwargs):
            if name == "serial":
                self.fail("Default mode imported serial")
            return original_import(name, *args, **kwargs)

        with mock.patch("builtins.__import__", side_effect=guarded_import), contextlib.redirect_stdout(io.StringIO()) as printed:
            self.assertEqual(bench.main([]), 0)
        self.assertIn("--port", printed.getvalue())

    def test_offline_export_retains_partial_failure_and_does_not_import_serial(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            capture = root / "input.txt"
            capture.write_text("\n".join(metadata(phase="BUDGET", n=1, samples=16) + sample(0, work=2, flag=1)), encoding="utf-8")
            original_import = __import__

            def guarded_import(name, *args, **kwargs):
                if name == "serial":
                    self.fail("Offline mode imported serial")
                return original_import(name, *args, **kwargs)

            with mock.patch("builtins.__import__", side_effect=guarded_import), contextlib.redirect_stdout(io.StringIO()) as printed:
                self.assertEqual(bench.main(["--parse", str(capture), "--output-root", str(root / "output")]), 0)
            output = Path(printed.getvalue().strip())
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertTrue(summary["collection_complete"])
            self.assertFalse(summary["job_success"])
            self.assertFalse(summary["session"]["hardware_opened"])
            self.assertEqual(summary["session"]["commands"], [])
            self.assertIn("168000", (output / "samples.csv").read_text(encoding="utf-8"))
            self.assertTrue((output / "collector_source.py").exists())

    def test_fragmented_response_is_reassembled_and_raw_bytes_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "raw.jsonl"
            recorder = bench.Recorder(path)
            port = FakePort([b"CBEN", b"CH v=1 id=0 phase=IDLE\r\ne", b"nd\r\n"])
            session = bench.SafeSession(port, recorder, .1, bench.CaptureParser())
            self.assertEqual(session.request("#CBENCH"), ["CBENCH v=1 id=0 phase=IDLE", "end"])
            recorder.close()
            self.assertEqual(port.writes, [b"#CBENCH\n"])
            events = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(sum(e["direction"] == "rx_chunk" for e in events), 3)
            self.assertTrue(all("utc" in e and "monotonic_ns" in e for e in events))

    def test_denied_command_never_writes(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            port = FakePort([])
            session = bench.SafeSession(port, recorder, .01, bench.CaptureParser())
            with self.assertRaises(bench.ProtocolError):
                session.request("!START")
            recorder.close()
            self.assertEqual(port.writes, [])

    def test_timeout_does_not_emit_cancel_or_retry(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            port = FakePort([b"CBENCH v=1 id=7 phase=RUNNING\r\n"])
            session = bench.SafeSession(port, recorder, .005, bench.CaptureParser())
            with self.assertRaises(TimeoutError):
                session.request("#CBENCH")
            recorder.close()
            self.assertEqual(port.writes, [b"#CBENCH\n"])

    def test_plain_and_jsonl_offline_capture(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "raw.jsonl"
            events = [{"direction": "tx", "command": "#CBENCH"},
                      {"direction": "rx_chunk", "hex": "00"},
                      {"direction": "rx", "line": "CBENCH v=1 id=0 phase=IDLE"}]
            path.write_text("\n".join(map(json.dumps, events)), encoding="utf-8")
            self.assertEqual(bench.capture_lines(path), ["CBENCH v=1 id=0 phase=IDLE"])


if __name__ == "__main__":
    unittest.main()
