"""Offline collector protocol/allowlist tests; no serial port is opened."""
from __future__ import annotations

import importlib.util
import contextlib
import io
import hashlib
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


def native_version(request=123):
    fields = ["v=20261015.07 type=CORE", "epoch=1792073743", "group=FF09EFFE4FCA49BB",
              "h0=AE75C7CED24C5AFA854C2B62F31FB987", "h1=9F4B2C76379E8F711D4B2B79A715D728", "end"]
    return [f"ok FW_VERSION [0] req={request} pending"] + [f"FW_VERSION [0] req={request} {field}" for field in fields]


def native_kin():
    return ["KIN v=2 xyz=mm abc=deg joints=deg rail=mm", "KIN base=207.3 db=0 arm=170 forearm=117",
            "KIN elbow=69.5 wrist=113 tool_default=114", "KIN rail_axis=+Y rail_min=-250 rail_max=250",
            "KIN TCP=@T x,y,z,a,b,c,speed,rail_auto"]


def native_tool():
    return ["TOOL v=1 rev=1 dirty=0 storage=0", "TOOL XYZ 0.000 0.000 114.000", "TOOL ABC 0.000 0.000 0.000"]


def native_stack():
    return ["ok STACK usb=6000 uart=6000 heap=32000", "ok STACK cmd=7000 size=8192 heap_min=31000",
            "ok STACK rt=3900 oled=1800 rgb=1800"]


def native_motion(optional=False):
    rows = ["MOTION id=0 kind=MOVEJ phase=0 mode=2 ms=0", "MOTION ready=000 confirmed=000 progress=000",
            "MOTION send=0 query=0 reject=0", "MOTION plan_ms=0.0 moving=000 txok=000 pending=000",
            "MOTION spread tx=-1 settled_obs=-1"]
    if optional:
        rows += ["MOTION CAN alst=0 terr=0 busy=0", "MOTION cancelled_drop=1"]
    for node in range(1, 8):
        rows += [f"MOTION J{node} goal=0.0000 real=0.0000 f=00 age=4294967295",
                 f"MOTION J{node} tx=-1 ready=-1 try=0 fail=0", f"MOTION J{node} plan=-1 pos=-1 settled=-1",
                 f"MOTION J{node} v=0.00000 a=30.000 d=0.0000"]
    return rows


def csv_samples(job_id=7, first=0, count=2):
    rows = []
    for index in range(first, first + count):
        rows += [f"CBENCH DATA {job_id},{index},168000,1,3,0", f"CBENCH RESULT {job_id},{index},3,3,24,0,0,42"]
    return rows


def profile_metadata(job_id=7, mode=1, n=2):
    return [f"CBPROF v=1 id={job_id} mode={mode} n={n}",
            "CBPROF caps samples=128 work=5 tasks=6",
            "CBPROF mapmask=31 irq_included=1", f"CBPROF end id={job_id}"]


def profile_sample(index=0, job_id=7, work=3, flags=0, wrapped=False):
    start = (4294967290 if wrapped else 1000) + index * 20
    start &= bench.UINT32_MAX
    planned = (start - 1) & bench.UINT32_MAX
    start_dwt = 4294900000 if wrapped else 1000000 + index * 3360000
    window = 168100
    rows = [f"CBPROF S {job_id},{index},{planned},{start},{(start+1)&bench.UINT32_MAX}",
            f"CBPROF H {job_id},{index},5000,5001,1",
            f"CBPROF D {job_id},{index},{start_dwt},{(start_dwt+window)&bench.UINT32_MAX},{window}"]
    rows += [f"CBPROF W {job_id},{index},{slot},{168000*(slot+1)//max(work,1)}" for slot in range(work)]
    rows += [f"CBPROF T {job_id},{index},{bucket},{cycles}" for bucket, cycles in
             enumerate((160000, 3000, 2000, 1000, 1000, 1100))]
    return rows + [f"CBPROF E {job_id},{index},{work},{flags}"]


def oled_metadata(job_id=7, enabled=1, skip=0, begun=3, ended=2, skipped=None):
    if skipped is None:
        skipped = ended if skip else 0
    return [f"CBOLED v=1 id={job_id} enabled={enabled} skip={skip}",
            "CBOLED caps samples=128 stages=5", f"CBOLED F {job_id},{begun},{ended},{skipped}",
            f"CBOLED L {job_id},1,2", f"CBOLED M {job_id},1,4000000", f"CBOLED end id={job_id}"]


def oled_metadata_v2(job_id=7, enabled=1, mode=2, begun=16, ended=16,
                     render_begun=None, rate_limited=None, min_gap=None, max_gap=None):
    if not enabled:
        mode, begun, ended = 0, 0, 0
    if render_begun is None:
        render_begun = begun if mode == 0 else 0 if mode == 1 else min(3, begun)
    if rate_limited is None:
        rate_limited = ended - min(render_begun, ended) if mode == 2 else 0
    if min_gap is None:
        min_gap = (100 if mode == 2 else 20) if render_begun >= 2 else 0
    if max_gap is None:
        max_gap = min_gap
    skipped = ended if mode == 1 else rate_limited if mode == 2 else 0
    return [f"CBOLED v=2 id={job_id} enabled={enabled} skip={int(enabled and mode == 1)} mode={mode}",
            "CBOLED caps samples=128 stages=5",
            f"CBOLED rate period_ticks={100 if enabled and mode == 2 else 0} tick_hz=1000",
            f"CBOLED F {job_id},{begun},{ended},{skipped}",
            f"CBOLED L {job_id},0,0", f"CBOLED M {job_id},0,0",
            f"CBOLED R {job_id},{render_begun},{rate_limited}",
            f"CBOLED G {job_id},{min_gap},{max_gap}", f"CBOLED end id={job_id}"]


def oled_sample(index=0, job_id=7, flags=0):
    residence = (500, 300, 600, 400, 200)
    elapsed = (5000, 3000, 60000, 40000, 60100)
    return ([f"CBOLED P {job_id},{index},{stage},{cycles}" for stage, cycles in enumerate(residence)]
            + [f"CBOLED W {job_id},{index},{stage},{cycles}" for stage, cycles in enumerate(elapsed)]
            + [f"CBOLED E {job_id},{index},{flags}"])


REAL_CAPTURES = (
    ("20261016T031427227642Z", "166a8f61bd3a64c104bc97ed0bd818090b9a88f01384e5c94152948f09327a72", 58582),
    ("20261016T031443598167Z", "789e855e1ff2cc10018e6b6f741c03695d3790e8c0a691a44a18070477ccc9c7", 58583),
)


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
    def test_own_version_end_finishes_without_a_nonexistent_outer_end(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        for line in native_version()[:-1]:
            self.assertFalse(boundary.feed(line))
        self.assertTrue(boundary.feed(native_version()[-1]))

    def test_missing_field_at_own_end_is_immediately_rejected(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        for line in native_version()[:-2]:
            self.assertFalse(boundary.feed(line))
        with self.assertRaises(bench.ProtocolError):
            boundary.feed(native_version()[-1])

    def test_other_request_or_node_cannot_complete_current_version(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        self.assertFalse(boundary.feed(native_version()[0]))
        for line in native_version(122)[1:]:
            self.assertFalse(boundary.feed(line))
        self.assertFalse(boundary.feed("FW_VERSION [1] req=123 end"))
        for line in native_version()[1:-1]:
            self.assertFalse(boundary.feed(line))
        self.assertTrue(boundary.feed(native_version()[-1]))

    def test_early_async_version_fields_are_matched_after_pending_announces_request(self):
        rows = native_version()
        orders = [rows[1:3] + rows[:1] + rows[3:], rows[1:] + rows[:1]]
        for order in orders:
            with self.subTest(pending_at=order.index(rows[0])):
                boundary = bench.ResponseBoundary("#FW_VERSION 0")
                for line in order[:-1]:
                    self.assertFalse(boundary.feed(line))
                self.assertTrue(boundary.feed(order[-1]))

    def test_duplicate_and_malformed_matching_version_fields_are_errors(self):
        for lines in [[native_version()[0], native_version()[1], native_version()[1]],
                      [native_version()[0], "FW_VERSION [0] req=123 epoch=not_a_number"]]:
            boundary = bench.ResponseBoundary("#FW_VERSION 0")
            with self.assertRaises(bench.ProtocolError):
                for line in lines:
                    boundary.feed(line)

    def test_version_error_is_immediate_without_any_end(self):
        boundary = bench.ResponseBoundary("#FW_VERSION 0")
        with self.assertRaises(bench.ProtocolError):
            boundary.feed("error FW_VERSION J0 req=0 busy")
        self.assertEqual(boundary.error, "error FW_VERSION J0 req=0 busy")

    def test_two_real_usb_captures_are_byte_frozen_and_complete_on_own_end(self):
        fixture_root = Path(__file__).parent / "fixtures/cartesian_benchmark"
        for stamp, expected_hash, request in REAL_CAPTURES:
            with self.subTest(stamp=stamp):
                fixture = fixture_root / f"usb_fw_version_timeout_{stamp}.jsonl"
                frozen = fixture.read_bytes()
                self.assertEqual(hashlib.sha256(frozen).hexdigest(), expected_hash)
                original = bench.REPO / f"firmware_artifacts/cartesian-benchmark-{stamp}/raw.jsonl"
                if original.exists():
                    self.assertEqual(original.read_bytes(), frozen)
                events = [json.loads(row) for row in frozen.decode("utf-8").splitlines()]
                lines = [event["line"] for event in events if event["direction"] == "rx"]
                bytes_received = b"".join(bytes.fromhex(event["hex"]) for event in events if event["direction"] == "rx_chunk")
                self.assertEqual(bytes_received.decode("ascii").splitlines(), lines)
                self.assertNotIn("end", lines)
                self.assertEqual(len(lines), 7)
                self.assertEqual(lines[-1], f"FW_VERSION [0] req={request} end")
                self.assertEqual([event["command"] for event in events if event["direction"] == "tx"], ["#FW_VERSION 0"])
                boundary = bench.ResponseBoundary("#FW_VERSION 0")
                self.assertEqual([boundary.feed(line) for line in lines], [False] * 6 + [True])


class CommandBoundaryTests(unittest.TestCase):
    def test_all_fixed_command_responses_finish_on_actual_last_line(self):
        examples = [("#KIN_CONFIG", native_kin()), ("#TOOL", native_tool()), ("#STACK", native_stack()),
                    ("#MOTION_STATUS", native_motion()), ("#MOTION_STATUS", native_motion(optional=True)),
                    ("#CBENCH", native_status()), ("#CBENCH RUN 2,0,3,2,20000", native_status(phase="PENDING", n=0, hz=0)),
                    ("#CBENCH CANCEL", native_status(phase="RUNNING", n=0, hz=0))]
        for command, rows in examples:
            with self.subTest(command=command, optional=len(rows)):
                self.assertNotIn("end", rows)
                boundary = bench.ResponseBoundary(command)
                for line in rows[:-1]:
                    self.assertFalse(boundary.feed(line), line)
                self.assertTrue(boundary.feed(rows[-1]))

    def test_background_lines_do_not_count_as_response_fields(self):
        boundary = bench.ResponseBoundary("#STACK")
        for line in ["boot complete", "end", "MOTION id=123 kind=MOVEJ phase=0 mode=2 ms=0", "RGB state=1"]:
            self.assertFalse(boundary.feed(line))
        for line in native_stack()[:-1]:
            self.assertFalse(boundary.feed(line))
        self.assertTrue(boundary.feed(native_stack()[-1]))

    def test_terminator_with_missing_required_records_cannot_hide_truncation(self):
        examples = [("#KIN_CONFIG", native_kin()), ("#TOOL", native_tool()), ("#STACK", native_stack()),
                    ("#MOTION_STATUS", native_motion()), ("#CBENCH", native_status())]
        for command, rows in examples:
            with self.subTest(command=command):
                boundary = bench.ResponseBoundary(command)
                with self.assertRaises(bench.ProtocolError):
                    for line in rows[1:]:
                        boundary.feed(line)

    def test_missing_interior_record_is_rejected_at_final_record(self):
        examples = [("#KIN_CONFIG", native_kin()), ("#TOOL", native_tool()), ("#STACK", native_stack()),
                    ("#MOTION_STATUS", native_motion()), ("#CBENCH", native_status())]
        for command, rows in examples:
            with self.subTest(command=command):
                boundary = bench.ResponseBoundary(command)
                for line in rows[:1] + rows[2:-1]:
                    self.assertFalse(boundary.feed(line))
                with self.assertRaises(bench.ProtocolError):
                    boundary.feed(rows[-1])

    def test_duplicate_required_rows_and_bad_matching_prefix_are_errors(self):
        for command, line in [("#KIN_CONFIG", native_kin()[0]), ("#TOOL", native_tool()[0]),
                              ("#STACK", native_stack()[0]), ("#MOTION_STATUS", native_motion()[0]),
                              ("#CBENCH", native_status()[0])]:
            with self.subTest(command=command):
                boundary = bench.ResponseBoundary(command)
                boundary.feed(line)
                with self.assertRaises(bench.ProtocolError):
                    boundary.feed(line)
        for command, line in [("#KIN_CONFIG", "KIN broken"), ("#TOOL", "TOOL ABC bad"),
                              ("#STACK", "ok STACK invalid"), ("#MOTION_STATUS", "MOTION invalid"),
                              ("#CBENCH", "CBENCH clock_hz=bad budget_us=20000 n=0")]:
            with self.subTest(command=command):
                with self.assertRaises(bench.ProtocolError):
                    bench.ResponseBoundary(command).feed(line)

    def test_error_response_aborts_every_query_without_waiting_for_more_lines(self):
        for command in bench.QUERIES:
            with self.subTest(command=command):
                boundary = bench.ResponseBoundary(command)
                with self.assertRaises(bench.ProtocolError):
                    boundary.feed("error response too long")
                self.assertEqual(boundary.error, "error response too long")

    def test_read_requires_expected_id_and_all_requested_pairs(self):
        with self.assertRaises(bench.ProtocolError):
            bench.ResponseBoundary("#CBENCH READ 0,2")
        boundary = bench.ResponseBoundary("#CBENCH READ 120,8", expected_id=4294967295)
        rows = csv_samples(job_id=4294967295, first=120, count=8)
        for line in rows[:-1]:
            self.assertFalse(boundary.feed(line))
        self.assertTrue(boundary.feed(rows[-1]))

    def test_read_rejects_wrong_id_index_duplicate_and_missing_pair(self):
        cases = [["CBENCH DATA 8,0,1,1,3,0"], ["CBENCH DATA 7,2,1,1,3,0"],
                 [csv_samples()[0], csv_samples()[0]], [csv_samples()[0], csv_samples()[2], csv_samples()[3]]]
        for rows in cases:
            with self.subTest(rows=rows):
                boundary = bench.ResponseBoundary("#CBENCH READ 0,2", expected_id=7)
                with self.assertRaises(bench.ProtocolError):
                    for line in rows:
                        boundary.feed(line)

    def test_known_status_and_cancel_require_the_expected_job_id(self):
        for command in ["#CBENCH", "#CBENCH CANCEL"]:
            with self.subTest(command=command):
                boundary = bench.ResponseBoundary(command, expected_id=7)
                with self.assertRaises(bench.ProtocolError):
                    boundary.feed(native_status(job_id=8)[0])

    def test_run_requires_new_id_and_echoed_configuration_and_budget(self):
        command = "#CBENCH RUN 2,0,3,2,20000"
        boundary = bench.ResponseBoundary(command, expected_id=7)
        with self.assertRaises(bench.ProtocolError):
            boundary.feed(native_status(job_id=7, phase="PENDING")[0])
        for bad_config in ["CBENCH config op=3 fixture=0 work=3 samples=2",
                           "CBENCH config op=2 fixture=1 work=3 samples=2",
                           "CBENCH config op=2 fixture=0 work=4 samples=2",
                           "CBENCH config op=2 fixture=0 work=3 samples=3"]:
            with self.subTest(config=bad_config):
                boundary = bench.ResponseBoundary(command, expected_id=7)
                boundary.feed(native_status(job_id=8, phase="PENDING")[0])
                with self.assertRaises(bench.ProtocolError):
                    boundary.feed(bad_config)
        boundary = bench.ResponseBoundary(command, expected_id=7)
        for line in native_status(job_id=8, phase="PENDING")[:2]:
            boundary.feed(line)
        with self.assertRaises(bench.ProtocolError):
            boundary.feed("CBENCH clock_hz=0 budget_us=19999 n=0")


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
                    lines = native_version(12)
                elif command == "#KIN_CONFIG":
                    lines = native_kin()
                elif command == "#TOOL":
                    lines = native_tool()
                elif command == "#MOTION_STATUS":
                    lines = native_motion(optional=True)
                elif command == "#STACK":
                    lines = native_stack()
                elif command == "#CBENCH":
                    lines = native_status(job_id=1) if self.started else native_status(job_id=0, phase="IDLE", n=0, hz=0)
                elif command == "#CBENCH RUN 2,0,3,2,20000":
                    self.started = True
                    lines = native_status(job_id=1, phase="PENDING", n=0, hz=0)
                elif command == "#CBENCH READ 0,2":
                    lines = csv_samples(job_id=1)
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
            rows = native_status(job_id=0, phase="IDLE", n=0, hz=0)
            payload = ("\r\n".join(rows) + "\r\n").encode("ascii")
            port = FakePort([payload[:4], payload[4:17], payload[17:]])
            session = bench.SafeSession(port, recorder, .1, bench.CaptureParser())
            self.assertEqual(session.request("#CBENCH"), rows)
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

    def test_same_chunk_background_tail_is_logged_and_does_not_block_next_query(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "raw.jsonl"
            recorder = bench.Recorder(path)
            stack_packet = ("\r\n".join(native_stack() + ["BACKGROUND rgb idle"]) + "\r\n").encode("ascii")
            tool_packet = ("\r\n".join(native_tool()) + "\r\n").encode("ascii")
            port = FakePort([stack_packet, tool_packet])
            session = bench.SafeSession(port, recorder, .1, bench.CaptureParser())
            session.request("#STACK")
            session.request("#TOOL")
            recorder.close()
            self.assertEqual(port.writes, [b"#STACK\n", b"#TOOL\n"])
            self.assertIn("BACKGROUND rgb idle", path.read_text(encoding="utf-8"))

    def test_split_chunk_background_is_ignored_between_real_response_records(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            rows = native_stack()
            chunks = [(rows[0] + "\r\nBACK").encode("ascii"),
                      ("GROUND rgb idle\r\n" + rows[1] + "\r\n").encode("ascii"),
                      (rows[2] + "\r\n").encode("ascii")]
            port = FakePort(chunks)
            session = bench.SafeSession(port, recorder, .1, bench.CaptureParser())
            result = session.request("#STACK")
            recorder.close()
            self.assertIn(rows[-1], result)
            self.assertEqual(port.writes, [b"#STACK\n"])

    def test_extra_related_record_in_same_chunk_is_not_silently_discarded(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            payload = ("\r\n".join(native_stack() + [native_stack()[-1]]) + "\r\n").encode("ascii")
            port = FakePort([payload])
            session = bench.SafeSession(port, recorder, .1, bench.CaptureParser())
            with self.assertRaises(bench.ProtocolError):
                session.request("#STACK")
            recorder.close()
            self.assertEqual(port.writes, [b"#STACK\n"])

    def test_read_session_binds_expected_id_from_current_parser_job(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            parser = bench.CaptureParser()
            for line in native_status():
                parser.feed(line)
            payload = ("\r\n".join(csv_samples()) + "\r\n").encode("ascii")
            port = FakePort([payload])
            session = bench.SafeSession(port, recorder, .1, parser)
            result = session.request("#CBENCH READ 0,2")
            recorder.close()
            self.assertEqual(result, csv_samples())
            self.assertEqual(port.writes, [b"#CBENCH READ 0,2\n"])

    def test_unrelated_cbench_background_does_not_pollute_parser_current_job(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "raw.jsonl"
            recorder = bench.Recorder(path)
            parser = bench.CaptureParser()
            for line in native_status(job_id=7):
                parser.feed(line)
            rows = native_stack()[:1] + native_status(job_id=99, phase="IDLE", n=0, hz=0) + native_stack()[1:]
            payload = ("\r\n".join(rows) + "\r\n").encode("ascii")
            session = bench.SafeSession(FakePort([payload]), recorder, .1, parser)
            session.request("#STACK")
            recorder.close()
            self.assertEqual(parser.current_id, 7)
            self.assertNotIn(99, parser.jobs)
            self.assertIn("CBENCH v=1 id=99 phase=IDLE", path.read_text(encoding="utf-8"))

    def test_rejected_status_id_does_not_mutate_current_parser_job(self):
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary) / "raw.jsonl")
            parser = bench.CaptureParser()
            for line in native_status(job_id=7):
                parser.feed(line)
            payload = ("\r\n".join(native_status(job_id=99)) + "\r\n").encode("ascii")
            session = bench.SafeSession(FakePort([payload]), recorder, .1, parser)
            with self.assertRaises(bench.ProtocolError):
                session.request("#CBENCH")
            recorder.close()
            self.assertEqual(parser.current_id, 7)
            self.assertNotIn(99, parser.jobs)

    def test_partial_last_line_times_out_without_cancel_or_truncated_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "raw.jsonl"
            recorder = bench.Recorder(path)
            rows = native_stack()
            payload = ("\r\n".join(rows[:2]) + "\r\n" + rows[-1][:-3]).encode("ascii")
            port = FakePort([payload])
            session = bench.SafeSession(port, recorder, .005, bench.CaptureParser())
            with self.assertRaises(TimeoutError):
                session.request("#STACK")
            recorder.close()
            self.assertEqual(port.writes, [b"#STACK\n"])
            events = [json.loads(row) for row in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(b"".join(bytes.fromhex(event["hex"]) for event in events if event["direction"] == "rx_chunk"), payload)

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


class ProfileTests(unittest.TestCase):
    def base(self, count=2, work=3, phase="DONE", flag=0):
        parser = bench.CaptureParser()
        for line in metadata(n=count, samples=count, work=work, phase=phase):
            parser.feed(line)
        for index in range(count):
            for line in sample(index, work=work, flag=flag):
                parser.feed(line)
        return parser

    def boundary(self, count=2, work=3):
        parser = self.base(count, work)
        return bench.ResponseBoundary(f"#CBENCH PREAD 0,{count}", expected_id=7,
                                      base_data=parser.jobs[7]["data"], clock_hz=168000000)

    def test_profile_allowlist_is_opt_in_and_bounded(self):
        for command in ("#CBENCH PROFILE", "#CBENCH RUN 3,0,5,128,20000,1", "#CBENCH PREAD 124,4"):
            self.assertTrue(bench.command_allowed(command), command)
        for command in ("#CBENCH RUN 3,0,6,1,20000,1", "#CBENCH RUN 3,0,1,1,20000,0",
                        "#CBENCH RUN 3,0,1,1,20000,2", "#CBENCH PREAD 0,5", "#CBENCH PREAD 127,2",
                        "#CBENCH PROFILE\n!START", "#CBENCH PREAD -1,1"):
            self.assertFalse(bench.command_allowed(command), command)
        self.assertNotIn("#CBENCH PROFILE", bench.QUERIES)

    def test_profile_metadata_requires_all_four_lines_and_matching_id(self):
        boundary = bench.ResponseBoundary("#CBENCH PROFILE", expected_id=7)
        self.assertEqual([boundary.feed(line) for line in profile_metadata()], [False, False, False, True])
        for rows in (profile_metadata()[1:], profile_metadata()[:1] + profile_metadata()[2:],
                     profile_metadata(job_id=8), profile_metadata()[:2] + ["CBPROF mapmask=31 irq_included=0"]):
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                boundary = bench.ResponseBoundary("#CBENCH PROFILE", expected_id=7)
                for line in rows:
                    boundary.feed(line)

    def test_pread_completes_only_on_last_requested_sample_E(self):
        boundary = self.boundary()
        lines = profile_sample(0) + profile_sample(1)
        self.assertEqual([boundary.feed(line) for line in lines], [False] * (len(lines) - 1) + [True])

    def test_pread_missing_duplicate_wrong_id_slot_and_uint_fail_closed(self):
        valid = profile_sample()
        cases = [valid[1:], valid[:3] + valid[4:], valid[:6] + valid[7:],
                 valid[:1] + valid, profile_sample(job_id=8),
                 [line.replace("W 7,0,0,", "W 7,0,3,") for line in valid],
                 [line.replace("T 7,0,0,", "T 7,0,6,") for line in valid],
                 [line.replace("D 7,0,1000000,", "D 7,0,4294967296,") for line in valid],
                 [line.replace("E 7,0,3,0", "E 7,0,2,0") for line in valid],
                 [line.replace("T 7,0,5,1100", "T 7,0,5,1101") for line in valid],
                 [line.replace("H 7,0,5000,5001,1", "H 7,0,5000,5001,2") for line in valid],
                 [line.replace("D 7,0,1000000,1168100,168100", "D 7,0,1000000,1168101,168100") for line in valid],
                 [line.replace("W 7,0,1,112000", "W 7,0,1,55000") for line in valid],
                 [line.replace("W 7,0,2,168000", "W 7,0,2,168001") for line in valid],
                 [line.replace("S 7,0,999,1000,1001", "S 7,0,999,1000,11001") for line in valid],
                 [line.replace("H 7,0,5000,5001,1", "H 7,0,5000,15001,1") for line in valid]]
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                boundary = self.boundary(count=1)
                for line in rows:
                    boundary.feed(line)

    def test_profile_unsigned_wrap_and_conservation_are_supported(self):
        boundary = self.boundary(count=1)
        rows = profile_sample(wrapped=True)
        self.assertTrue([boundary.feed(line) for line in rows][-1])
        boundary = self.boundary(count=1)
        rows[0] = "CBPROF S 7,0,4294967295,0,1"
        self.assertTrue([boundary.feed(line) for line in rows][-1])

    def test_profile_maximum_wire_lines_fit_existing_response_buffer(self):
        for line in ("CBPROF v=1 id=4294967295 mode=1 n=128", "CBPROF mapmask=31 irq_included=1",
                     "CBPROF S 4294967295,127,4294967295,4294967295,4294967295",
                     "CBPROF D 4294967295,127,4294967295,4294967295,4294967295",
                     "CBPROF E 4294967295,127,5,4294967295"):
            self.assertLess(len(line.encode("ascii")), 62)

    def test_profile_offline_summary_keeps_kernel_and_residence_separate(self):
        parser = self.base()
        for line in profile_metadata() + profile_sample(0) + profile_sample(1):
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["job_success"])
        profile = summary["profile"]
        self.assertTrue(profile["accounting_valid"])
        self.assertEqual(profile["sample_count"], 2)
        self.assertEqual(profile["late_ticks"], {"count": 2, "max": 1, "late_samples": 2})
        self.assertEqual(summary["timing_all_observed"]["p50_us"], 1000)
        self.assertAlmostEqual(profile["window_minus_kernel"]["p50_us"], 100/168)
        self.assertEqual(profile["work_intervals"]["0"]["p50_us"], 56000/168)
        self.assertIn("not exclusive", profile["note"])

    def test_profile_nonzero_flags_are_visible_without_relabeling_kernel_failure(self):
        parser = self.base(count=1)
        for line in profile_metadata(n=1) + profile_sample(flags=4):
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["job_success"])
        self.assertTrue(summary["profile"]["records_complete"])
        self.assertFalse(summary["profile"]["accounting_valid"])
        self.assertEqual(summary["profile"]["nonzero_flags"], [{"i": 0, "flags": 4}])

    def test_profile_budget_after_all_work_and_partial_work_remain_distinct(self):
        for work in (2, 3):
            parser = self.base(count=1, work=3, phase="BUDGET", flag=1)
            parser.jobs[7]["data"][0]["work"] = work
            for line in profile_metadata(n=1) + profile_sample(work=work):
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertTrue(summary["collection_complete"])
            self.assertFalse(summary["job_success"])
            self.assertTrue(summary["profile"]["accounting_valid"])

    def test_offline_incomplete_or_malformed_profile_does_not_pass(self):
        for extra in (profile_metadata() + profile_sample(0),
                      profile_metadata() + profile_sample(0)[:-1] + profile_sample(1),
                      profile_metadata() + profile_sample(0) + profile_sample(1, job_id=8),
                      profile_metadata()[:2], ["CBPROF malformed"]):
            parser = self.base()
            for line in extra:
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertFalse(summary["collection_complete"])

    def test_pread_context_rejected_before_any_serial_write(self):
        for mode, base_present, phase in ((0, True, "DONE"), (1, False, "DONE"), (1, True, "RUNNING")):
            parser = self.base(count=1, phase=phase)
            if not base_present:
                parser.jobs[7]["data"].clear()
            for line in profile_metadata(mode=mode, n=1):
                parser.feed(line)
            with tempfile.TemporaryDirectory() as temporary:
                recorder = bench.Recorder(Path(temporary)/"raw.jsonl")
                port = FakePort([])
                session = bench.SafeSession(port, recorder, .01, parser)
                with self.assertRaises(bench.ProtocolError):
                    session.request("#CBENCH PREAD 0,1")
                recorder.close()
                self.assertEqual(port.writes, [])

    def test_eight_previous_real_logs_replay_without_changing_originals(self):
        stamps = ["031427227642", "031443598167", "033714146669", "033723804593",
                  "034331525527", "034340250808", "035324572035", "035329868626"]
        expected = [None, None, "IDLE", "DONE", "DONE", "BUDGET", "DONE", "BUDGET"]
        for stamp, phase in zip(stamps, expected):
            capture = bench.REPO/f"firmware_artifacts/cartesian-benchmark-20261016T{stamp}Z/raw.jsonl"
            if not capture.exists():
                self.skipTest("Host-specific real capture directories are unavailable")
            original = capture.read_bytes()
            parser = bench.CaptureParser()
            for line in bench.capture_lines(capture):
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertEqual(summary.get("phase"), phase)
            self.assertEqual(capture.read_bytes(), original)
            if phase is not None:
                self.assertTrue(summary["collection_complete"])

    def test_full_fake_usb_profile_flow_and_offline_export_without_bare_end(self):
        class FakeProfileUSB(FakePort):
            def __init__(self, supported=True):
                super().__init__([])
                self.started, self.opened, self.supported = False, False, supported

            def open(self):
                self.opened = True

            def close(self):
                self.opened = False

            def write(self, encoded):
                self.writes.append(encoded)
                command = encoded.decode("ascii").strip()
                fixed = {"#FW_VERSION 0": native_version(), "#KIN_CONFIG": native_kin(),
                         "#TOOL": native_tool(), "#MOTION_STATUS": native_motion(), "#STACK": native_stack()}
                if command in fixed:
                    lines = fixed[command]
                elif command == "#CBENCH":
                    lines = native_status(job_id=1) if self.started else native_status(job_id=0, phase="IDLE", n=0, hz=0)
                elif command == "#CBENCH PROFILE":
                    lines = (profile_metadata(job_id=1) if self.started else profile_metadata(job_id=0, mode=0, n=0)) if self.supported else ["error CBENCH syntax"]
                elif command == "#CBENCH RUN 2,0,3,2,20000,1":
                    self.started = True
                    lines = native_status(job_id=1, phase="PENDING", n=0, hz=0)
                elif command == "#CBENCH READ 0,2":
                    lines = csv_samples(job_id=1)
                elif command == "#CBENCH PREAD 0,2":
                    lines = profile_sample(0, job_id=1) + profile_sample(1, job_id=1)
                else:
                    raise AssertionError(f"Unexpected command {command}")
                self.chunks.append(("\r\n".join(lines) + "\r\n").encode("ascii"))
                return len(encoded)

        for supported in (True, False):
            port = FakeProfileUSB(supported)
            fake_serial = types.SimpleNamespace(Serial=lambda **kwargs: port, __version__="offline_fake")
            with tempfile.TemporaryDirectory() as temporary, self.subTest(supported=supported):
                with mock.patch.dict("sys.modules", {"serial": fake_serial}), contextlib.redirect_stdout(io.StringIO()) as printed, contextlib.redirect_stderr(io.StringIO()):
                    code = bench.main(["--port", "OFFLINE_FAKE", "--run", "--profile", "--work", "3", "--samples", "2",
                                       "--poll-ms", "20", "--output-root", temporary])
                output = Path(printed.getvalue().strip())
                self.assertFalse(port.opened)
                commands = [command.decode("ascii").strip() for command in port.writes]
                self.assertTrue(all(bench.command_allowed(command) for command in commands))
                if not supported:
                    self.assertEqual(code, 2)
                    self.assertFalse(port.started)
                    self.assertFalse(any(command.startswith("#CBENCH RUN") for command in commands))
                    continue
                self.assertEqual(code, 0)
                self.assertEqual(commands.count("#CBENCH PROFILE"), 2)
                self.assertNotIn("#CBENCH OLED", commands)
                self.assertLess(commands.index("#CBENCH PROFILE"), commands.index("#CBENCH RUN 2,0,3,2,20000,1"))
                self.assertLess(commands.index("#CBENCH READ 0,2"), commands.index("#CBENCH PREAD 0,2"))
                summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
                self.assertTrue(summary["job_success"])
                self.assertTrue(summary["profile"]["accounting_valid"])
                self.assertEqual(summary["profile"]["sample_count"], 2)
                self.assertIn("window_minus_kernel_cycles", (output/"profile.csv").read_text(encoding="utf-8"))
                self.assertTrue((output/"collector_source.py").exists())
                written = list(port.writes)
                with contextlib.redirect_stdout(io.StringIO()) as parsed_printed:
                    self.assertEqual(bench.main(["--parse", str(output/"raw.jsonl"), "--output-root", temporary]), 0)
                parsed_summary = json.loads((Path(parsed_printed.getvalue().strip())/"summary.json").read_text(encoding="utf-8"))
                self.assertEqual(parsed_summary["profile"], summary["profile"])
                self.assertEqual(port.writes, written)

    def test_profile_cli_rejects_missing_run_and_large_work_before_serial_import(self):
        for arguments in (["--profile"], ["--port", "OFFLINE_FAKE", "--run", "--profile", "--work", "6"]):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                bench.main(arguments)
            self.assertEqual(exc.exception.code, 2)

    def test_pread_extra_index_duplicate_task_and_overflowing_sum_fail(self):
        rows = profile_sample()
        cases = [[line.replace(",0,", ",1,", 1) for line in rows],
                 rows[:7] + [rows[6]] + rows[7:],
                 [line.replace("T 7,0,0,160000", "T 7,0,0,4294967295") for line in rows],
                 [line.replace("E 7,0,3,0", "E 7,0,3,4294967296") for line in rows]]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(bench.ProtocolError):
                boundary = self.boundary(count=1)
                for line in case:
                    boundary.feed(line)

    def test_profile_pretransmit_rejects_invalid_known_work_or_cycles(self):
        for key, value in (("work", 6), ("cycles", 4294967296)):
            parser = self.base(count=1)
            for line in profile_metadata(n=1):
                parser.feed(line)
            parser.jobs[7]["data"][0][key] = value
            with tempfile.TemporaryDirectory() as temporary:
                recorder = bench.Recorder(Path(temporary)/"raw.jsonl")
                port = FakePort([])
                with self.assertRaises(bench.ProtocolError):
                    bench.SafeSession(port, recorder, .01, parser).request("#CBENCH PREAD 0,1")
                recorder.close()
                self.assertEqual(port.writes, [])

    def test_profile_metadata_mapmask_is_limited_to_five_known_task_bits(self):
        for mapmask in (32, 63, 4294967295):
            rows = [line.replace("mapmask=31", f"mapmask={mapmask}") for line in profile_metadata()]
            with self.subTest(mapmask=mapmask), self.assertRaises(bench.ProtocolError):
                boundary = bench.ResponseBoundary("#CBENCH PROFILE", expected_id=7)
                for line in rows:
                    boundary.feed(line)

    def test_incomplete_mapping_retains_records_but_cannot_validate_accounting(self):
        for mapmask in (0, 1, 16, 30):
            parser = self.base(count=1)
            lines = [line.replace("mapmask=31", f"mapmask={mapmask}") for line in profile_metadata(n=1)]
            for line in lines + profile_sample():
                parser.feed(line)
            summary, _ = parser.summarize()
            with self.subTest(mapmask=mapmask):
                self.assertTrue(summary["collection_complete"])
                self.assertTrue(summary["job_success"])
                self.assertTrue(summary["profile"]["records_complete"])
                self.assertFalse(summary["profile"]["accounting_valid"])
                self.assertEqual(summary["profile"]["sample_count"], 1)
                self.assertFalse(parser.profile_summary(parser.jobs[7])[1][0]["accounting_valid"])

    def test_zero_sample_profile_cancellation_is_complete_but_not_valid_accounting(self):
        for mapmask in (0, 31):
            parser = bench.CaptureParser()
            lines = metadata(phase="CANCELLED", n=0, samples=16, work=1, hz=0)
            lines += [line.replace("mapmask=31", f"mapmask={mapmask}") for line in profile_metadata(n=0)]
            for line in lines:
                parser.feed(line)
            summary, _ = parser.summarize()
            with self.subTest(mapmask=mapmask):
                self.assertTrue(summary["collection_complete"])
                self.assertFalse(summary["job_success"])
                self.assertTrue(summary["profile"]["records_complete"])
                self.assertFalse(summary["profile"]["accounting_valid"])
                self.assertEqual(summary["profile"]["sample_count"], 0)


class OledTests(unittest.TestCase):
    class FakeOledUSB(FakePort):
        def __init__(self, mode="normal", supported=True, terminal_skip=None, version=1, final_mode=None):
            super().__init__([])
            self.mode, self.supported, self.terminal_skip = mode, supported, terminal_skip
            self.version, self.final_mode = version, final_mode
            self.started, self.opened = False, False

        def open(self):
            self.opened = True

        def close(self):
            self.opened = False

        def write(self, encoded):
            self.writes.append(encoded)
            command = encoded.decode("ascii").strip()
            fixed = {"#FW_VERSION 0": native_version(), "#KIN_CONFIG": native_kin(),
                     "#TOOL": native_tool(), "#MOTION_STATUS": native_motion(), "#STACK": native_stack()}
            if command in fixed:
                lines = fixed[command]
            elif command == "#CBENCH":
                lines = native_status(job_id=1) if self.started else native_status(job_id=0, phase="IDLE", n=0, hz=0)
            elif command == "#CBENCH PROFILE":
                lines = profile_metadata(job_id=1) if self.started else profile_metadata(job_id=0, mode=0, n=0)
            elif command == "#CBENCH OLED":
                if not self.supported:
                    lines = ["error CBENCH syntax"]
                elif self.version == 2:
                    lines = oled_metadata_v2(job_id=int(self.started), enabled=int(self.started),
                                             mode=bench.OLED_MODES[self.mode] if self.final_mode is None else self.final_mode)
                elif not self.started:
                    lines = oled_metadata(job_id=0, enabled=0, begun=0, ended=0)
                    lines[3:5] = ["CBOLED L 0,0,0", "CBOLED M 0,0,0"]
                else:
                    skip = int(self.mode == "skip-render") if self.terminal_skip is None else self.terminal_skip
                    lines = oled_metadata(job_id=1, skip=skip)
            elif command == f"#CBENCH RUN 2,0,3,2,20000,1,{bench.OLED_MODES[self.mode]}":
                self.started = True
                lines = native_status(job_id=1, phase="PENDING", n=0, hz=0)
            elif command == "#CBENCH READ 0,2":
                lines = csv_samples(job_id=1)
            elif command == "#CBENCH PREAD 0,2":
                lines = profile_sample(0, job_id=1) + profile_sample(1, job_id=1)
            elif command == "#CBENCH OREAD 0,2":
                lines = oled_sample(0, job_id=1) + oled_sample(1, job_id=1)
            else:
                raise AssertionError(f"Unexpected command {command}")
            self.chunks.append(("\r\n".join(lines) + "\r\n").encode("ascii"))
            return len(encoded)

    def fake_capture(self, port, mode, root):
        fake_serial = types.SimpleNamespace(Serial=lambda **kwargs: port, __version__="offline_fake")
        with mock.patch.dict("sys.modules", {"serial": fake_serial}), contextlib.redirect_stdout(io.StringIO()) as printed, contextlib.redirect_stderr(io.StringIO()):
            code = bench.main(["--port", "OFFLINE_FAKE", "--run", "--profile", "--oled-mode", mode,
                               "--work", "3", "--samples", "2", "--poll-ms", "20", "--output-root", str(root)])
        return code, Path(printed.getvalue().strip())

    def test_full_fake_usb_normal_and_skip_capture_and_offline_export(self):
        for mode in ("normal", "skip-render"):
            with tempfile.TemporaryDirectory() as temporary, self.subTest(mode=mode):
                port = self.FakeOledUSB(mode=mode)
                code, output = self.fake_capture(port, mode, temporary)
                self.assertEqual(code, 0)
                self.assertFalse(port.opened)
                commands = [encoded.decode("ascii").strip() for encoded in port.writes]
                run = f"#CBENCH RUN 2,0,3,2,20000,1,{int(mode == 'skip-render')}"
                self.assertEqual(commands.count("#CBENCH OLED"), 2)
                self.assertLess(commands.index("#CBENCH OLED"), commands.index(run))
                self.assertLess(commands.index("#CBENCH PREAD 0,2"), commands.index("#CBENCH OREAD 0,2"))
                self.assertTrue(all(bench.command_allowed(command) for command in commands))
                summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
                self.assertTrue(summary["job_success"])
                self.assertTrue(summary["oled"]["accounting_valid"])
                self.assertTrue(summary["oled"]["application_observed"])
                self.assertEqual(summary["oled"]["requested_mode"], mode)
                self.assertEqual(summary["session"]["oled_requested_mode"], mode)
                self.assertEqual(len((output/"oled.csv").read_text(encoding="utf-8").splitlines()), 3)
                events = [json.loads(line) for line in (output/"raw.jsonl").read_text(encoding="utf-8").splitlines()]
                self.assertEqual(events[0]["oled_requested_mode"], mode)
                before = list(port.writes)
                with contextlib.redirect_stdout(io.StringIO()) as printed:
                    self.assertEqual(bench.main(["--parse", str(output/"raw.jsonl"), "--output-root", temporary]), 0)
                parsed = json.loads((Path(printed.getvalue().strip())/"summary.json").read_text(encoding="utf-8"))
                self.assertTrue(parsed["oled"]["accounting_valid"])
                # Saved wire metadata proves the job mode, but replay does not
                # invent CLI intent from rx-only records.
                self.assertIsNone(parsed["oled"]["requested_mode"])
                self.assertEqual(parsed["oled"]["metadata"], summary["oled"]["metadata"])
                self.assertEqual(port.writes, before)

    def test_unsupported_oled_board_stops_before_RUN(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = self.FakeOledUSB(supported=False)
            code, output = self.fake_capture(port, "normal", temporary)
            self.assertEqual(code, 2)
            self.assertFalse(port.started)
            self.assertFalse(port.opened)
            self.assertFalse(any(encoded.startswith(b"#CBENCH RUN ") for encoded in port.writes))
            self.assertTrue((output/"raw.jsonl").exists())

    def test_mismatched_terminal_oled_mode_does_not_issue_OREAD(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = self.FakeOledUSB(mode="skip-render", terminal_skip=0)
            code, output = self.fake_capture(port, "skip-render", temporary)
            self.assertEqual(code, 2)
            self.assertFalse(any(encoded.startswith(b"#CBENCH OREAD ") for encoded in port.writes))
            summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
            self.assertFalse(summary["oled"]["records_complete"])
            self.assertFalse(summary["oled"]["accounting_valid"])

    def base(self, count=2, phase="DONE", work=3, flag=0, profile_flags=0):
        parser = bench.CaptureParser()
        for line in metadata(n=count, samples=max(count, 1), work=3, phase=phase):
            parser.feed(line)
        for index in range(count):
            for line in sample(index, work=work, flag=flag):
                parser.feed(line)
        for line in profile_metadata(n=count):
            parser.feed(line)
        for index in range(count):
            for line in profile_sample(index, work=work, flags=profile_flags):
                parser.feed(line)
        return parser

    def boundary(self, count=1):
        parser = self.base(count=count)
        return bench.ResponseBoundary(f"#CBENCH OREAD 0,{count}", expected_id=7,
                                      profile_records=parser.jobs[7]["profile_records"].records)

    def test_oled_allowlist_requires_profile_field_and_bounded_reads(self):
        for command in ("#CBENCH OLED", "#CBENCH OREAD 124,4", "#CBENCH RUN 3,0,5,128,20000,1,0",
                        "#CBENCH RUN 3,0,5,128,20000,1,1", "#CBENCH RUN 3,0,5,128,20000,1,2"):
            self.assertTrue(bench.command_allowed(command), command)
        for command in ("#CBENCH RUN 3,0,5,128,20000,0,1", "#CBENCH RUN 3,0,5,128,20000,1,5",
                        "#CBENCH RUN 3,0,5,128,20000,1,1,1", "#CBENCH RUN 3,0,6,128,20000,1,1",
                        "#CBENCH OREAD 0,5", "#CBENCH OREAD 127,2", "#CBENCH OREAD -1,1",
                        "#CBENCH OLED\n!START", "#CBENCH RUN 3,0,5,128,20000,,1"):
            self.assertFalse(bench.command_allowed(command), command)
        self.assertNotIn("#CBENCH OLED", bench.QUERIES)

    def test_oled_metadata_finishes_only_on_own_end(self):
        boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
        self.assertEqual([boundary.feed(line) for line in oled_metadata()], [False] * 5 + [True])
        self.assertTrue(boundary.oled_metadata.values["begun"] > boundary.oled_metadata.values["ended"])
        rows = oled_metadata(job_id=0, enabled=0, begun=0, ended=0)
        rows[3:5] = ["CBOLED L 0,0,0", "CBOLED M 0,0,0"]
        boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=0)
        self.assertEqual([boundary.feed(line) for line in rows], [False] * 5 + [True])

    def test_oled_metadata_rejects_missing_duplicate_wrong_id_caps_and_uint(self):
        valid = oled_metadata()
        cases = [valid[1:], valid[:2] + valid[3:], valid[:2] + [valid[1]] + valid[2:],
                 oled_metadata(job_id=8), [line.replace("stages=5", "stages=6") for line in valid],
                 [line.replace("F 7,3,2,0", "F 7,4294967296,2,0") for line in valid],
                 valid[:-1] + ["CBOLED end id=8"],
                 [line.replace("enabled=1 skip=0", "enabled=0 skip=1") for line in valid]]
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
                for line in rows:
                    boundary.feed(line)

    def test_oled_frame_counter_constraints_preserve_incomplete_frame(self):
        valid = oled_metadata(skip=1)
        cases = [oled_metadata(begun=1, ended=2), oled_metadata(skip=1, skipped=3),
                 oled_metadata(skip=0, skipped=1),
                 [line.replace("L 7,1,2", "L 7,4,2") for line in valid],
                 [line.replace("M 7,1,4000000", "M 7,3,4000000") for line in valid]]
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
                for line in rows:
                    boundary.feed(line)

    def test_oread_finishes_on_last_requested_E(self):
        boundary = self.boundary(count=2)
        rows = oled_sample(0) + oled_sample(1)
        self.assertEqual([boundary.feed(line) for line in rows], [False] * (len(rows) - 1) + [True])

    def test_oread_missing_duplicate_id_index_stage_uint_and_conservation_rejected(self):
        valid = oled_sample()
        cases = [valid[1:], valid[:5] + valid[6:], valid[:1] + valid,
                 oled_sample(job_id=8), oled_sample(index=1),
                 [line.replace("P 7,0,0,500", "P 7,0,5,500") for line in valid],
                 [line.replace("W 7,0,0,5000", "W 7,0,5,5000") for line in valid],
                 [line.replace("P 7,0,0,500", "P 7,0,0,501") for line in valid],
                 [line.replace("W 7,0,0,5000", "W 7,0,0,5001") for line in valid],
                 [line.replace("W 7,0,0,5000", "W 7,0,0,4294967295") for line in valid],
                 [line.replace("E 7,0,0", "E 7,0,4294967296") for line in valid],
                 [line.replace("P 7,0,0,500", "P 7,0,0,-1") for line in valid],
                 [line.replace("P 7,0,0,500", "P 7,0,0,５００") for line in valid]]
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                boundary = self.boundary()
                for line in rows:
                    boundary.feed(line)

    def test_oread_per_stage_residence_must_not_exceed_elapsed(self):
        rows = [line.replace("W 7,0,0,5000", "W 7,0,0,499").replace("W 7,0,4,60100", "W 7,0,4,64601")
                for line in oled_sample()]
        with self.assertRaises(bench.ProtocolError):
            boundary = self.boundary()
            for line in rows:
                boundary.feed(line)

    def test_oled_summary_partitions_window_and_task_without_claiming_exclusive_cpu(self):
        parser = self.base()
        for line in oled_metadata(skip=1) + oled_sample(0) + oled_sample(1):
            parser.feed(line)
        summary, _ = parser.summarize()
        oled = summary["oled"]
        self.assertTrue(summary["job_success"])
        self.assertTrue(oled["records_complete"])
        self.assertTrue(oled["accounting_valid"])
        self.assertTrue(oled["application_observed"])
        self.assertEqual(oled["sample_count"], 2)
        self.assertAlmostEqual(oled["stage_residence"]["imu"]["p50_us"], 500/168)
        self.assertAlmostEqual(oled["stage_elapsed"]["imu"]["p50_us"], 5000/168)
        self.assertIn("including preemption", oled["note"])
        self.assertIn("whole job", oled["note"])

    def test_oled_nonzero_flags_preserved_but_not_valid_accounting(self):
        parser = self.base(count=1)
        for line in oled_metadata() + oled_sample(flags=4):
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["job_success"])
        self.assertTrue(summary["oled"]["records_complete"])
        self.assertFalse(summary["oled"]["accounting_valid"])
        self.assertEqual(summary["oled"]["nonzero_flags"], [{"i": 0, "flags": 4}])

    def test_oled_base_profile_flags_prevent_valid_accounting(self):
        parser = self.base(count=1, profile_flags=4)
        for line in oled_metadata() + oled_sample():
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["oled"]["records_complete"])
        self.assertFalse(summary["oled"]["accounting_valid"])

    def test_oled_zero_frames_do_not_prove_application_but_can_have_valid_accounting(self):
        for skip in (0, 1):
            parser = self.base(count=1)
            meta = oled_metadata(skip=skip, begun=0, ended=0)
            meta[3:5] = ["CBOLED L 7,0,0", "CBOLED M 7,0,0"]
            for line in meta + oled_sample():
                parser.feed(line)
            summary, _ = parser.summarize()
            with self.subTest(skip=skip):
                self.assertTrue(summary["oled"]["records_complete"])
                self.assertTrue(summary["oled"]["accounting_valid"])
                self.assertFalse(summary["oled"]["application_observed"])

    def test_oled_partial_budget_and_zero_sample_cancel_preserve_failure(self):
        parser = self.base(count=1, phase="BUDGET", work=2, flag=1)
        for line in oled_metadata() + oled_sample():
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertTrue(summary["oled"]["accounting_valid"])
        parser = self.base(count=0, phase="CANCELLED")
        meta = oled_metadata(skip=1, begun=0, ended=0)
        meta[3:5] = ["CBOLED L 7,0,0", "CBOLED M 7,0,0"]
        for line in meta:
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertTrue(summary["oled"]["records_complete"])
        self.assertFalse(summary["oled"]["accounting_valid"])
        self.assertFalse(summary["oled"]["application_observed"])

    def test_offline_oled_missing_meta_sample_or_profile_fails(self):
        for extra in (oled_metadata()[:2], oled_metadata() + oled_sample(),
                      oled_metadata() + oled_sample(0)[:-1] + oled_sample(1),
                      oled_metadata() + oled_sample(0) + oled_sample(1, job_id=8)):
            parser = self.base()
            for line in extra:
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertFalse(summary["collection_complete"])
            self.assertTrue(summary["kernel_success"])
        parser = self.base(count=1)
        parser.jobs[7]["profile_records"] = None
        for line in oled_metadata() + oled_sample():
            parser.feed(line)
        self.assertFalse(parser.summarize()[0]["collection_complete"])

    def test_kernel_status_is_independent_of_auxiliary_errors_and_never_success_for_budget_or_cancel(self):
        parser = bench.CaptureParser()
        parser.feed("CBOLED E 7,0,0")
        parser.feed("CBPROF E 7,0,3,0")
        for line in metadata(n=1, samples=1) + sample(0):
            parser.feed(line)
        summary, _ = parser.summarize()
        self.assertTrue(summary["kernel_collection_complete"])
        self.assertTrue(summary["kernel_success"])
        self.assertFalse(summary["collection_complete"])
        self.assertFalse(summary["job_success"])
        self.assertEqual(len(summary["errors"]), 2)
        for phase, count in (("BUDGET", 1), ("CANCELLED", 0)):
            parser = self.base(count=count, phase=phase, flag=1 if count else 0)
            summary, _ = parser.summarize()
            self.assertTrue(summary["kernel_collection_complete"])
            self.assertFalse(summary["kernel_success"])
        parser = self.base(count=1)
        parser.jobs[7]["results"].clear()
        summary, _ = parser.summarize()
        self.assertFalse(summary["kernel_collection_complete"])
        self.assertFalse(summary["kernel_success"])

    def test_oled_cli_rejects_missing_run_profile_and_large_work_before_import(self):
        cases = [["--oled-mode", "normal"], ["--port", "OFFLINE_FAKE", "--run", "--oled-mode", "normal"],
                 ["--port", "OFFLINE_FAKE", "--run", "--profile", "--oled-mode", "skip-render", "--work", "6"],
                 ["--parse", "absent.jsonl", "--oled-mode", "normal"]]
        for argv in cases:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                bench.main(argv)
            self.assertEqual(exc.exception.code, 2)

    def test_oread_context_is_checked_before_serial_write(self):
        for problem in ("disabled", "nonterminal", "missingprofile", "incompleteprofile", "wrongn"):
            parser = self.base()
            for line in oled_metadata(enabled=0 if problem == "disabled" else 1):
                parser.feed(line)
            if problem == "nonterminal":
                parser.jobs[7]["phase"] = "RUNNING"
            elif problem == "missingprofile":
                parser.jobs[7]["profile_records"] = None
            elif problem == "incompleteprofile":
                del parser.jobs[7]["profile_records"].records[1]
            elif problem == "wrongn":
                parser.jobs[7]["profile_meta"]["n"] = 1
            with tempfile.TemporaryDirectory() as temporary, self.subTest(problem=problem):
                recorder = bench.Recorder(Path(temporary)/"raw.jsonl")
                port = FakePort([])
                with self.assertRaises(bench.ProtocolError):
                    bench.SafeSession(port, recorder, .01, parser).request("#CBENCH OREAD 0,1")
                recorder.close()
                self.assertEqual(port.writes, [])

    def test_oled_maximum_legal_lines_fit_response_payload(self):
        lines = ["CBOLED v=1 id=4294967295 enabled=1 skip=1", "CBOLED caps samples=128 stages=5",
                 "CBOLED F 4294967295,4294967295,4294967295,4294967295",
                 "CBOLED L 4294967295,4294967295,4294967295", "CBOLED M 4294967295,4294967295,4294967295",
                 "CBOLED P 4294967295,127,4,4294967295", "CBOLED W 4294967295,127,4,4294967295",
                 "CBOLED E 4294967295,127,4294967295", "CBOLED end id=4294967295"]
        for line in lines:
            self.assertLess(len(line.encode("ascii")), 62, line)

    def test_twelve_real_captures_replay_without_new_oled_queries_or_mutation(self):
        stamps = ["031427227642", "031443598167", "033714146669", "033723804593",
                  "034331525527", "034340250808", "035324572035", "035329868626",
                  "055510423993", "055526657141", "055532013634", "055536071752"]
        phases = [None, None, "IDLE", "DONE", "DONE", "BUDGET", "DONE", "BUDGET", None, "IDLE", "BUDGET", "BUDGET"]
        for stamp, phase in zip(stamps, phases):
            path = bench.REPO/f"firmware_artifacts/cartesian-benchmark-20261016T{stamp}Z/raw.jsonl"
            if not path.exists():
                self.skipTest("Host-specific real captures are unavailable")
            original = path.read_bytes()
            parser = bench.CaptureParser()
            for line in bench.capture_lines(path):
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertEqual(summary.get("phase"), phase)
            self.assertEqual(path.read_bytes(), original)
            if phase is not None:
                self.assertTrue(summary["collection_complete"])
                self.assertFalse(summary["oled"]["requested"])
            if stamp == "055536071752":
                self.assertTrue(summary["profile"]["accounting_valid"])
                self.assertEqual(summary["profile"]["sample_count"], 4)


class Rate10Tests(unittest.TestCase):
    base = OledTests.base
    fake_capture = OledTests.fake_capture

    def check_metadata(self, rows, job_id=7):
        boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=job_id)
        outcomes = [boundary.feed(line) for line in rows]
        self.assertEqual(outcomes, [False] * (len(rows) - 1) + [True])
        return boundary.oled_metadata.values

    def summary_for(self, rows, phase="DONE", profile_flags=0):
        parser = self.base(phase=phase, profile_flags=profile_flags)
        for line in rows + oled_sample(0) + oled_sample(1):
            parser.feed(line)
        return parser.summarize()[0]

    def test_v2_all_modes_and_disabled_nine_line_contract(self):
        for mode in range(3):
            with self.subTest(mode=mode):
                values = self.check_metadata(oled_metadata_v2(mode=mode))
                self.assertEqual(values["version"], 2)
                self.assertEqual(values["mode"], mode)
                self.assertEqual(values["period_ticks"], 100 if mode == 2 else 0)
        disabled = self.check_metadata(oled_metadata_v2(job_id=0, enabled=0), job_id=0)
        self.assertEqual(disabled["mode"], 0)
        self.assertEqual(disabled["begun"], 0)

    def test_v2_missing_any_line_never_completes_or_raises(self):
        valid = oled_metadata_v2()
        for index in range(len(valid)):
            with self.subTest(index=index):
                boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
                outcomes = []
                try:
                    outcomes = [boundary.feed(line) for line in valid[:index] + valid[index+1:]]
                except bench.ProtocolError:
                    pass
                self.assertNotIn(True, outcomes)

    def test_v2_duplicate_each_line_rejected(self):
        valid = oled_metadata_v2()
        for index in range(len(valid)):
            with self.subTest(index=index), self.assertRaises(bench.ProtocolError):
                self.check_metadata(valid[:index+1] + [valid[index]] + valid[index+1:])

    def test_v2_adjacent_out_of_order_pairs_rejected(self):
        valid = oled_metadata_v2()
        for index in range(len(valid)-1):
            rows = list(valid)
            rows[index], rows[index+1] = rows[index+1], rows[index]
            with self.subTest(index=index), self.assertRaises(bench.ProtocolError):
                self.check_metadata(rows)

    def test_v2_schema_caps_and_mode_mismatches_rejected(self):
        valid = oled_metadata_v2()
        changes = [(0, "v=2", "v=3"), (0, " mode=2", ""), (0, "mode=2", "mode=3"),
                   (0, "enabled=1 skip=0", "enabled=1 skip=1"),
                   (1, "samples=128", "samples=127"), (1, "stages=5", "stages=6"),
                   (2, "period_ticks=100", "period_ticks=99"), (2, "tick_hz=1000", "tick_hz=100"),
                   (3, "F 7,", "F 8,"), (6, "R 7,", "R 8,"), (7, "G 7,", "G 8,"),
                   (8, "id=7", "id=8")]
        for index, old, new in changes:
            rows = list(valid)
            rows[index] = rows[index].replace(old, new)
            with self.subTest(change=(index, old, new)), self.assertRaises(bench.ProtocolError):
                self.check_metadata(rows)
        with self.assertRaises(bench.ProtocolError):
            self.check_metadata([oled_metadata()[0] + " mode=0"] + oled_metadata()[1:])
        for key in (valid[2], valid[6], valid[7]):
            with self.subTest(v1_extra=key), self.assertRaises(bench.ProtocolError):
                self.check_metadata(oled_metadata()[:2] + [key] + oled_metadata()[2:])

    def test_v2_uint_negative_unicode_huge_and_extra_fields_rejected(self):
        valid = oled_metadata_v2()
        for index, target in ((0, "id=7"), (2, "period_ticks=100"), (2, "tick_hz=1000"),
                              (3, ",16"), (4, ",0"), (5, ",0"), (6, ",3"), (7, ",100"), (8, "id=7")):
            for value in ("-1", "4294967296", "9" * 1000, "１００", "+1"):
                replacement = target.split("=")[0] + "=" + value if "=" in target else "," + value
                rows = list(valid)
                rows[index] = rows[index].replace(target, replacement, 1)
                with self.subTest(index=index, value=value[:12]), self.assertRaises(bench.ProtocolError):
                    self.check_metadata(rows)
        for index in (3, 4, 5, 6, 7):
            rows = list(valid)
            rows[index] += ",0"
            with self.subTest(extra=index), self.assertRaises(bench.ProtocolError):
                self.check_metadata(rows)

    def test_v2_mode_counter_combinations_rejected(self):
        cases = [oled_metadata_v2(mode=0, render_begun=15),
                 oled_metadata_v2(mode=0, rate_limited=1),
                 oled_metadata_v2(mode=1, render_begun=1),
                 oled_metadata_v2(mode=1, rate_limited=1),
                 oled_metadata_v2(begun=16, ended=14),
                 oled_metadata_v2(render_begun=17),
                 oled_metadata_v2(rate_limited=17),
                 oled_metadata_v2(rate_limited=12),
                 oled_metadata_v2(rate_limited=14),
                 oled_metadata_v2(min_gap=110, max_gap=100),
                 oled_metadata_v2(render_begun=1, min_gap=100),
                 oled_metadata_v2(render_begun=0, min_gap=0, max_gap=100)]
        valid = oled_metadata_v2()
        for index, old, new in ((3, ",16,16,13", ",16,16,12"),
                                (0, "enabled=1", "enabled=0"),
                                (4, ",0,0", ",17,1"), (5, ",0,0", ",17,1")):
            rows = list(valid)
            rows[index] = rows[index].replace(old, new)
            cases.append(rows)
        disabled = oled_metadata_v2(enabled=0)
        disabled[0] = disabled[0].replace("mode=0", "mode=2")
        cases.append(disabled)
        for rows in cases:
            with self.subTest(rows=rows), self.assertRaises(bench.ProtocolError):
                self.check_metadata(rows)

    def test_v2_terminal_inflight_render_or_rate_frame_is_retained(self):
        for rendered in (3, 4):
            values = self.check_metadata(oled_metadata_v2(begun=17, ended=16,
                                                         render_begun=rendered, rate_limited=13))
            self.assertEqual(values["begun"] - values["ended"], 1)
            self.assertEqual(values["render_begun"], rendered)

    def test_v2_normal_can_have_zero_tick_gap_and_uint32_max_counters(self):
        values = self.check_metadata(oled_metadata_v2(mode=0, begun=bench.UINT32_MAX,
                                                     ended=bench.UINT32_MAX, min_gap=0, max_gap=0))
        self.assertEqual(values["render_begun"], bench.UINT32_MAX)

    def test_rate_policy_observed_separate_from_stage_accounting_and_kernel(self):
        summary = self.summary_for(oled_metadata_v2())
        self.assertTrue(summary["kernel_success"])
        self.assertTrue(summary["oled"]["accounting_valid"])
        self.assertTrue(summary["oled"]["policy_observed"])
        self.assertEqual(summary["oled"]["policy_status"], "observed")
        self.assertTrue(summary["oled"]["rate10_test_passed"])
        invalid = self.summary_for(oled_metadata_v2(), profile_flags=4)
        self.assertTrue(invalid["kernel_success"])
        self.assertFalse(invalid["oled"]["accounting_valid"])
        self.assertTrue(invalid["oled"]["policy_observed"])
        self.assertFalse(invalid["oled"]["rate10_test_passed"])

    def test_rate_policy_insufficient_evidence_does_not_claim_pass(self):
        for rendered, limited in ((0, 16), (1, 15), (16, 0)):
            summary = self.summary_for(oled_metadata_v2(render_begun=rendered, rate_limited=limited))
            with self.subTest(rendered=rendered, limited=limited):
                self.assertTrue(summary["kernel_success"])
                self.assertTrue(summary["oled"]["accounting_valid"])
                self.assertFalse(summary["oled"]["policy_observed"])
                self.assertEqual(summary["oled"]["policy_status"], "insufficient_evidence")
                self.assertFalse(summary["oled"]["rate10_test_passed"])

    def test_rate_policy_short_gap_is_failure_evidence_preserved(self):
        summary = self.summary_for(oled_metadata_v2(min_gap=99, max_gap=120))
        self.assertTrue(summary["collection_complete"])
        self.assertTrue(summary["oled"]["accounting_valid"])
        self.assertEqual(summary["oled"]["metadata"]["min_render_gap_ticks"], 99)
        self.assertFalse(summary["oled"]["policy_observed"])
        self.assertEqual(summary["oled"]["policy_status"], "violated")
        self.assertFalse(summary["oled"]["rate10_test_passed"])

    def test_v2_budget_never_rate_test_pass_even_with_policy_evidence(self):
        summary = self.summary_for(oled_metadata_v2(), phase="BUDGET")
        self.assertFalse(summary["kernel_success"])
        self.assertTrue(summary["oled"]["policy_observed"])
        self.assertFalse(summary["oled"]["rate10_test_passed"])

    def test_old_board_rate_request_rejected_before_RUN_and_records_capability(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = OledTests.FakeOledUSB(mode="rate-10hz", version=1)
            code, output = self.fake_capture(port, "rate-10hz", temporary)
            self.assertEqual(code, 2)
            self.assertFalse(port.started)
            self.assertFalse(any(encoded.startswith(b"#CBENCH RUN ") for encoded in port.writes))
            session = json.loads((output/"session.json").read_text(encoding="utf-8"))
            self.assertEqual(session["oled_capability"]["version"], 1)
            self.assertFalse(session["compute_requested"])
            self.assertIn("requires CBOLED v2", session["error"])

    def test_v2_fake_usb_all_modes_collect_and_offline_replay(self):
        for mode in ("normal", "skip-render", "rate-10hz"):
            with tempfile.TemporaryDirectory() as temporary, self.subTest(mode=mode):
                port = OledTests.FakeOledUSB(mode=mode, version=2)
                code, output = self.fake_capture(port, mode, temporary)
                self.assertEqual(code, 0)
                commands = [item.decode("ascii").strip() for item in port.writes]
                self.assertIn(f"#CBENCH RUN 2,0,3,2,20000,1,{bench.OLED_MODES[mode]}", commands)
                summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
                self.assertTrue(summary["job_success"])
                self.assertEqual(summary["oled"]["metadata"]["version"], 2)
                self.assertEqual(summary["oled"]["metadata"]["mode"], bench.OLED_MODES[mode])
                self.assertEqual(summary["oled"]["rate10_test_passed"], True if mode == "rate-10hz" else None)
                self.assertIn(mode, (output/"oled.csv").read_text(encoding="utf-8"))
                with contextlib.redirect_stdout(io.StringIO()) as printed:
                    self.assertEqual(bench.main(["--parse", str(output/"raw.jsonl"), "--output-root", temporary]), 0)
                replay = json.loads((Path(printed.getvalue().strip())/"summary.json").read_text(encoding="utf-8"))
                self.assertEqual(replay["oled"]["metadata"], summary["oled"]["metadata"])
                self.assertEqual(replay["oled"]["rate10_test_passed"], summary["oled"]["rate10_test_passed"])

    def test_v2_wrong_terminal_mode_does_not_issue_OREAD(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = OledTests.FakeOledUSB(mode="rate-10hz", version=2, final_mode=0)
            code, output = self.fake_capture(port, "rate-10hz", temporary)
            self.assertEqual(code, 2)
            self.assertFalse(any(item.startswith(b"#CBENCH OREAD ") for item in port.writes))
            self.assertFalse(json.loads((output/"summary.json").read_text(encoding="utf-8"))["oled"]["rate10_test_passed"])

    def test_v2_maximum_lines_fit_response_payload(self):
        rows = oled_metadata_v2(job_id=bench.UINT32_MAX, mode=0, begun=bench.UINT32_MAX,
                                ended=bench.UINT32_MAX, min_gap=bench.UINT32_MAX, max_gap=bench.UINT32_MAX)
        self.check_metadata(rows, job_id=bench.UINT32_MAX)
        rows += ["CBOLED rate period_ticks=4294967295 tick_hz=4294967295",
                 "CBOLED R 4294967295,4294967295,4294967295", "CBOLED G 4294967295,4294967295,4294967295"]
        for line in rows:
            self.assertLess(len(line.encode("ascii")), 62, line)

    def test_v2_job_mode_must_stay_fixed_for_same_id(self):
        parser = self.base()
        for line in oled_metadata_v2(mode=0) + oled_metadata_v2(mode=2):
            parser.feed(line)
        self.assertFalse(parser.summarize()[0]["oled"]["records_complete"])
        self.assertIn("OLED job mode changed", " ".join(parser.jobs[7]["oled_errors"]))

    def test_v2_incomplete_saved_metadata_retains_kernel_success_without_collection_success(self):
        for count in range(9):
            summary = self.summary_for(oled_metadata_v2()[:count])
            with self.subTest(count=count):
                self.assertTrue(summary["kernel_success"])
                self.assertFalse(summary["collection_complete"])

    def test_four_real_v1_oled_captures_replay_without_mutation(self):
        stamps = ("062721713638", "062725772613", "062729846713", "062733871560")
        for index, stamp in enumerate(stamps):
            path = bench.REPO/f"firmware_artifacts/cartesian-benchmark-20261016T{stamp}Z/raw.jsonl"
            if not path.exists():
                self.skipTest("Host-specific real OLED captures are unavailable")
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            parser = bench.CaptureParser()
            for line in bench.capture_lines(path):
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertTrue(summary["collection_complete"])
            self.assertEqual(summary["phase"], "IDLE" if index == 0 else "DONE")
            if index:
                self.assertTrue(summary["oled"]["accounting_valid"])
                self.assertEqual(summary["oled"]["metadata"]["version"], 1)
                self.assertIsNone(summary["oled"]["policy_observed"])
                self.assertIsNone(summary["oled"]["rate10_test_passed"])
                self.assertEqual(summary["oled"]["sample_count"], 16)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)


def split_profile_metadata(job_id=7, mode=1, n=2):
    return [f"CBPROF v=2 id={job_id} mode={mode} n={n}",
            "CBPROF caps samples=128 work=5 tasks=7", "CBPROF mapmask=63 irq_included=1",
            f"CBPROF end id={job_id}"]


def split_profile_sample(index=0, job_id=7, work=3, flags=0, split=True, wrapped=False):
    lines = profile_sample(index, job_id, work, flags, wrapped)
    lines[-2:-1] = ([f"CBPROF T {job_id},{index},5,{500 if split else 0}",
                    f"CBPROF T {job_id},{index},6,{600 if split else 1100}"])
    return lines


def oled_metadata_v3(job_id=7, enabled=1, mode=4, runtime_flags=0, restored=1,
                     priority=None, render=3, limited=13, min_gap=100, max_gap=100):
    worker = oled_metadata_v2(job_id, enabled, mode=min(mode, 2))[:-1]
    worker[0] = f"CBOLED v=3 id={job_id} enabled={enabled} skip={int(enabled and mode == 1)} mode={mode if enabled else 0}"
    worker[1] += " lanes=2"
    if enabled and mode >= 3:
        worker[3] = f"CBOLED F {job_id},16,16,16"
        worker[6:8] = [f"CBOLED R {job_id},0,0", f"CBOLED G {job_id},0,0"]
    display = ([f"CBOLED D {job_id},16,16,{limited}", f"CBOLED J {job_id},1,2",
                f"CBOLED N {job_id},0,100000", f"CBOLED U {job_id},{render},{limited}",
                f"CBOLED H {job_id},{min_gap},{max_gap}"] if enabled and mode >= 3 else
               [f"CBOLED {kind} {job_id},0,0" + (",0" if kind == "D" else "") for kind in "DJNUH"])
    return worker + display + [f"CBOLED S {job_id},{20 if enabled else 0},{20 if enabled and mode >= 3 else 0}",
                f"CBOLED A {job_id},{3 if enabled and mode >= 3 else 0},{4 if enabled and mode >= 3 else 0},{101 if enabled else 0}",
                f"CBOLED C {job_id},{runtime_flags},{restored},{(16 if mode == 4 else 24) if priority is None else priority}",
                f"CBOLED K {job_id},1900,2048", f"CBOLED end id={job_id}"]


def oled_sample_v3(index=0, job_id=7, flags=0, split=True):
    worker_p = (500, 300, 0, 0, 1200) if split else (500, 300, 600, 400, 200)
    worker_w = (5000, 3000, 0, 0, 160100) if split else (5000, 3000, 60000, 40000, 60100)
    display_q = (0, 0, 200, 300, 0) if split else (0, 0, 0, 0, 0)
    display_v = (0, 0, 50000, 30000, 88100) if split else (0, 0, 0, 0, 168100)
    return ([f"CBOLED {kind} {job_id},{index},{stage},{values[stage]}" for stage in range(5)
             for kind, values in (("P", worker_p), ("W", worker_w), ("Q", display_q), ("V", display_v))]
            + [f"CBOLED E {job_id},{index},{flags}"])


class SplitDisplayTests(unittest.TestCase):
    class FakeSplitUSB(OledTests.FakeOledUSB):
        def __init__(self, mode="split-low", oled_version=3, profile_version=2,
                     previous_failed=False, final_mode=None, final_flags=0):
            super().__init__(mode=mode, version=oled_version)
            self.profile_version = profile_version
            self.previous_failed, self.final_mode, self.final_flags = previous_failed, final_mode, final_flags

        def write(self, encoded):
            command = encoded.decode("ascii").strip()
            lines = None
            if command == "#CBENCH PROFILE":
                factory = split_profile_metadata if self.profile_version == 2 else profile_metadata
                lines = factory(job_id=int(self.started), mode=int(self.started), n=2 if self.started else 0)
            elif command == "#CBENCH OLED" and self.version == 3:
                mode = bench.OLED_MODES[self.mode] if self.final_mode is None else self.final_mode
                lines = oled_metadata_v3(job_id=int(self.started), enabled=int(self.started), mode=mode,
                         runtime_flags=self.final_flags if self.started else 7 if self.previous_failed else 0,
                         restored=1 if self.started or not self.previous_failed else 0,
                         priority=(16 if mode == 4 else 24) if self.started else 0)
            elif command == "#CBENCH PREAD 0,2" and self.profile_version == 2:
                lines = split_profile_sample(0, job_id=1, split=bench.OLED_MODES[self.mode] >= 3)
                lines += split_profile_sample(1, job_id=1, split=bench.OLED_MODES[self.mode] >= 3)
            elif command == "#CBENCH OREAD 0,2" and self.version == 3:
                lines = oled_sample_v3(0, job_id=1, split=bench.OLED_MODES[self.mode] >= 3)
                lines += oled_sample_v3(1, job_id=1, split=bench.OLED_MODES[self.mode] >= 3)
            if lines is None:
                return super().write(encoded)
            self.writes.append(encoded)
            self.chunks.append(("\r\n".join(lines) + "\r\n").encode("ascii"))
            return len(encoded)

    def parser_for(self, mode=4, phase="DONE", oled=None, profile_flags=0, oled_flags=0):
        parser = bench.CaptureParser()
        lines = metadata(phase=phase, n=2, samples=2) + csv_samples()
        lines += split_profile_metadata() + split_profile_sample(0, flags=profile_flags, split=mode >= 3)
        lines += split_profile_sample(1, split=mode >= 3)
        lines += oled if oled is not None else oled_metadata_v3(mode=mode)
        lines += oled_sample_v3(0, flags=oled_flags, split=mode >= 3) + oled_sample_v3(1, split=mode >= 3)
        for line in lines:
            parser.feed(line)
        return parser

    def read_boundary(self, count=1):
        parser = self.parser_for()
        return bench.ResponseBoundary(f"#CBENCH OREAD 0,{count}", expected_id=7,
                    profile_records=parser.jobs[7]["profile_records"].records,
                    profile_version=2, oled_version=3)

    def metadata_values(self, lines):
        boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
        results = [boundary.feed(line) for line in lines]
        self.assertEqual(results, [False] * 17 + [True])
        return boundary.oled_metadata.values

    def test_new_modes_allowlist_and_cli_require_explicit_profile(self):
        for mode in (3, 4):
            self.assertTrue(bench.command_allowed(f"#CBENCH RUN 3,0,5,128,20000,1,{mode}"))
            for suffix in (f",0,{mode}", f",{mode}", f",1,{mode},0"):
                self.assertFalse(bench.command_allowed("#CBENCH RUN 3,0,5,128,20000" + suffix))
        for mode in ("split-same", "split-low"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                bench.main(["--port", "OFFLINE_FAKE", "--run", "--oled-mode", mode])

    def test_profile_v2_caps_and_seven_bucket_conservation(self):
        parser = self.parser_for()
        summary, _ = parser.summarize()
        self.assertTrue(summary["profile"]["accounting_valid"])
        self.assertEqual(summary["profile"]["metadata"]["version"], 2)
        self.assertEqual(tuple(summary["profile"]["task_residence"]), bench.PROFILE_TASKS_V2)
        row = parser.profile_summary(parser.jobs[7])[1][0]
        self.assertEqual(row["task_oled_worker_cycles"], 2000)
        self.assertEqual(row["task_display_cycles"], 500)
        self.assertEqual(row["task_other_cycles"], 600)

    def test_profile_v2_wrong_caps_map_version_and_bucket_fail(self):
        for old, new in (("tasks=7", "tasks=6"), ("mapmask=63", "mapmask=64"), ("v=2", "v=3")):
            boundary = bench.ResponseBoundary("#CBENCH PROFILE", expected_id=7)
            with self.subTest(new=new), self.assertRaises(bench.ProtocolError):
                for line in split_profile_metadata():
                    boundary.feed(line.replace(old, new))
        data = {0: {"work": 3, "cycles": 168000}}
        for change in ("missing", "duplicate", "extra", "sum"):
            rows = split_profile_sample()
            if change == "missing": rows.remove("CBPROF T 7,0,6,600")
            elif change == "duplicate": rows.insert(-1, "CBPROF T 7,0,5,500")
            elif change == "extra": rows.insert(-1, "CBPROF T 7,0,7,0")
            else: rows = [row.replace("T 7,0,6,600", "T 7,0,6,601") for row in rows]
            record = bench.ProfileRecords(7, data, 168000000, version=2)
            with self.subTest(change=change), self.assertRaises(bench.ProtocolError):
                for row in rows: record.feed(row)

    def test_metadata_v3_all_five_modes_exact_eighteen_lines(self):
        for mode in range(5):
            with self.subTest(mode=mode):
                values = self.metadata_values(oled_metadata_v3(mode=mode))
                self.assertEqual(values["lanes"], 2)
                self.assertEqual(values["mode"], mode)
                self.assertEqual(values["period_ticks"], 100 if mode >= 2 else 0)

    def test_disabled_v3_capability_allows_cold_start_priority_zero(self):
        values = self.metadata_values(oled_metadata_v3(enabled=0, priority=0))
        self.assertEqual(values["mode"], 0)
        self.assertEqual(values["applied_priority"], 0)

    def test_metadata_v3_each_missing_or_duplicate_or_out_of_order_rejected(self):
        valid = oled_metadata_v3()
        for index in range(18):
            variants = (valid[:index] + valid[index+1:], valid[:index] + [valid[index]] + valid[index:])
            for rows in variants:
                boundary = bench.ResponseBoundary("#CBENCH OLED", expected_id=7)
                with self.subTest(index=index), self.assertRaises((bench.ProtocolError, AssertionError)):
                    results = [boundary.feed(line) for line in rows]
                    self.assertTrue(results[-1])
            if index < 17:
                rows = list(valid)
                rows[index], rows[index+1] = rows[index+1], rows[index]
                with self.subTest(swap=index), self.assertRaises(bench.ProtocolError):
                    self.metadata_values(rows)

    def test_metadata_v3_wrong_versions_id_caps_stack_and_bounds_rejected(self):
        changes = ((0, "v=3", "v=2"), (0, "id=7", "id=8"),
                   (1, "lanes=2", "lanes=1"), (1, " lanes=2", ""),
                   (2, "period_ticks=100", "period_ticks=20"),
                   (15, ",0,1,16", ",0,2,16"), (16, ",1900,2048", ",2049,2048"),
                   (16, ",1900,2048", ",1900,4096"), (13, ",20,20", ",-1,20"),
                   (14, ",3,4", ",4294967296,4"), (14, ",3,4", ",٣,4"))
        for index, old, new in changes:
            rows = oled_metadata_v3()
            rows[index] = rows[index].replace(old, new)
            with self.subTest(new=new), self.assertRaises(bench.ProtocolError):
                self.metadata_values(rows)

    def test_v3_worker_and_display_counter_consistency(self):
        changes = ((3, ",16,16,16", ",16,14,14"), (6, ",0,0", ",1,0"),
                   (7, ",0,0", ",100,100"), (8, ",16,16,13", ",16,14,13"),
                   (8, ",16,16,13", ",16,16,12"), (9, ",1,2", ",17,2"),
                   (10, ",0,100000", ",17,100000"), (12, ",100,100", ",101,100"))
        for index, old, new in changes:
            rows = oled_metadata_v3()
            rows[index] = rows[index].replace(old, new)
            with self.subTest(index=index), self.assertRaises(bench.ProtocolError):
                self.metadata_values(rows)
        for mode in range(3):
            rows = oled_metadata_v3(mode=mode)
            rows[8] = "CBOLED D 7,1,0,0"
            with self.subTest(mode=mode), self.assertRaises(bench.ProtocolError):
                self.metadata_values(rows)

    def test_v3_display_inflight_and_zero_gap_insufficient_preserved(self):
        rows = oled_metadata_v3()
        rows[8] = "CBOLED D 7,17,16,13"
        rows[11] = "CBOLED U 7,4,13"
        values = self.metadata_values(rows)
        self.assertEqual(values["display_begun"] - values["display_ended"], 1)
        rows = oled_metadata_v3(render=1, limited=15, min_gap=0, max_gap=0)
        summary, _ = self.parser_for(oled=rows).summarize()
        self.assertTrue(summary["kernel_success"])
        self.assertFalse(summary["oled"]["policy_observed"])

    def test_two_lane_oread_exact_interleaved_twenty_one_lines(self):
        boundary = self.read_boundary()
        self.assertEqual([boundary.feed(line) for line in oled_sample_v3()], [False] * 20 + [True])
        record = boundary.oled_records.records[0]
        self.assertEqual(sum(record["w"].values()), 168100)
        self.assertEqual(sum(record["v"].values()), 168100)
        self.assertEqual(sum(record["p"].values()), 2000)
        self.assertEqual(sum(record["q"].values()), 500)

    def test_lane_conservation_and_elapsed_not_added(self):
        parser = self.parser_for()
        summary, _ = parser.summarize()
        self.assertTrue(summary["oled"]["accounting_valid"])
        self.assertTrue(summary["oled"]["split_test_passed"])
        row = parser.oled_summary(parser.jobs[7])[1][0]
        self.assertEqual(sum(row[f"{s}_elapsed_cycles"] for s in bench.OLED_STAGES), 168100)
        self.assertEqual(sum(row[f"display_{s}_elapsed_cycles"] for s in bench.OLED_STAGES), 168100)
        self.assertIn("must not be added", summary["oled"]["note"])

    def test_each_v3_stage_missing_or_duplicated_or_out_of_order_rejected(self):
        valid = oled_sample_v3()
        for index in range(21):
            for rows in (valid[:index] + valid[index+1:], valid[:index] + [valid[index]] + valid[index:]):
                boundary = self.read_boundary()
                with self.subTest(index=index), self.assertRaises((bench.ProtocolError, AssertionError)):
                    results = [boundary.feed(line) for line in rows]
                    self.assertTrue(results[-1])
        rows = list(valid)
        rows[0], rows[1] = rows[1], rows[0]
        with self.assertRaises(bench.ProtocolError):
            boundary = self.read_boundary()
            for line in rows: boundary.feed(line)

    def test_each_lane_wrong_total_or_residence_above_elapsed_rejected(self):
        for old, new in (("P 7,0,0,500", "P 7,0,0,501"),
                         ("W 7,0,0,5000", "W 7,0,0,5001"),
                         ("Q 7,0,2,200", "Q 7,0,2,201"),
                         ("V 7,0,2,50000", "V 7,0,2,50001")):
            boundary = self.read_boundary()
            with self.subTest(kind=old[0]), self.assertRaises(bench.ProtocolError):
                for line in oled_sample_v3(): boundary.feed(line.replace(old, new))
        rows = [line.replace("Q 7,0,2,200", "Q 7,0,2,500").replace("Q 7,0,3,300", "Q 7,0,3,0")
                .replace("V 7,0,2,50000", "V 7,0,2,0").replace("V 7,0,4,88100", "V 7,0,4,138100") for line in oled_sample_v3()]
        with self.assertRaises(bench.ProtocolError):
            boundary = self.read_boundary()
            for line in rows: boundary.feed(line)

    def test_v3_rejects_single_lane_profile_and_v1_rejects_display_records(self):
        parser = ProfileTests().base(count=1)
        for line in profile_metadata(n=1) + profile_sample(): parser.feed(line)
        with self.assertRaises(bench.ProtocolError):
            bench.OledRecords(7, parser.jobs[7]["profile_records"].records, version=3)
        for version in (0, 4):
            with self.assertRaises(bench.ProtocolError):
                bench.OledRecords(7, parser.jobs[7]["profile_records"].records, version=version)
        legacy = bench.OledRecords(7, parser.jobs[7]["profile_records"].records)
        with self.assertRaises(bench.ProtocolError): legacy.feed("CBOLED Q 7,0,0,0")

    def test_v3_runtime_failure_restoration_and_wrong_priority_do_not_claim_pass(self):
        for mode in (3, 4):
            for options in ({"runtime_flags": 1}, {"runtime_flags": 2}, {"runtime_flags": 4},
                            {"restored": 0}, {"priority": 16 if mode == 3 else 24}):
                parser = self.parser_for(mode=mode, oled=oled_metadata_v3(mode=mode, **options))
                summary, _ = parser.summarize()
                with self.subTest(mode=mode, options=options):
                    self.assertTrue(summary["kernel_success"])
                    self.assertTrue(summary["collection_complete"])
                    self.assertTrue(summary["oled"]["accounting_valid"])
                    self.assertFalse(summary["oled"]["runtime_valid"])
                    self.assertFalse(summary["oled"]["application_observed"])
                    self.assertFalse(summary["oled"]["policy_observed"])
                    self.assertFalse(summary["oled"]["split_test_passed"])

    def test_v3_profile_or_oled_flags_preserve_kernel_but_prevent_policy_pass(self):
        for options in ({"profile_flags": 4}, {"oled_flags": 2}):
            summary, _ = self.parser_for(**options).summarize()
            self.assertTrue(summary["kernel_success"])
            self.assertFalse(summary["oled"]["accounting_valid"])
            self.assertFalse(summary["oled"]["policy_observed"])
            self.assertFalse(summary["oled"]["split_test_passed"])

    def test_v3_budget_abort_and_cancel_never_split_pass(self):
        for phase in ("BUDGET", "ABORTED", "CANCELLED", "CLOCK_ERROR"):
            summary, _ = self.parser_for(phase=phase).summarize()
            with self.subTest(phase=phase):
                self.assertTrue(summary["oled"]["policy_observed"])
                self.assertFalse(summary["kernel_success"])
                self.assertFalse(summary["oled"]["split_test_passed"])

    def test_v3_short_render_gap_is_preserved_as_policy_failure(self):
        summary, _ = self.parser_for(oled=oled_metadata_v3(min_gap=99)).summarize()
        self.assertTrue(summary["collection_complete"])
        self.assertEqual(summary["oled"]["metadata"]["display_min_render_gap_ticks"], 99)
        self.assertFalse(summary["oled"]["split_test_passed"])
        self.assertEqual(summary["oled"]["policy_status"], "violated")

    def test_v3_legacy_modes_display_residence_zero_elapsed_other_full_window(self):
        for mode in range(3):
            parser = self.parser_for(mode=mode)
            summary, _ = parser.summarize()
            self.assertTrue(summary["oled"]["accounting_valid"])
            record = parser.jobs[7]["oled_records"].records[0]
            self.assertEqual(sum(record["q"].values()), 0)
            self.assertEqual(record["v"][4], 168100)
            self.assertIsNone(summary["oled"]["split_test_passed"])

    def test_v3_incomplete_saved_metadata_keeps_kernel_result(self):
        for count in range(18):
            summary, _ = self.parser_for(oled=oled_metadata_v3()[:count]).summarize()
            with self.subTest(count=count):
                self.assertTrue(summary["kernel_success"])
                self.assertFalse(summary["oled"]["records_complete"])

    def test_new_modes_full_fake_usb_and_offline_replay_csv(self):
        for mode in bench.OLED_MODES:
            with tempfile.TemporaryDirectory() as temporary, self.subTest(mode=mode):
                port = self.FakeSplitUSB(mode)
                code, output = OledTests().fake_capture(port, mode, temporary)
                self.assertEqual(code, 0)
                self.assertFalse(port.opened)
                summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
                self.assertTrue(summary["kernel_success"])
                self.assertTrue(summary["profile"]["accounting_valid"])
                self.assertTrue(summary["oled"]["accounting_valid"])
                self.assertTrue(summary["oled"]["application_observed"])
                self.assertEqual(summary["oled"]["split_test_passed"], True if mode.startswith("split-") else None)
                self.assertFalse(summary["hardware_evidence"])
                self.assertIn("task_display_cycles", (output/"profile.csv").read_text(encoding="utf-8"))
                self.assertIn("display_send_resident_cycles", (output/"oled.csv").read_text(encoding="utf-8"))
                commands = [row.decode("ascii").strip() for row in port.writes]
                self.assertTrue(all(bench.command_allowed(row) for row in commands))
                self.assertLess(commands.index("#CBENCH PROFILE"), commands.index(f"#CBENCH RUN 2,0,3,2,20000,1,{bench.OLED_MODES[mode]}"))
                self.assertLess(commands.index("#CBENCH PREAD 0,2"), commands.index("#CBENCH OREAD 0,2"))
                with contextlib.redirect_stdout(io.StringIO()) as printed:
                    self.assertEqual(bench.main(["--parse", str(output/"raw.jsonl"), "--output-root", temporary]), 0)
                replay = json.loads((Path(printed.getvalue().strip())/"summary.json").read_text(encoding="utf-8"))
                self.assertEqual(replay["oled"]["metadata"], summary["oled"]["metadata"])
                self.assertEqual(replay["oled"]["split_test_passed"], summary["oled"]["split_test_passed"])

    def test_split_capability_rejected_on_old_oled_or_profile_before_run(self):
        for oled_version, profile_version in ((1, 1), (2, 1), (2, 2), (3, 1)):
            for mode in ("split-same", "split-low"):
                with tempfile.TemporaryDirectory() as temporary, self.subTest(oled=oled_version, profile=profile_version, mode=mode):
                    port = self.FakeSplitUSB(mode, oled_version=oled_version, profile_version=profile_version)
                    code, output = OledTests().fake_capture(port, mode, temporary)
                    self.assertEqual(code, 2)
                    self.assertFalse(any(row.startswith(b"#CBENCH RUN ") for row in port.writes))
                    self.assertFalse(port.started)
                    session = json.loads((output/"session.json").read_text(encoding="utf-8"))
                    self.assertFalse(session["compute_requested"])

    def test_previous_failed_capability_does_not_reject_new_legal_job(self):
        for mode in ("split-same", "split-low"):
            with tempfile.TemporaryDirectory() as temporary:
                port = self.FakeSplitUSB(mode, previous_failed=True)
                code, output = OledTests().fake_capture(port, mode, temporary)
                self.assertEqual(code, 0)
                summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
                self.assertEqual(summary["session"]["oled_capability"]["runtime_flags"], 7)
                self.assertTrue(summary["oled"]["split_test_passed"])

    def test_final_runtime_failure_is_exported_without_being_timing_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = self.FakeSplitUSB(final_flags=4)
            code, output = OledTests().fake_capture(port, "split-low", temporary)
            self.assertEqual(code, 0)
            summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
            self.assertTrue(summary["kernel_success"])
            self.assertTrue(summary["oled"]["records_complete"])
            self.assertEqual(summary["oled"]["metadata"]["runtime_flags"], 4)
            self.assertFalse(summary["oled"]["split_test_passed"])

    def test_final_wrong_split_mode_rejected_before_oread(self):
        with tempfile.TemporaryDirectory() as temporary:
            port = self.FakeSplitUSB(final_mode=3)
            code, output = OledTests().fake_capture(port, "split-low", temporary)
            self.assertEqual(code, 2)
            self.assertFalse(any(row.startswith(b"#CBENCH OREAD ") for row in port.writes))
            summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
            self.assertFalse(summary["oled"]["split_test_passed"])

    def test_v3_bounded_pretransmit_context_and_wrong_ids(self):
        parser = self.parser_for()
        parser.jobs[7]["profile_records"].records[0]["t"][7] = 0
        with tempfile.TemporaryDirectory() as temporary:
            recorder = bench.Recorder(Path(temporary)/"raw.jsonl")
            port = FakePort([])
            with self.assertRaises(bench.ProtocolError):
                bench.SafeSession(port, recorder, .01, parser).request("#CBENCH OREAD 0,1")
            recorder.close()
            self.assertEqual(port.writes, [])
        for old, new in (("7,0", "8,0"), ("7,0", "7,128"), (",0,500", ",5,500"),
                         (",0,500", ",0,4294967296"), (",0,500", ",0,٥٠٠")):
            with self.subTest(new=new), self.assertRaises(bench.ProtocolError):
                self.read_boundary().feed(oled_sample_v3()[0].replace(old, new))

    def test_v2_profile_dwt_wrap_and_v3_elapsed_preserved(self):
        record = bench.ProfileRecords(7, {0: {"cycles": 168000, "work": 3}}, 168000000, version=2)
        for line in split_profile_sample(wrapped=True): record.feed(line)
        oled = bench.OledRecords(7, record.records, version=3)
        for line in oled_sample_v3(): oled.feed(line)
        self.assertTrue(oled.records[0]["ended"])

    def test_v3_maximum_numeric_record_lines_fit_response_buffer(self):
        maximum = "4294967295"
        lines = [f"CBOLED {kind} {maximum},{maximum},4,{maximum}" for kind in "PWQV"]
        lines += [f"CBOLED C {maximum},{maximum},1,{maximum}", f"CBOLED K {maximum},{maximum},{maximum}",
                  f"CBOLED v=3 id={maximum} enabled=1 skip=0 mode=4", "CBOLED caps samples=128 stages=5 lanes=2"]
        for line in lines: self.assertLess(len(line.encode("ascii")), 62, line)

    def test_send_gap_tail_metric_preserved_without_arbitrary_pass_threshold(self):
        for gap in (0, 100, 500, bench.UINT32_MAX):
            rows = [line.replace("A 7,3,4,101", f"A 7,3,4,{gap}") for line in oled_metadata_v3()]
            summary, _ = self.parser_for(oled=rows).summarize()
            self.assertEqual(summary["oled"]["metadata"]["max_send_gap_ticks"], gap)
            self.assertTrue(summary["oled"]["split_test_passed"])
        rows = [line.replace("A 7,3,4,101", "A 7,3,4") for line in oled_metadata_v3()]
        with self.assertRaises(bench.ProtocolError): self.metadata_values(rows)

    def test_profile_schema_cannot_change_after_enabled_metadata_same_job(self):
        for prior, later in ((profile_metadata(), split_profile_metadata()),
                             (split_profile_metadata(), profile_metadata()),
                             (split_profile_metadata(), split_profile_metadata(mode=0))):
            parser = bench.CaptureParser()
            for line in metadata() + csv_samples() + prior + later:
                parser.feed(line)
            summary, _ = parser.summarize()
            self.assertFalse(summary["profile"]["accounting_valid"])
            self.assertIn("schema/mode changed", " ".join(summary["profile"]["errors"]))


if __name__ == "__main__":
    unittest.main()
