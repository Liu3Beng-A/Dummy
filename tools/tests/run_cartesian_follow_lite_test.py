"""Frozen-source, host-only experiment. Never builds/flashes target firmware.

Run: python tools/tests/run_cartesian_follow_lite_test.py
Outputs: firmware_artifacts/cartesian-follow-lite-<UTC timestamp>/
Requires VS x64 C++ tools on Windows (or g++/clang++ elsewhere). Standard-library
Python runs the model and report; installed matplotlib optionally renders plots.
Neither the proposed follower nor its brake/rail strategy is production code.
--guide-budget-only runs the separately preregistered C / bounded-midpoint
guide comparison (294 runs per math mode), preserving the normal A/B/C matrix.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone


REPO = Path(__file__).resolve().parents[2]
ARTIFACTS = REPO / "firmware_artifacts"
CORE = Path("firmware_new/dummy-ref-core-fw/Robot")
MOTOR = Path("firmware_new/dummy-motor-fw/Ctrl/Motor")
IK = CORE / "algorithms/kinematic"
INST = CORE / "instances"
INPUTS = [
    IK / "6dof_kinematic.cpp", IK / "6dof_kinematic.h",
    INST / "cartesian_planner.cpp", INST / "cartesian_planner.h",
    INST / "dummy_robot.cpp", INST / "dummy_robot.h", INST / "tcp_tool.cpp",
    MOTOR / "motion_planner.cpp", MOTOR / "motion_planner.h",
    MOTOR / "motor.cpp", MOTOR / "motor.h",
    Path("firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp"),
    Path("firmware_new/dummy-motor-fw/UserApp/main.cpp"),
    Path("firmware_new/stream_protocol.h"),
    Path("tools/tests/fixtures/arm_math_native_shim.h"),
    Path("tools/tests/cartesian_follow_lite_test.cpp"),
    Path("tools/tests/run_cartesian_follow_lite_test.py"),
]
PROTECTED = [Path("AGENTS.md"), Path("docs/SESSION_STATUS.md"),
             Path("docs/MoveL有限目标与手柄连续跟随规划_20261014.md"),
             Path("docs/主控MoveL与低精度地轨协同方案_20261014.txt"),
             Path("docs/Dummy七轴MoveL算法包实现方案.md"),
             Path("docs/MoveL离线研究记录_20261015.md")]
RESEARCH_EXPERIMENTS = {
    'r_h60_on': {'guide_ms':60,'feedback_scale':True,'stop_before_turn':True},
    'r_h60_off': {'guide_ms':60,'feedback_scale':False,'stop_before_turn':True},
    'r_h120_on': {'guide_ms':120,'feedback_scale':True,'stop_before_turn':True},
    'r_h120_off': {'guide_ms':120,'feedback_scale':False,'stop_before_turn':True},
    'r_h240_on': {'guide_ms':240,'feedback_scale':True,'stop_before_turn':True},
    'r_h240_off': {'guide_ms':240,'feedback_scale':False,'stop_before_turn':True},
    'r_turn_off': {'guide_ms':60,'feedback_scale':True,'stop_before_turn':False},
    'r_turn_off_no_feedback': {'guide_ms':60,'feedback_scale':False,'stop_before_turn':False},
}
RESEARCH_VARIANTS = ['base60','acc10','speed20','speed2x','feedback20ms','zero_skew','skew6ms']
RESEARCH_CASES = ['short_line_x','short_line_xyz','slider_short','rail_line','rail_hold_tcp','rapid_short_reverse','rapid_short_corner']
LEGACY_BODIES = {
    "    void UpdateBaseline(": "07e477faad2a02a82ce7413a0ead742bf8f35e97ed6bd565216a55b7cbe5bd46",
    "    void Brake(": "e4a072d57832dafb79ec2210764dcf1de3a8ac3d78e4eaaf2cdadc7af3c09553",
    "    void Search(": "2cdcf12eddf0b77c1cac5a2b3f93999ad08732312fa2c3e3f74915043484653a",
    "    Vec FiniteGoal(": "9ab21ab2b88cf6b64bbb65cd0cf08887e6cc70a61dd29fd2a952371a86f7969c",
    "    Vec JoystickVelocity(": "ab1e4aeaa254fc2b7a584f641a3352a9914c53ba3c3680b2de8e717b887c92a9",
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_identity() -> dict[str, str]:
    return {p.as_posix(): digest(REPO / p) for p in INPUTS}


def formal_identity() -> dict[str, str]:
    # All pre-existing top-level release files, including CURRENT_RELEASE and
    # verification evidence. Only a new analysis subdirectory is writable here.
    return {p.name: digest(p) for p in ARTIFACTS.iterdir() if p.is_file()}


def validate_assumptions(snapshot: Path) -> None:
    """Fail if the model's documented hardware mapping no longer matches source."""
    text = (snapshot / INST / "dummy_robot.cpp").read_text(encoding="utf-8-sig")
    for literal in (
        "DOF6Kinematic(0.2073f, 0.0f, 0.170f, 0.117f, 0.0695f, 0.113f)",
        "1, false, 50, -175, 175", "2, true, 50, -75, 90",
        "3, true, 50, 0, 180", "4, true, 50, -270, 270",
        "5, true, 50, -100, 100", "6, true, 30, -180, 180",
        "7, true, 1, -250, 250",
    ):
        if literal not in text:
            raise RuntimeError(f"Model mapping needs review; source no longer contains {literal!r}")
    header = (snapshot / INST / "dummy_robot.h").read_text(encoding="utf-8-sig")
    if "RAIL_MM_PER_CIRCLE 5.0f" not in header:
        raise RuntimeError("Rail pitch changed; update offline model explicitly")
    motor_main = (snapshot / "firmware_new/dummy-motor-fw/UserApp/main.cpp").read_text(encoding="utf-8-sig")
    if motor_main.count(".velocityAcc = 30 * Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS") != 4:
        raise RuntimeError("Motor default acceleration changed; review test baseline")
    source = (snapshot / "tools/tests/cartesian_follow_lite_test.cpp").read_text(encoding="utf-8-sig")
    for signature, expected in LEGACY_BODIES.items():
        start = source.index("{", source.index(signature))
        end, depth = start + 1, 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        if hashlib.sha256(source[start:end].encode()).hexdigest() != expected:
            raise RuntimeError("Original baseline kernel changed: " + signature)


def compare_legacy(out: Path, mode: str, report: dict, identity: dict) -> dict:
    reference = ARTIFACTS / "cartesian-follow-lite-20261015T042204153261Z"
    if not (reference / mode / "results.json").exists():
        return {"available": False, "body_hashes_verified": True}
    previous_identity = json.loads((reference / "source_identity.json").read_text(encoding="utf-8"))["before"]
    unchanged = all(identity["before"][p] == value for p, value in previous_identity.items()
                    if p not in ("tools/tests/cartesian_follow_lite_test.cpp", "tools/tests/run_cartesian_follow_lite_test.py"))
    old = json.loads((reference / mode / "results.json").read_text(encoding="utf-8"))["results"]
    new = {(r["case"], r["variant"], r["fixture"]): r for r in report["results"] if r["controller"] == "baseline"}
    differences = []
    max_absolute = 0.
    for row in old:
        key = row["case"], row["variant"], row["fixture"]
        current = new[key]
        for field, before in row.items():
            after = current[field]
            if isinstance(before, (float, int)):
                delta = abs(before-after)
                max_absolute = max(max_absolute, delta)
                if delta > 1e-6 + 1e-8*abs(before):
                    differences.append([key, field, before, after])
            elif before != after:
                differences.append([key, field, before, after])
    result = {"available": True, "reference": str(reference), "pairs": len(old),
              "same_full_input_identity":unchanged,
              "input_differences":[p for p,value in previous_identity.items() if p not in ('tools/tests/cartesian_follow_lite_test.cpp','tools/tests/run_cartesian_follow_lite_test.py') and identity['before'][p]!=value],
              "max_absolute_difference": max_absolute, "differences": differences, "passed": not differences}
    (out / mode / "baseline_regression.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    if differences:
        raise RuntimeError(f"{mode}: archived baseline regression changed; inspect baseline_regression.json")
    return result


def compare_observation_only(out: Path, mode: str, report: dict, identity: dict) -> dict:
    """All A/B/C old scalar results AND the entire old trace must stay exact."""
    reference = ARTIFACTS / "cartesian-follow-lite-20261015T060108287178Z"
    if not (reference / mode / "results.json").exists():
        raise RuntimeError("Missing pre-instrumentation A/B/C archive: " + str(reference))
    old = json.loads((reference / mode / "results.json").read_text(encoding="utf-8"))["results"]
    new = {(r["controller"], r["case"], r["variant"], r["fixture"]): r for r in report["results"]}
    differences, maximum = [], 0.
    for row in old:
        key = row["controller"], row["case"], row["variant"], row["fixture"]
        for field, before in row.items():
            after = new[key][field]
            if isinstance(before, (float, int)):
                maximum = max(maximum, abs(before-after))
            if before != after:
                differences.append([key, field, before, after])
    previous = json.loads((reference / "source_identity.json").read_text(encoding="utf-8"))["before"]
    compiled = [IK / "6dof_kinematic.cpp", IK / "6dof_kinematic.h",
                INST / "cartesian_planner.cpp", INST / "cartesian_planner.h",
                MOTOR / "motion_planner.cpp", MOTOR / "motion_planner.h",
                Path("tools/tests/fixtures/arm_math_native_shim.h")]
    changed = [p.as_posix() for p in compiled if previous[p.as_posix()] != identity["before"][p.as_posix()]]
    result = {"reference": str(reference), "pairs": len(old), "differences": differences,
              "max_absolute_difference": maximum, "compiled_production_input_differences": changed,
              "legacy_trace_byte_identical": digest(reference / mode / "traces.csv") == digest(out / mode / "traces.csv")}
    result["passed"] = not differences and not changed and result["legacy_trace_byte_identical"]
    (out / mode / "observation_regression.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    if not result["passed"]:
        raise RuntimeError("Diagnostics changed existing behavior/input; inspect observation_regression.json")
    return result


def validate_diagnostics(report: dict) -> dict:
    checks = 0
    for row in report["results"]:
        d = row["diagnostics"]
        if d["observer_mismatches"]:
            raise RuntimeError("PositionTracker branch observer mismatch")
        checks += d["observer_checks"]
        for phase, axes in d["motor_phases"].items():
            totals = [sum(reasons.values()) for reasons in axes.values()]
            if max(totals)-min(totals) > 1e-5:
                raise RuntimeError("Motor branch time accounting differs between axes")
        all_ms = sum(d["motor_phases"]["all"]["J7"].values())
        if abs(all_ms-d["observer_checks"] / 7 / 20) > 1e-5:
            raise RuntimeError("Incomplete 20 kHz observer accounting")
        for phase in d["control_phases"].values():
            for s in phase["reasons"].values():
                if s["ms"] > phase["observed_ms"]+1e-5 or s["longest_ms"] > s["ms"]+1e-5:
                    raise RuntimeError("Invalid slow-reason duration")
    return {"passed": True, "motor_branch_checks": checks,
            "note": "Read-only oracle; no speed/branch data fed to controller. Reasons overlap; durations are not additive."}


def validate_diagnostic_exports(folder: Path, report: dict) -> dict:
    """Cross-check CSV counters against JSON and each successful speed-budget row."""
    totals, checked = {}, 0
    with (folder/'control_updates.csv').open(encoding='utf-8',newline='') as stream:
        for raw in csv.DictReader(stream):
            key=raw['controller'],raw['case'],raw['variant'],int(raw['fixture'])
            group=totals.setdefault(key,{'ms':0.,'reasons':{}})
            dt=float(raw['dt_ms']);group['ms']+=dt
            for name,value in raw.items():
                if name.startswith('reason_'):
                    group['reasons'][name[7:]]=group['reasons'].get(name[7:],0.)+dt*int(value)
            failure=any(int(raw['reason_'+r]) for r in ('stale_feedback','batch_wait','probe_ik_failure','step_ik_failure','guide_ik_failure','tracking_fault'))
            if raw['controller']!='baseline' and int(raw['active']) and not failure:
                stages=[float(raw[k]) for k in ('input_rate','after_turn','after_joint','after_feedback','allowed')]
                if any(a+1e-6<b for a,b in zip(stages,stages[1:])):
                    raise RuntimeError('Non-monotonic speed budget in diagnostic export')
            checked+=1
    for r in report['results']:
        key=r['controller'],r['case'],r['variant'],r['fixture']
        if key not in totals:
            continue
        json_stats=r['diagnostics']['control_phases']['all'];csv_stats=totals[key]
        if abs(json_stats['observed_ms']-csv_stats['ms'])>1e-4 or any(abs(s['ms']-csv_stats['reasons'][reason])>1e-4 for reason,s in json_stats['reasons'].items()):
            raise RuntimeError('CSV / JSON diagnostic duration mismatch')
    return {'passed':True,'control_rows_checked':checked,'exported_runs_checked':len(totals)}


def build(snapshot: Path, build_dir: Path, mode: str) -> tuple[Path, dict]:
    build_dir.mkdir()
    shutil.copyfile(snapshot / "tools/tests/fixtures/arm_math_native_shim.h", build_dir / "arm_math.h")
    sources = [snapshot / "tools/tests/cartesian_follow_lite_test.cpp",
               snapshot / IK / "6dof_kinematic.cpp",
               snapshot / INST / "cartesian_planner.cpp",
               snapshot / MOTOR / "motion_planner.cpp"]
    includes = [build_dir, snapshot / IK, snapshot / INST, snapshot / MOTOR]
    executable = build_dir / ("cartesian_follow_lite_test.exe" if os.name == "nt" else "cartesian_follow_lite_test")
    if os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.check_output([
            str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"
        ], text=True).strip()
        if not installation:
            raise RuntimeError("Visual Studio x64 C++ tools not found")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        command = ["cl", "/nologo", "/utf-8", "/std:c++17", "/O2", "/W4", "/EHsc", "/fp:precise"]
        if mode == "cmsis512":
            command.append("/DCMSIS_INTERPOLATED")
        command += ["/I" + str(p) for p in includes] + [str(p) for p in sources] + ["/Fe:" + str(executable)]
        # Only generated, controlled paths enter this batch; subprocess.list2cmdline
        # supplies Windows argument quoting. No user-supplied source text is run.
        script = build_dir / "build.cmd"
        script.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                          + subprocess.list2cmdline(command) + '\nexit /b %errorlevel%\n', encoding="utf-8")
        process = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=build_dir, capture_output=True)
    else:
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise RuntimeError("C++17 compiler not found")
        command = [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra"]
        if mode == "cmsis512":
            command.append("-DCMSIS_INTERPOLATED")
        command += ["-I" + str(p) for p in includes] + [str(p) for p in sources] + ["-o", str(executable)]
        process = subprocess.run(command, cwd=build_dir, capture_output=True)
    (build_dir / "compile.log").write_bytes(process.stdout + process.stderr)
    if process.returncode:
        raise RuntimeError(f"Host compilation failed; inspect {build_dir / 'compile.log'}")
    return executable, {"command": command, "math": mode, "fp": "precise", "mcu_timing": False}


def write_report(out: Path, reports: dict) -> None:
    lines = ["# 轻量笛卡尔跟随：离线参考执行实验", "",
             "复用真实IK/FK、坐标变换和20kHz PositionTracker；候选跟随器只存在于测试源码。",
             "输出是电机规划参考，不包含PID/编码器/负载/机械误差/真实CAN/RTOS；PC耗时不代表F405/F103预算。",
             "本批运行只写新分析目录；生产固件/上位机及研究文档在运行期间以哈希保护。论文研究目的与文档更新见protocol_snapshot。", "",
             "## A/B定义与观察限制", "",
             "- baseline保留上一轮Update/Brake/Search及输入函数原函数体SHA256；归档可用时逐项回归上一轮430组结果，另行报告生产源码身份差异。",
             "- coordinated新增共同路径速度、关节速度/加速度约束、终点/改向联合减速、逐轴跟随误差检查和位置反馈速度估计。",
             "- B(coordinated)保留固定领先20/60/120ms；C(coordinated_brake_lead)再按各轴制动距离增加共同领先，最多300ms，单独作为诊断对照。实际领先见guide_horizon_max_ms。",
             "- 两方案使用完全相同的IK/FK、PositionTracker、地轨目标搜索和场景；共同进度并未给电机增加共同执行时间戳。",
             "- 新方案控制器只读取20ms位置反馈和接收时刻，使用差分滤波估计速度；不读取电机内部go_velocity。旧方案的独立制动仍保留其原oracle读数。",
             "- 新增feedback20ms：20ms采样，延迟20～26ms；feedback_dropout：0.30～0.44s丢反馈，新方案80ms时效门限。",
             "- 最终目标诊断完成条件：无拒绝、参考剩余<0.01mm、TCP终误差<0.1mm、轨误差<0.1mm、末速<1mm/s；这不是用户验收精度。",
             "- intent_path_max_mm到独立用户意图路径计算，避免将自己修改后的路径当成唯一精度标准；转向距离损失/延迟与全程路程另外报告。",
             "- 原path_max_mm仍是到各自控制器参考折线的距离，用于历史对照；不能把低速/少移动/提前停机视作单纯精度改善。",
             "- 正常改向/松手采用共同速度减速；新方案故障停止只用观测位置/估速计算逐轴保底终点，不等同生产STOP验证。",
             "- 本轮未改地轨分配策略，首轮简化10%触发/越过10%后结束仍保留；完整25%回收未由本次协调对照验证。", "",
             "## 固定实验条件", "",
             "- 主控更新20/40ms；目标领先20/60/120ms；轴间间隔0/0.15/1ms（最后一轴最多落后6ms）。",
             "- 电机加速度30r/s²对应板级默认与主控fallback，10r/s²为敏感性；不是设备当前EEPROM回读。",
             "- 常规TCP速度40mm/s；speed2x为80mm/s，speed10/20为10/20mm/s。固定TCP移轨基准20mm/s，随speedScale缩放。",
             "- TCP朝向固定，包含工具Z+114mm；姿态旋转拖动没有在本轮验证。",
             "- 三个公开固定初始姿态；恢复例使用独立近限位姿态；不挑选通过的样本替代失败样本。",
             "- 保留首轮80/60mm长线与长行程反向压力例，另加20mm短线、15mm滑块目标、0.25s改向的短操作。",
             "- reference lag是同一采样时刻参考与执行参考的位置距离；路径偏差是到完整参考折线的最短距离，不能代替时序误差。",
             "- 速度为20ms位移窗口；CV只在预先规定巡航窗口计算，启动/转弯/停止另外观察。",
             "- 路径/限位/朝向指标每1ms采样，不是连续时间严格上界；原始绘图数据每5ms。",
             "- rail_line给定世界Y+60mm与地轨+90mm；rail_hold_tcp固定世界TCP且移轨40mm，用来单独检查协同。",
             "- rail_recovery是每100ms发起、每运动更新最多2次额外IK的实验搜索；非完整10%/25%整路径策略。",
             "- 输入过期使用最后心跳1.00s、100ms超时；反向发生1.00s，松手发生2.00s。",
             "- 停止用本地参考速度计算各轴制动终点并清除待发送旧目标；不等同生产STOP协议。",
             "- far_goal仅检查滚动近端解，没有远端可达性预检；其未拒绝不代表最终目标可达。",
             "- 无用户精度合格线，以下仅报告事实，不因编译或模型自检成功宣称方案通过。", ""]
    for mode, report in reports.items():
        rows = [r for r in report["results"] if r["controller"] == "baseline"]
        lines += [f"## {mode}: {report['run_count']}组（以下历史表仅baseline）", "",
                  "下表为base60参数的全部初始姿态范围；拒绝次数包含无合法连续IK或超出30r/s上限的受控制动。出现提前拒绝的组，均速/CV含停下阶段，不能当正常巡航。", "",
                  "|场景|路径最大偏差 mm|巡航均速 mm/s|速度CV %|最终误差 mm|拒绝次数|",
                  "|---|---:|---:|---:|---:|---:|"]
        for case in sorted({r["case"] for r in rows}):
            group = [r for r in rows if r["case"] == case and r["variant"] == "base60"]
            def span(key):
                values = [r[key] for r in group]
                return f"{min(values):.3f}–{max(values):.3f}"
            lines.append(f"|{case}|{span('path_max_mm')}|{span('cruise_speed_mean_mm_s')}|{span('cruise_speed_cv_pct')}|{span('final_error_mm')}|{sum(r['rejected_updates'] for r in group)}|")
        lines += ["", "|参数组|常规直线最差偏差 mm|常规直线均速范围 mm/s|最差CV %|所有场景拒绝次数|", "|---|---:|---:|---:|---:|"]
        for variant in dict.fromkeys(r["variant"] for r in rows):
            all_group = [r for r in rows if r["variant"] == variant]
            group = [r for r in all_group if r["case"] in ("line_x", "line_xyz")]
            speeds = [r["cruise_speed_mean_mm_s"] for r in group]
            lines.append(f"|{variant}|{max(r['path_max_mm'] for r in group):.3f}|{min(speeds):.2f}–{max(speeds):.2f}|{max(r['cruise_speed_cv_pct'] for r in group):.2f}|{sum(r['rejected_updates'] for r in all_group)}|")
        lines += ["", "下面单列短距离例：20mm直线、约15mm滑块目标、0.25s快速改向。保留所有失败样例；低速有限目标/固定TCP移轨的观察时长相应延长，仍须同时查看planned_remaining_mm和final_error_mm。", "",
                  "|参数组|短直线最差偏差 mm|短滑块最差偏差 mm|短转弯最差偏差 mm|固定TCP移轨最差漂移 mm|这些样例拒绝次数|", "|---|---:|---:|---:|---:|---:|"]
        for variant in ("near20", "base60", "far120", "zero_skew", "period40", "speed10", "speed20"):
            group = [r for r in rows if r["variant"] == variant]
            def worst(cases):
                return max(r["path_max_mm"] for r in group if r["case"] in cases)
            cases = ("short_line_x", "short_line_xyz", "slider_short", "rapid_short_corner", "rail_hold_tcp")
            lines.append(f"|{variant}|{worst(('short_line_x','short_line_xyz')):.3f}|{worst(('slider_short',)):.3f}|{worst(('rapid_short_corner',)):.3f}|{worst(('rail_hold_tcp',)):.3f}|{sum(r['rejected_updates'] for r in group if r['case'] in cases)}|")
        lines += ["", "|base60场景/姿态|反向延迟 ms|继续前冲 mm|松手停止延迟 ms|松手后累计路程 mm|地轨最终 mm|最差余量 起→终|", "|---|---:|---:|---:|---:|---:|---:|"]
        for r in rows:
            if r["variant"] == "base60" and r["case"] in ("joystick_reverse", "joystick_corner", "rapid_short_reverse", "rapid_short_corner", "input_expiry", "rail_line", "rail_hold_tcp", "rail_recovery"):
                lines.append(f"|{r['case']}/f{r['fixture']}|{r['reverse_delay_ms']:.1f}|{r['reverse_extra_forward_mm']:.3f}|{r['stop_delay_ms']:.1f}|{r['stop_travel_mm']:.3f}|{r['rail_final_mm']:.3f}|{r['minimum_arm_margin_start']:.1%}→{r['minimum_arm_margin_end']:.1%}|")
        lines += ["", "|base60提前拒绝场景/姿态|时刻 s|原因|剩余计划路程 mm|", "|---|---:|---|---:|"]
        for r in rows:
            if r["variant"] == "base60" and r["rejected_updates"]:
                lines.append(f"|{r['case']}/f{r['fixture']}|{r['first_reject_at_s']:.3f}|{r['reject_reason']}|{r['planned_remaining_mm']:.3f}|")
        lines += ["", "完整逐组结果见本数学模式目录results.json，逐点数据见traces.csv；-1表示不适用或观察窗口内未检测到事件，提前停止的样例不报告成功反向/松手延迟。", ""]
    if "precise" in reports and "cmsis512" in reports:
        a = {(r['controller'], r['case'], r['variant'], r['fixture']): r for r in reports['precise']['results']}
        b = {(r['controller'], r['case'], r['variant'], r['fixture']): r for r in reports['cmsis512']['results']}
        difference = max(abs(a[k]['path_max_mm']-b[k]['path_max_mm']) for k in a)
        lines += ["## 两种主机数学模型对照", "", f"对应组最大路径偏差的最大差值为{difference:.6f}mm。CMSIS512为主机插值模型，不是ARM库位级一致性或硬件验证。", ""]
    (out / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def comparison_report(out: Path, reports: dict, identity: dict) -> None:
    lines = ["# 最小七轴协调 A/B 离线实验", "",
             "只修改cartesian_follow_lite_test.cpp与run_cartesian_follow_lite_test.py。",
             "电机规划参考模型；无PID/机械/真实CAN/RTOS，未测主控耗时；编译/模型自检通过不代表运动方案通过。", "",
             "A=baseline，保留原算法；B=coordinated。共同推进加速度上限400mm/s²、逐轴加速度/曲率约束，目标切向变化超过约20°先减速；这些是实验参数。",
             "B先保留原固定领先；另保留C=coordinated_brake_lead诊断版：按制动距离增加共同领先至最多300ms，用来检查提前制动和远前瞻失败之间的取舍。",
             "B用20ms位置回包差分滤波估速，位置误差调节速度上限；逐轴误差标尺0.30°/轨0.5mm，反馈过期80ms触发原型保底停止。",
             "B/C同时检查反馈TCP位置/朝向误差（标尺1mm/0.5°），逐轴或TCP误差超过12倍连续3更新则停止；这些门限仅为实验值。",
             "B正常松手继续共同减速，故障保底也只使用回包/估速。电机本地20kHz规划、0x07语义均未修改。",
             "地轨目标策略保持A不变，尚未加入完整25%回收锁存。本轮只验证执行协调，不把此策略当新通过项。",
             "路径误差主表使用独立用户意图路径，转向/松手制动包含在内；原控制器自定义路径指标另存JSON。",
             "必须同时看实际速度、完成情况和剩余距离，降低速度或提前停止不能单独算成功。", ""]
    comparison_rows = []
    for mode, report in reports.items():
        rows = report['results']
        lookup = {(r['controller'], r['case'], r['variant'], r['fixture']): r for r in rows}
        baseline = [r for r in rows if r['controller']=='baseline']
        pairs = [(a, lookup[('coordinated', a['case'], a['variant'], a['fixture'])]) for a in baseline]
        lost = sum(a['target_complete'] and not b['target_complete'] for a,b in pairs)
        gained = sum(not a['target_complete'] and b['target_complete'] for a,b in pairs)
        lost_nonfault=sum(a['target_complete'] and not b['target_complete'] for a,b in pairs if a['variant']!='feedback_dropout')
        complete = [(a,b) for a,b in pairs if a['target_complete'] and b['target_complete']]
        better = sum(b['intent_path_max_mm'] < a['intent_path_max_mm']*.9 for a,b in complete)
        worse = sum(b['intent_path_max_mm'] > a['intent_path_max_mm']*1.1 for a,b in complete)
        regression = identity['baseline_regression'][mode]
        lines += [f"## {mode}：{report['run_count']}组 / {len(pairs)}对", "",
                  f"历史基线回归：{json.dumps(regression,ensure_ascii=False) if not regression.get('passed') else str(regression['pairs'])+'组逐指标一致（数值容差内）'}。",
                  f"有限目标共同完成{len(complete)}对，其中独立路径最大误差改善>10%的{better}对、退化>10%的{worse}对。",
                  f"相对A，B失去完成的有限目标{lost}组，新增完成{gained}组；故障/不可达例必须结合原因另看。", "",
                  f"排除故意丢反馈组后，B失去完成{lost_nonfault}组。归档与当前完整输入身份相同：{regression.get('same_full_input_identity','未知')}；差异文件：{regression.get('input_differences',[])}。", "",
                  "### 基准20ms/60ms、40mm/s；三姿态范围，格式A→B", "",
                  "|场景|独立路径最大偏差 mm|窗口均速 mm/s|窗口CV %|有限目标完成 A/B|拒绝 A/B|",
                  "|---|---|---|---|---|---|"]
        for case in sorted({r['case'] for r in baseline}):
            aa=[r for r in baseline if r['case']==case and r['variant']=='base60']
            bb=[lookup[('coordinated',a['case'],a['variant'],a['fixture'])] for a in aa]
            def span(group,key):
                values=[r[key] for r in group]
                return f"{min(values):.3f}–{max(values):.3f}"
            done='—' if not aa[0]['finite_target'] else f"{sum(r['target_complete'] for r in aa)}/{sum(r['target_complete'] for r in bb)}（各{len(aa)}）"
            lines.append(f"|{case}|{span(aa,'intent_path_max_mm')} → {span(bb,'intent_path_max_mm')}|{span(aa,'cruise_speed_mean_mm_s')} → {span(bb,'cruise_speed_mean_mm_s')}|{span(aa,'cruise_speed_cv_pct')} → {span(bb,'cruise_speed_cv_pct')}|{done}|{sum(r['rejected_updates'] for r in aa)}/{sum(r['rejected_updates'] for r in bb)}|")
        lines += ["", "固定窗口包含协调器启动/转向减速时不能解释成纯匀速纹波；若一方提前拒绝，窗口含停下阶段。", "",
                  "### 参数敏感性：短线/短滑块/短转向＋松手/固定TCP移轨的三姿态最差误差", "",
                  "|参数|短线 A→B mm|短滑块 A→B mm|短转向 A→B mm|固定TCP移轨 A→B mm|短线/滑块完成 A/B（各9）|", "|---|---|---|---|---|---|"]
        for variant in dict.fromkeys(a['variant'] for a in baseline):
            aa=[r for r in baseline if r['variant']==variant]
            bb=[lookup[('coordinated',a['case'],a['variant'],a['fixture'])] for a in aa]
            def worst(group,cases):return max(r['intent_path_max_mm'] for r in group if r['case'] in cases)
            groups=[('short_line_x','short_line_xyz'),('slider_short',),('rapid_short_corner',),('rail_hold_tcp',)]
            values=[f"{worst(aa,g):.3f}→{worst(bb,g):.3f}" for g in groups]
            finite=('short_line_x','short_line_xyz','slider_short')
            done=f"{sum(r['target_complete'] for r in aa if r['case'] in finite)}/{sum(r['target_complete'] for r in bb if r['case'] in finite)}"
            lines.append('|'+variant+'|'+'|'.join(values)+'|'+done+'|')
        lines += ["", "### 基准短反向/转向及停止响应（逐姿态，A→B）", "",
                  "|场景/姿态|反向延迟 ms|继续前冲 mm|松手停止延迟 ms|松手后路程 mm|独立意图终误差 mm|", "|---|---|---|---|---|---|"]
        for a,b in pairs:
            if a['variant']=='base60' and a['case'] in ('rapid_short_reverse','rapid_short_corner','input_expiry'):
                fields=('reverse_delay_ms','reverse_extra_forward_mm','stop_delay_ms','stop_travel_mm','intent_end_error_mm')
                values=[f"{a[k]:.3f}→{b[k]:.3f}" for k in fields]
                lines.append('|'+a['case']+f"/f{a['fixture']}|"+'|'.join(values)+'|')
        lines += ["", "### 基准B未完成/拒绝项", "", "|场景/姿态|拒绝原因|剩余计划 mm|最终误差 mm|末速 mm/s|", "|---|---|---:|---:|---:|"]
        for a,b in pairs:
            if a['variant']=='base60' and (b['rejected_updates'] or b['finite_target'] and not b['target_complete']):
                lines.append(f"|{b['case']}/f{b['fixture']}|{b['reject_reason'] or '观察结束未满足完成条件'}|{b['planned_remaining_mm']:.3f}|{b['final_error_mm']:.3f}|{b['final_speed_mm_s']:.3f}|")
        for a,b in pairs:
            row={'math':mode,'case':a['case'],'variant':a['variant'],'fixture':a['fixture']}
            for k in ('intent_path_max_mm','path_max_mm','cruise_speed_mean_mm_s','cruise_speed_cv_pct','actual_travel_mm','intent_end_error_mm','target_complete','planned_remaining_mm','rejected_updates','reject_reason','max_ik_calls_per_update','guide_horizon_max_ms'):
                row['A_'+k]=a[k];row['B_'+k]=b[k]
            comparison_rows.append(row)
        lines += ["", f"B外部IK峰值{max(b['max_ik_calls_per_update'] for a,b in pairs)}次/更新（含轨策略），不包含解析IK内部候选FK，不能当主控实时预算。", "",
                  "### C制动前瞻诊断版（不替代B，基准三姿态）", "",
                  "|场景|独立路径最大偏差 mm|完成数|拒绝数|最大实际领先 ms|", "|---|---:|---:|---:|---:|"]
        for case in ('short_line_x','short_line_xyz','slider_short','rapid_short_corner','rail_line','rail_hold_tcp'):
            group=[r for r in rows if r['controller']=='coordinated_brake_lead' and r['variant']=='base60' and r['case']==case]
            lines.append(f"|{case}|{max(r['intent_path_max_mm'] for r in group):.3f}|{sum(r['target_complete'] for r in group) if group[0]['finite_target'] else '—'}|{sum(r['rejected_updates'] for r in group)}|{max(r['guide_horizon_max_ms'] for r in group):.1f}|")
        lines += ["", "C所有逐组数据也在results.json/traces.csv。增加领先可能更早遇到无解或12°分支连续门限；本轮没有通过放宽连续性门限消除失败。", ""]
    (out/'comparison.md').write_text('\n'.join(lines),encoding='utf-8')
    with (out/'comparisons.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(comparison_rows[0]));writer.writeheader();writer.writerows(comparison_rows)


def slowdown_report(out: Path, reports: dict, identity: dict) -> None:
    """Observational attribution, without changing limits, inputs or algorithms."""
    lines = ["# 降速原因与快速改向：只读诊断", "",
             "A/B/C运动算法与上轮逐项回归，完整旧traces.csv须字节相同。本轮仅增加观察；无反事实消融，不能把各限速贡献当成独立因果百分比。",
             "复用真实PositionTracker，但无PID/机械/真实CAN/RTOS；没有测F405计算耗时。", "",
             "统计口径：主控原因按一次更新后的保持时间计，终点最后一次更新计0ms；原因可以重叠，不能相加。",
             "巡航窗口沿用原定义，base60的rail_line为0.30<t<1.25s，其他速度组窗口随要求速度变化；仍需查看启动是否结束。电机分支逐20kHz tick精确累计。",
             "电机观察器读取内部参考位置/速度，仅作诊断；控制器仍只读取原位置回包/差分估速。每次调用检查分支与实际速度变化一致。",
             "feedback_scale统计误差倍率>1（即使输入已经为0）；joint/curvature统计候选速度界低于转向处理后的速度，并非每次都是最终主导项。",
             "速度预算表是原代码执行顺序的快照，反馈乘数作用在关节约束之后；不能解释成删掉某项就能恢复多少速度。",
             "control_updates.csv在更新结束、当前批次派送前取样；sent_goal是本次发送值，goal_before_dispatch是电机上一已收目标。motor_observations.csv为执行后1ms样本。",
             "轴1..7代表J1..J7；worst_source=8为TCP位置、9为TCP朝向、0为未计算。角度单位°，地轨mm，速度对应°/s或mm/s；cap不是带符号速度命令。",
             "沿线落后/横向偏离使用当前共同切向及当前回包TCP；固定TCP移轨的零切向情形不能据此作沿线/横向分解。", ""]
    aggregates, reason_rows, plot_data = {}, [], {}
    for mode, report in reports.items():
        by_key = {(r['controller'],r['case'],r['variant'],r['fixture']):r for r in report['results']}
        controls = {}
        with (out/mode/'control_updates.csv').open(encoding='utf-8',newline='') as stream:
            for raw in csv.DictReader(stream):
                if raw['case'] not in ('rail_line','rail_hold_tcp','rapid_short_reverse','rapid_short_corner'):
                    continue
                key=(raw['controller'],raw['case'],raw['variant'],int(raw['fixture']))
                controls.setdefault(key,[]).append({k:float(v) for k,v in raw.items() if k not in ('controller','case','variant')})
        motor = {}
        with (out/mode/'motor_observations.csv').open(encoding='utf-8',newline='') as stream:
            for raw in csv.DictReader(stream):
                if raw['variant']!='base60':
                    continue
                key=(raw['controller'],raw['case'],int(raw['fixture']))
                motor.setdefault(key,[]).append({k:(v if k.endswith('_reason') else float(v)) for k,v in raw.items() if k not in ('controller','case','variant')})
        def avg(group,field):
            denominator=sum(r['dt_ms'] for r in group)
            return sum(r[field]*r['dt_ms'] for r in group)/denominator if denominator else 0.
        def phase(group):return [r for r in group if r['active'] and r['cruise_window'] and r['dt_ms']>0]
        def ms(row,reason,p='cruise_window'):return row['diagnostics']['control_phases'][p]['reasons'][reason]['ms']
        lines += [f"## {mode}", "",
                  f"只读回归：{identity['observation_regression'][mode]['pairs']}组原指标最大变化{identity['observation_regression'][mode]['max_absolute_difference']}；旧轨迹字节一致。",
                  f"20kHz分支观察检查{identity['diagnostic_checks'][mode]['motor_branch_checks']}次，无不一致。", "",
                  "### 地轨配合：速度预算（B，base60；TCP要求40mm/s、轨要求60mm/s）", "",
                  "|姿态|输入→转向后→关节约束后→反馈后→终点约束后→共同速度 mm/s|实际TCP窗口均速|轨实际窗口均速|最大单轴速度限制触发ms|曲率限制触发ms|反馈缩速ms|终点制动ms|",
                  "|---|---|---:|---:|---:|---:|---:|---:|"]
        mode_data={"rail":[],"turns":[]}
        for f in range(3):
            key=('coordinated','rail_line','base60',f);r=by_key[key];g=phase(controls[key]);m=motor[('coordinated','rail_line',f)]
            window=[x for x in m if .30<x['t_s']<1.25]
            stages={k:avg(g,k) for k in ('input_rate','after_turn','after_joint','after_feedback','allowed','common')}
            worst={i:sum(x['dt_ms'] for x in g if x['worst_source']==i) for i in range(1,10)}
            dom=max(worst,key=worst.get)
            rail={'fixture':f,'stages':stages,'actual_tcp_speed':r['cruise_speed_mean_mm_s'],
                  'rail_speed_mean':sum(x['j7_oracle_velocity'] for x in window)/len(window),
                  'dominant_feedback_source':dom,'dominant_feedback_source_ms':worst[dom],
                  'worst_ratio_mean':avg(g,'worst_ratio'),'along_lag_mean_mm':avg(g,'along_lag_mm'),
                  'cross_lag_mean_mm':avg(g,'cross_lag_mm'),'j7_tracking_ratio_mean':avg(g,'j7_tracking_ratio'),
                  'feedback_factor_mean':avg(g,'feedback_factor'),
                  'joint_speed_bound_min':min(x['velocity_bound'] for x in g),'curvature_bound_min':min(x['curvature_bound'] for x in g),
                  'j7_motor_ms':r['diagnostics']['motor_phases']['cruise_window']['J7']}
            mode_data['rail'].append(rail)
            lines.append(f"|f{f}|"+' → '.join(f"{v:.3f}" for v in stages.values())+f"|{rail['actual_tcp_speed']:.3f}|{rail['rail_speed_mean']:.3f}|{ms(r,'joint_speed_bound'):.0f}|{ms(r,'curvature_speed_bound'):.0f}|{ms(r,'feedback_scale'):.0f}|{ms(r,'terminal_brake'):.0f}|")
        lines += ["", "### 地轨配合：反馈误差与电机本地分支", "",
                  "|姿态|反馈最常主导来源|平均最差倍率|平均沿线落后/横向偏离 mm|轨位置误差平均倍率|轨本地：距离制动/速度帽制动/加速/巡航 ms|",
                  "|---|---|---:|---|---:|---|"]
        for r in mode_data['rail']:
            source='J'+str(r['dominant_feedback_source']) if r['dominant_feedback_source']<=7 else ('TCP位置' if r['dominant_feedback_source']==8 else 'TCP朝向')
            b=r['j7_motor_ms']
            lines.append(f"|f{r['fixture']}|{source}，{r['dominant_feedback_source_ms']:.0f}ms|{r['worst_ratio_mean']:.3f}|{r['along_lag_mean_mm']:.3f} / {r['cross_lag_mean_mm']:.3f}|{r['j7_tracking_ratio_mean']:.3f}|{b['distance_brake']:.1f} / {b['cap_brake']:.1f} / {b['accelerating']:.1f} / {b['cruising']:.1f}|")
        lines += ["", "### A/B/C地轨诊断对照（base60）", "",
                  "|方案/姿态|实际TCP窗口均速|轨距离制动ms|反馈缩速ms|最大前瞻ms|完成/拒绝原因|", "|---|---:|---:|---:|---:|---|"]
        for c in ('baseline','coordinated','coordinated_brake_lead'):
            for f in range(3):
                r=by_key[(c,'rail_line','base60',f)];b=r['diagnostics']['motor_phases']['cruise_window']['J7']
                lines.append(f"|{c}/f{f}|{r['cruise_speed_mean_mm_s']:.3f}|{b['distance_brake']:.1f}|{ms(r,'feedback_scale'):.0f}|{r['guide_horizon_max_ms']:.1f}|{r['target_complete']} / {r['reject_reason'] or '无拒绝'}|")
        lines += ["", "### 快速改向时间线（base60，输入0.250s改变，0.500s松手）", "",
                  "新方向参考启动：共同切向与新指令点积>0.9且共同速度>0.05；实际新方向启动：处理新输入后20ms位移沿新方向的速度>1mm/s。-1表示松手前未出现；-2表示输入改变前已有该方向漂移，不能判为响应。",
                  "共同速度到零是参考状态，不能当作电机已经停下；事件时间分辨率分别为20ms更新和1ms FK采样。", "",
                  "|场景/方案/姿态|首次处理新输入s|共同速度首次≤0.05s|新方向参考启动s|实际新方向启动s|原反向延迟ms|转向制动累计/最长ms|改向后反馈缩速ms|0.25至0.50s新方向净位移mm|",
                  "|---|---:|---:|---:|---:|---:|---|---:|---:|"]
        for case in ('rapid_short_reverse','rapid_short_corner'):
            direction=(-1,0,0) if case.endswith('reverse') else (0,1,0)
            for c in ('baseline','coordinated','coordinated_brake_lead'):
                for f in range(3):
                    r=by_key[(c,case,'base60',f)];g=[x for x in controls[(c,case,'base60',f)] if .25<=x['t_s']<.5 and x['active']]
                    first=lambda rows,condition:next((x['t_s'] for x in rows if condition(x)),-1.)
                    zero=first(g,lambda x:x['common']<=.05) if c!='baseline' else -1.
                    ref=first(g,lambda x:x['common']>.05 and sum(x['direction_'+k]*v for k,v in zip('xyz',direction))>.9)
                    m=motor[(c,case,f)];actual=-1.
                    at=lambda t:min(m,key=lambda x:abs(x['t_s']-t))
                    p0,p1,pre=at(.25),at(.50),at(.23)
                    pre_speed=sum((p0[k+'_mm']-pre[k+'_mm'])*v for k,v in zip('xyz',direction))/.020
                    for i in range(20,len(m)):
                        x=m[i]
                        if g and g[0]['t_s']<=x['t_s']<.5 and sum((x[k+'_mm']-m[i-20][k+'_mm'])*v for k,v in zip('xyz',direction))/.020>1:
                            actual=x['t_s'];break
                    if pre_speed>1:actual=-2.
                    displacement=sum((p1[k+'_mm']-p0[k+'_mm'])*v for k,v in zip('xyz',direction))
                    stat=r['diagnostics']['control_phases']['turn_to_release']['reasons']['turn_brake']
                    item={'case':case,'controller':c,'fixture':f,'first_update_s':g[0]['t_s'] if g else -1,
                          'common_zero_s':zero,'new_reference_s':ref,'new_actual_s':actual,
                          'legacy_reverse_delay_ms':r['reverse_delay_ms'],'turn_brake_ms':stat['ms'],'turn_brake_longest_ms':stat['longest_ms'],
                          'feedback_scale_ms':ms(r,'feedback_scale','turn_to_release'),'new_direction_displacement_mm':displacement,
                          'prechange_new_direction_speed_mm_s':pre_speed}
                    mode_data['turns'].append(item)
                    lines.append(f"|{case}/{c}/f{f}|{item['first_update_s']:.3f}|{zero:.3f}|{ref:.3f}|{actual:.3f}|{r['reverse_delay_ms']:.1f}|{stat['ms']:.0f} / {stat['longest_ms']:.0f}|{item['feedback_scale_ms']:.0f}|{displacement:.3f}|")
        lines += ["", "### 原因复核：关键时刻（B，姿态0）", "",
                  "地轨在0.500s：距离与制动距离采用执行后1ms快照，下一个20kHz周期可能已经切换分支；完整分支累计见上表。", ""]
        m=motor[('coordinated','rail_line',0)];sample=min(m,key=lambda x:abs(x['t_s']-.5))
        gap=abs(sample['j7_goal']-sample['j7_position']);braking=sample['j7_oracle_velocity']**2/(2*150)
        lines += [f"轨速度{sample['j7_oracle_velocity']:.3f}mm/s，收到速度帽{sample['j7_cap']:.3f}mm/s；距电机目标{gap:.4f}mm，按本例150mm/s²得到制动距离{braking:.4f}mm。当前位置已接近本地制动边界，速度帽尚未用满。", "",
                  "要求轨60mm/s时，同一加速度下制动距离为12mm；固定60ms前瞻对应3.6mm轨行程，另有当前20ms小段及跟随误差，不能把3.6mm直接当成实际目标间距。", "",
                  "|场景/时刻|共同速度mm/s|共同加速度上限mm/s²|限制加速度的轴|曲率速度界mm/s|限制曲率的轴|", "|---|---:|---:|---|---:|---|"]
        for case in ('rapid_short_reverse','rapid_short_corner'):
            for t in (.24,.34,.36,.44,.48):
                x=min(controls[('coordinated',case,'base60',0)],key=lambda x:abs(x['t_s']-t))
                axis=lambda k:'共同上限' if x[k]==0 else 'J'+str(int(x[k]))
                lines.append(f"|{case}/{t:.2f}s|{x['common']:.3f}|{x['acceleration']:.3f}|{axis('acceleration_axis')}|{x['curvature_bound']:.3f}|{axis('curvature_axis')}|")
        lines += ["", "该姿态90°转向后共同加速度只约41mm/s²，主要由J4对应的局部约束压低；因此除先停再转外，新方向重新起步也很慢。这是当前有限差分/保守系数给出的限制，尚不能当成实机物理极限。", ""]
        lines += ["", "### 参数对照（地轨；姿态0）", "",
                  "|参数|B实际TCP均速|B共同速度均值|B反馈乘数均值|B轨距离制动ms|A实际TCP均速|", "|---|---:|---:|---:|---:|---:|"]
        for variant in dict.fromkeys(r['variant'] for r in report['results']):
            b=by_key[('coordinated','rail_line',variant,0)];a=by_key[('baseline','rail_line',variant,0)]
            g=phase(controls[('coordinated','rail_line',variant,0)])
            lines.append(f"|{variant}|{b['cruise_speed_mean_mm_s']:.3f}|{avg(g,'common'):.3f}|{avg(g,'feedback_factor'):.3f}|{b['diagnostics']['motor_phases']['cruise_window']['J7']['distance_brake']:.1f}|{a['cruise_speed_mean_mm_s']:.3f}|")
        for r in report['results']:
            for p,phase_stats in r['diagnostics']['control_phases'].items():
                for reason,stat in phase_stats['reasons'].items():
                    reason_rows.append({'math':mode,'controller':r['controller'],'case':r['case'],'variant':r['variant'],'fixture':r['fixture'],
                                        'phase':p,'reason':reason,'observed_ms':phase_stats['observed_ms'],**stat})
        aggregates[mode]=mode_data
        if mode=='precise' or len(reports)==1:plot_data={'controls':controls,'motor':motor}
    lines += ["", "## 判断边界", "",
              "这些统计能定位哪些代码实际执行、速度在哪个阶段被压低；电机距离制动可能与主控反馈缩速形成相互影响，不能按时间占比断言某单项占因果的多少。",
              "C只是已有的长前瞻对照，出现拒绝的均速含停止阶段；不能拿它直接证明增加前瞻已解决问题。",
              "下一步应按此证据逐项讨论，再决定是否授权优化；本轮没有修改转向逻辑、误差门限、前瞻策略、地轨分配或任何生产代码。", ""]
    (out/'slowdown_analysis.md').write_text('\n'.join(lines),encoding='utf-8')
    (out/'slowdown_analysis.json').write_text(json.dumps(aggregates,ensure_ascii=False,indent=2),encoding='utf-8')
    with (out/'slowdown_reasons.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(reason_rows[0]));writer.writeheader();writer.writerows(reason_rows)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    for c in ('baseline','coordinated','coordinated_brake_lead'):
        m=plot_data['motor'][(c,'rail_line',0)]
        axes[0,0].plot([x['t_s'] for x in m],[x['speed_20ms_mm_s'] for x in m],label=c)
        m=plot_data['motor'][(c,'rapid_short_reverse',0)]
        vx=[0.]*20+[(m[i]['x_mm']-m[i-20]['x_mm'])/.020 for i in range(20,len(m))]
        axes[1,0].plot([x['t_s'] for x in m],vx,label=c)
    g=plot_data['controls'][('coordinated','rail_line','base60',0)]
    for key in ('input_rate','after_joint','after_feedback','common'):
        axes[0,1].step([x['t_s'] for x in g],[x[key] for x in g],where='post',label=key)
    g=plot_data['controls'][('coordinated','rapid_short_reverse','base60',0)]
    axes[1,1].step([x['t_s'] for x in g],[x['common']*x['direction_x'] for x in g],where='post',label='signed reference speed')
    axes[1,1].step([x['t_s'] for x in g],[40*x['reason_turn_brake'] for x in g],where='post',label='turn brake flag x40',alpha=.65)
    titles=('Rail + arm: actual TCP speed','B rail speed budget','Rapid reversal: actual signed X speed','B reversal reference and turn brake')
    for ax,title in zip(axes.flat,titles):
        ax.set_title(title);ax.set_xlabel('time (s)');ax.set_ylabel('mm/s');ax.grid(alpha=.3);ax.legend(fontsize=7)
    axes[0,0].set_xlim(0,3.5);axes[0,1].set_xlim(0,1.5)
    for ax in axes[1]:ax.set_xlim(.15,.65);ax.axvline(.25,color='gray',linestyle=':');ax.axvline(.5,color='gray',linestyle=':')
    fig.suptitle('Read-only diagnostics; fixture 0; offline references, no MCU timing')
    fig.savefig(out/'slowdown_analysis.png',dpi=150);plt.close(fig)


def research_reference_check(report: dict, reference: dict) -> dict:
    gold={(r['case'],r['variant'],r['fixture']):r for r in reference['results'] if r['controller']=='coordinated'}
    differences=[];pairs=0
    for r in report['results']:
        if r['controller']!='r_h60_on':continue
        pairs+=1;a=gold[(r['case'],r['variant'],r['fixture'])]
        for field,value in a.items():
            if field!='controller' and r[field]!=value:differences.append([r['case'],r['variant'],r['fixture'],field])
    if differences or pairs!=147:raise RuntimeError('Research reference differs from unchanged B: '+str(differences[:5]))
    return {'passed':True,'pairs':pairs,'all_result_fields_identical_except_label':True}


def research_report(out: Path, reports: dict, identity: dict) -> None:
    lines=['# MoveL前瞻、反馈与转向：预注册离线消融','',
           '目的：为后续可能的论文保存可复现证据，当前不声称实机性能、实时预算或算法新颖性。协议在protocol.json及protocol_snapshot中冻结；原始A/B/C和旧产物保留。',
           '8控制设置×7参数组×7场景×3姿态=1176组/数学模式。固定前瞻60/120/240ms×反馈缩速开/关构成6组；另两组取消先停再转，仅用于诊断，不能作为可部署候选。',
           '禁用的是全局反馈缩速，位置/速度修正、硬误差停止、限位、IK检查等均保留；不存在“去掉所有反馈”的组。',
           '参考加速度倍率为连续qref边界差分得到的关节速度再差分，与模型物理加速度比较；不是实机测量，也不是段内严格界。不同实际速度、拒绝和少移动必须同时考虑。','']
    flat=[];pairs=[]
    for mode,report in reports.items():
        rows=report['results'];lookup={(r['controller'],r['case'],r['variant'],r['fixture']):r for r in rows}
        lines += [f'## {mode}','',f"原B匹配回归{identity['research_checks'][mode]['reference']['pairs']}组全部字段一致（仅控制器标签不同）。",'',
                  '### 全矩阵失败与参考动态约束','',
                  '|设置|有限目标完成/总数|拒绝组数与原因|参考离散加速度>1.01倍组数|最高倍率|',
                  '|---|---|---|---:|---:|']
        for c in RESEARCH_EXPERIMENTS:
            group=[r for r in rows if r['controller']==c];finite=[r for r in group if r['finite_target']]
            reasons={reason:sum(r['reject_reason']==reason for r in group) for reason in sorted({r['reject_reason'] for r in group if r['reject_reason']})}
            lines.append(f"|{c}|{sum(r['target_complete'] for r in finite)}/{len(finite)}|{sum(bool(r['rejected_updates']) for r in group)}：{json.dumps(reasons,ensure_ascii=False)}|{sum(r['reference_acceleration_ratio_max']>1.01 for r in group)}|{max(r['reference_acceleration_ratio_max'] for r in group):.3f}|")
        lines += ['', '### base60地轨配合：每个姿态完整列出','',
                  '|设置/姿态|TCP均速mm/s|独立路径最大误差mm|参考最大落后mm|轨距离制动ms|反馈缩速ms|完成|拒绝原因|',
                  '|---|---:|---:|---:|---:|---:|---|---|']
        for c in RESEARCH_EXPERIMENTS:
            for f in range(3):
                r=lookup[(c,'rail_line','base60',f)];d=r['diagnostics'];motor=d['motor_phases']['cruise_window']['J7'];control=d['control_phases']['cruise_window']['reasons']
                lines.append(f"|{c}/f{f}|{r['cruise_speed_mean_mm_s']:.3f}|{r['intent_path_max_mm']:.3f}|{r['reference_lag_max_mm']:.3f}|{motor['distance_brake']:.1f}|{control['feedback_scale']['ms']:.0f}|{r['target_complete']}|{r['reject_reason'] or '无'}|")
        lines += ['', '### base60快速改向：速度、偏差与参考动态一起观察','',
                  '|场景/设置/姿态|原定义反向延迟ms|独立路径最大偏差mm|意图终误差mm|实际总路程mm|参考加速度最高倍率|拒绝|',
                  '|---|---:|---:|---:|---:|---:|---|']
        for case in ('rapid_short_reverse','rapid_short_corner'):
            for c in ('r_h60_on','r_h60_off','r_turn_off','r_turn_off_no_feedback'):
                for f in range(3):
                    r=lookup[(c,case,'base60',f)]
                    lines.append(f"|{case}/{c}/f{f}|{r['reverse_delay_ms']:.1f}|{r['intent_path_max_mm']:.3f}|{r['intent_end_error_mm']:.3f}|{r['actual_travel_mm']:.3f}|{r['reference_acceleration_ratio_max']:.3f}|{r['reject_reason'] or '无'}|")
        lines += ['', '### base60固定世界TCP移轨：三姿态最差漂移','',
                  '|设置|最差漂移mm|完成数|拒绝数|','|---|---:|---:|---:|']
        for c in RESEARCH_EXPERIMENTS:
            group=[lookup[(c,'rail_hold_tcp','base60',f)] for f in range(3)]
            lines.append(f"|{c}|{max(r['intent_path_max_mm'] for r in group):.3f}|{sum(r['target_complete'] for r in group)}|{sum(bool(r['rejected_updates']) for r in group)}|")
        for r in rows:
            item={k:v for k,v in r.items() if k!='diagnostics'};item['math']=mode
            for key in ('turn_brake','feedback_scale','reference_acceleration_over_physical'):
                item[key+'_ms']=r['diagnostics']['control_phases']['all']['reasons'][key]['ms']
            flat.append(item)
            if r['controller'] in ('r_h60_off','r_h120_off','r_h240_off'):
                a=lookup[(r['controller'].replace('_off','_on'),r['case'],r['variant'],r['fixture'])]
                pairs.append({'math':mode,'case':r['case'],'variant':r['variant'],'fixture':r['fixture'],'horizon_ms':RESEARCH_EXPERIMENTS[r['controller']]['guide_ms'],
                              'on_complete':a['target_complete'],'off_complete':r['target_complete'],'on_reject':a['reject_reason'],'off_reject':r['reject_reason'],
                              'on_speed':a['cruise_speed_mean_mm_s'],'off_speed':r['cruise_speed_mean_mm_s'],
                              'on_error':a['intent_path_max_mm'],'off_error':r['intent_path_max_mm'],
                              'on_lag':a['reference_lag_max_mm'],'off_lag':r['reference_lag_max_mm']})
        lines += ['', '均速沿用既定窗口，失败组可能含停止阶段；这些组保留在表中但不能作为正常巡航性能。完整敏感性及匹配对照见CSV，不挑选通过样本。','']
    # A transparent necessary-distance estimate, not a substitute for the plant.
    envelope={'assumptions':'constant rail speed, linear rail path, no guide reset, zero dispatch delay; maximum guide distance immediately after update',
              'rail_request_mm_s':60,'rail_acceleration_mm_s2':150,'period_s':.020,
              'hard_error_scale_mm':.5,'hard_error_multiple':12,
              'required_braking_distance_mm':60**2/(2*150),'rows':[]}
    for h in (.060,.120,.240):
        lead=60*(h+.020)
        envelope['rows'].append({'horizon_ms':h*1000,'maximum_nominal_lead_mm':lead,
                                 'minimum_lag_to_reach_braking_distance_mm':max(0.,12-lead)})
    lines += ['## 制动距离的必要条件估算','',
              '目标轨速60mm/s、加速度150mm/s²时，制动距离12mm。60ms前瞻加20ms下一小段最多提供4.8mm名义领先；即使更新后立即派送，也需至少7.2mm落后才能让目标间距达到12mm。当前硬误差门限标尺0.5mm×12=6mm，且要求连续3次更新才停。',
              '这解释了短前瞻与误差门限存在潜在冲突；它只是一项理想化必要距离估算，不是对完整闭环稳定性或精确稳态速度的证明。120ms对应8.4mm名义领先、至少3.6mm落后；240ms对应15.6mm名义领先，但远端IK/连续性仍须独立检查。','',
              '## 研究边界','',
              '快速改向场景当前锁轨，不能把其中J4限制当成整个七轴系统能力。模型不含真实闭环/通信/调度；本轮不变更生产固件，也不形成论文新颖性结论。']
    (out/'research_analysis.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (out/'rail_distance_envelope.json').write_text(json.dumps(envelope,ensure_ascii=False,indent=2),encoding='utf-8')
    for name,rows in (('research_results.csv',flat),('research_feedback_pairs.csv',pairs)):
        with (out/name).open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def plots(out: Path, modes: list[str]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        (out / "plots_unavailable.txt").write_text("matplotlib unavailable; results and CSV are complete", encoding="utf-8")
        return
    mode = "precise" if "precise" in modes else modes[0]
    with (out / mode / "traces.csv").open(encoding="utf-8", newline="") as f:
        all_rows = [r for r in csv.DictReader(f) if r["fixture"] == "0"]
        rows = [r for r in all_rows if r['controller']=='baseline']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for variant in ("near20", "base60", "far120"):
        r = [x for x in rows if x["case"] == "short_line_xyz" and x["variant"] == variant]
        axes[0, 0].plot([float(x["t_s"]) for x in r], [float(x["speed_20ms_mm_s"]) for x in r], label=variant)
        r = [x for x in rows if x["case"] == "rapid_short_reverse" and x["variant"] == variant]
        if r:
            x0 = float(r[0]["x_mm"])
            axes[0, 1].plot([float(x["t_s"]) for x in r], [float(x["x_mm"])-x0 for x in r], label=variant)
    for case in ("rail_line", "rail_hold_tcp"):
        r = [x for x in rows if x["case"] == case and x["variant"] == "base60"]
        if r:
            x0,y0,z0 = (float(r[0][f"{k}_mm"]) for k in "xyz")
            axes[1, 0].plot([float(x["t_s"]) for x in r], [float(x["rail_mm"]) for x in r], label=case+" rail")
            axes[1, 0].plot([float(x["t_s"]) for x in r], [float(x["y_mm"])-y0 for x in r], linestyle="--", label=case+" TCP Y")
            if case == "rail_hold_tcp":
                error = [math.sqrt((float(x["x_mm"])-x0)**2+(float(x["y_mm"])-y0)**2+(float(x["z_mm"])-z0)**2) for x in r]
                axes[1, 1].plot([float(x["t_s"]) for x in r], error, label="fixed world TCP drift")
    labels = [("20 mm diagonal: TCP speed (20 ms window)","speed (mm/s)"),
              ("Joystick reverse at 0.25 s, release at 0.5 s","X displacement (mm)"),
              ("Rail and world TCP motion","displacement (mm)"),
              ("World TCP held while rail moves","position drift (mm)")]
    for ax, (title,ylabel) in zip(axes.flat,labels):
        ax.set_title(title);ax.set_xlabel("time (s)");ax.set_ylabel(ylabel);ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.suptitle("Offline PositionTracker references only; fixture 0; "+mode)
    fig.savefig(out / "overview.png", dpi=150)
    plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for controller in ('baseline','coordinated'):
        subset=[r for r in all_rows if r['controller']==controller and r['variant']=='base60']
        r=[x for x in subset if x['case']=='short_line_xyz']
        axes[0,0].plot([float(x['t_s']) for x in r],[float(x['speed_20ms_mm_s']) for x in r],label=controller)
        r=[x for x in subset if x['case']=='rapid_short_reverse'];x0=float(r[0]['x_mm'])
        axes[0,1].plot([float(x['t_s']) for x in r],[float(x['x_mm'])-x0 for x in r],label=controller)
        r=[x for x in subset if x['case']=='rail_hold_tcp']
        drift=[math.sqrt(sum((float(x[k+'_mm'])-float(x['ref_'+k+'_mm']))**2 for k in 'xyz')) for x in r]
        axes[1,0].plot([float(x['t_s']) for x in r],drift,label=controller)
        r=[x for x in subset if x['case']=='rapid_short_corner'];x0,y0=float(r[0]['x_mm']),float(r[0]['y_mm'])
        axes[1,1].plot([float(x['x_mm'])-x0 for x in r],[float(x['y_mm'])-y0 for x in r],label=controller)
    for ax,title,ylabel in zip(axes.flat,('20 mm diagonal TCP speed','Reverse at 0.25 s, release at 0.5 s','World TCP held, rail moves 40 mm','Short corner: XY projection'),('speed (mm/s)','X displacement (mm)','TCP drift (mm)','Y displacement (mm)')):
        ax.set_title(title);ax.set_ylabel(ylabel);ax.grid(alpha=.3);ax.legend()
    for ax in (axes[0,0],axes[0,1],axes[1,0]):ax.set_xlabel('time (s)')
    axes[0,0].set_xlim(0,2);axes[1,1].set_xlabel('X displacement (mm)');axes[1,1].axis('equal')
    fig.suptitle('Offline A/B; fixture 0; '+mode+'; no hardware or MCU timing')
    fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)


def guide_budget_reference_check(report: dict, mode: str) -> dict:
    reference=ARTIFACTS/'cartesian-follow-lite-20261015T065643160059Z'
    old=json.loads((reference/mode/'results.json').read_text(encoding='utf-8'))['results']
    gold={(r['case'],r['variant'],r['fixture']):r for r in old if r['controller']=='coordinated_brake_lead'}
    differences=[];pairs=0
    for row in report['results']:
        if row['controller']!='coordinated_brake_lead':continue
        pairs+=1;prior=gold[(row['case'],row['variant'],row['fixture'])]
        for field,value in prior.items():
            if row[field]!=value:differences.append([row['case'],row['variant'],row['fixture'],field])
    if pairs!=147 or differences:raise RuntimeError('Original C changed: '+str(differences[:5]))
    return {'passed':True,'pairs':pairs,'all_preexisting_result_fields_identical':True,'reference':str(reference)}


def guide_budget_report(out: Path, reports: dict, identity: dict) -> None:
    controllers=['coordinated_brake_lead','coordinated_brake_lead_midpoint']
    lines=['# 有界远guide连续验证：执行模型独立对照','',
           '预注册：2控制器×7配置×7场景×3姿态=294组/数学模式。原C全部控制保持，候选仅在远guide直接求解被branch_jump_over_12deg拒绝时补中点与端点重试；近点12°、限位、反馈、停止和电机接口不变。',
           '每次最多3个guide IK，近步和两个导数probe另计。几何链成功不保证独立电机实际走中间路径，也未修复原参考加速度/终点缺陷。所有失败保留；此报告不是硬件、F405实时或论文新颖性结论。','']
    all_rows=[];pairs=[]
    fields=['path_max_mm','intent_path_max_mm','cruise_speed_mean_mm_s','cruise_speed_cv_pct','reverse_delay_ms','final_error_mm','target_complete','reference_acceleration_ratio_max','max_ik_calls_per_update']
    # Use the actual result keys; a missing metric must fail rather than silently
    # export a different definition under a familiar name.
    for mode,report in reports.items():
        rows=report['results'];lookup={(r['controller'],r['case'],r['variant'],r['fixture']):r for r in rows}
        lines += [f'## {mode}','',f"原C回归{identity['guide_budget_checks'][mode]['reference']['pairs']}组旧字段一致。",'',
                  '|控制器|有限完成/105|拒绝/147|参考a>1.01倍组数|最高参考a倍率|重试更新/成功|最高guide IK/总IK|',
                  '|---|---:|---:|---:|---:|---:|---:|']
        for c in controllers:
            group=[r for r in rows if r['controller']==c];finite=[r for r in group if r['finite_target']]
            lines.append(f"|{c}|{sum(bool(r['target_complete']) for r in finite)}/{len(finite)}|{sum(r['rejected_updates']>0 for r in group)}|{sum(r['reference_acceleration_ratio_max']>1.01 for r in group)}|{max(r['reference_acceleration_ratio_max'] for r in group):.6f}|{sum(r['guide_retry_updates'] for r in group)}/{sum(r['guide_retry_successes'] for r in group)}|{max(r['guide_ik_peak'] for r in group)}/{max(r['max_ik_calls_per_update'] for r in group)}|")
            reasons={}
            for r in group:
                if r['rejected_updates']:reasons[r['reject_reason']]=reasons.get(r['reject_reason'],0)+1
                all_rows.append({'math':mode,**{k:v for k,v in r.items() if k!='diagnostics'}})
            lines.append('')
            lines.append(c+'拒绝原因：'+json.dumps(reasons,ensure_ascii=False))
            lines.append('')
        lines += ['默认rail_line逐姿态（同一主窗口，包含启动，不另选有利时窗）：','',
                  '|姿态|控制器|完成|拒绝原因|TCP均速mm/s|TCP路径max mm|参考a峰值倍率|',
                  '|---|---|---|---|---:|---:|---:|']
        for f in range(3):
            for c in controllers:
                r=lookup[c,'rail_line','base60',f]
                lines.append(f"|{f}|{c}|{r['target_complete']}|{r['reject_reason']}|{r['cruise_speed_mean_mm_s']:.6f}|{r['path_max_mm']:.6f}|{r['reference_acceleration_ratio_max']:.6f}|")
        for c,case,var,f in lookup:
            if c!=controllers[1]:continue
            a=lookup[controllers[0],case,var,f];b=lookup[c,case,var,f]
            pair={'math':mode,'case':case,'variant':var,'fixture':f,'old_reject_reason':a['reject_reason'],'new_reject_reason':b['reject_reason'],
                  'old_rejected':a['rejected_updates'],'new_rejected':b['rejected_updates'],'guide_retry_updates':b['guide_retry_updates'],'guide_retry_successes':b['guide_retry_successes']}
            for field in fields:pair['old_'+field]=a[field];pair['new_'+field]=b[field]
            pairs.append(pair)
        lines.append('')
    for name,rows in [('guide_budget_results.csv',all_rows),('guide_budget_pairs.csv',pairs)]:
        with (out/name).open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (out/'guide_budget_analysis.md').write_text('\n'.join(lines),encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--math", choices=("precise", "cmsis512", "both"), default="both")
    selection=parser.add_mutually_exclusive_group()
    selection.add_argument("--research", action="store_true", help="Also run the preregistered 1176-case ablation per math mode")
    selection.add_argument("--guide-budget-only",action="store_true",help="Only run original C versus bounded-midpoint C, 294 cases per math mode")
    args = parser.parse_args()
    ARTIFACTS.mkdir(exist_ok=True)
    out = ARTIFACTS / ("cartesian-follow-lite-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    out.mkdir()
    before, formal_before = source_identity(), formal_identity()
    protected_before={p.as_posix():digest(REPO/p) for p in PROTECTED}
    identity = {"before": before, "formal_before": formal_before,
                "hardware": False, "production_written": False, "mcu_timing": False,
                "python": sys.version, "platform": platform.platform(), "builds": [], "output": str(out),
                "protected_before":protected_before,"baseline_regression":{},"observation_regression":{},"diagnostic_checks":{},"research_checks":{},"guide_budget_checks":{},
                "argv":sys.argv,"started_utc":datetime.now(timezone.utc).isoformat(),"future_paper_purpose":True}
    status = 1
    try:
        snapshot = out / "snapshot"
        for relative in INPUTS:
            dest = snapshot / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(REPO / relative, dest)
            if digest(dest) != before[relative.as_posix()]:
                raise RuntimeError("Source changed while freezing snapshot: " + str(relative))
        for relative in PROTECTED:
            dest=out/'protocol_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(REPO/relative,dest)
            if digest(dest)!=protected_before[relative.as_posix()]:raise RuntimeError('Research document changed during snapshot')
        protocol={'purpose':'Reproducible offline research for a possible later paper; no novelty or hardware claim',
                  'created_before_runs_utc':datetime.now(timezone.utc).isoformat(),'research_enabled':args.research,
                  'experiments':RESEARCH_EXPERIMENTS,'variants':RESEARCH_VARIANTS,'cases':RESEARCH_CASES,'fixtures':[0,1,2],
                  'expected_research_count_per_math':1176,'default_abc_count_per_math':1548,
                  'guide_budget_only':args.guide_budget_only,'guide_budget_count_per_math':294,
                  'guide_budget_rule':'Original adaptive C versus C with direct-first guide; only branch_jump retries midpoint then endpoint, max3 guide IK, strict near-step checks unchanged.',
                  'controls_held':'All old B controls and hard checks preserved, except explicitly factored knobs. No motor changes.',
                  'metrics':'Keep failures and completion, path error, actual speed/travel, lag, response delay, reference finite-difference dynamics together.',
                  'randomness':'None: deterministic fixtures and inputs; fixture variation is not a statistical replication.',
                  'production_identity':before,'document_identity':protected_before,'command':sys.argv}
        (out/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2),encoding='utf-8')
        validate_assumptions(snapshot)
        reports = {};research_reports={}
        modes = ["precise", "cmsis512"] if args.math == "both" else [args.math]
        for mode in modes:
            folder = out / mode
            executable, metadata = build(snapshot, folder, mode)
            metadata['executable_sha256']=digest(executable)
            identity["builds"].append(metadata)
            print("Running frozen host model:", mode, flush=True)
            command=[str(executable),str(folder)]+(['guide-budget'] if args.guide_budget_only else [])
            process = subprocess.run(command, cwd=folder, capture_output=True)
            (folder / "run.log").write_bytes(process.stdout + process.stderr)
            if process.returncode:
                raise RuntimeError(f"Model failed ({process.returncode}); inspect {folder / 'run.log'}")
            report = json.loads((folder / "results.json").read_text(encoding="utf-8"))
            if args.guide_budget_only:
                expected={(c,case,var,f) for c in ['coordinated_brake_lead','coordinated_brake_lead_midpoint'] for case in RESEARCH_CASES for var in RESEARCH_VARIANTS for f in range(3)}
                actual={(r['controller'],r['case'],r['variant'],r['fixture']) for r in report['results']}
                if not report.get('model_checks') or report.get('run_count')!=294 or len(report['results'])!=294 or actual!=expected:
                    raise RuntimeError('Incomplete guide-budget matrix')
                for row in report['results']:
                    if any(isinstance(x,float) and not math.isfinite(x) for x in row.values()):raise RuntimeError('Nonfinite guide-budget result')
                    if row['guide_ik_peak']>3 or row['max_ik_calls_per_update']>6:raise RuntimeError('Guide IK budget exceeded')
                    if row['guide_retry_successes']>row['guide_retry_updates'] or row['guide_retry_extra_ik']>2*row['guide_retry_updates']:
                        raise RuntimeError('Guide retry accounting mismatch')
                reports[mode]=report
                identity['guide_budget_checks'][mode]={'reference':guide_budget_reference_check(report,mode),'diagnostics':validate_diagnostics(report),
                    'exports':validate_diagnostic_exports(folder,report),'command':command,'guide_ik_limit':3,'whole_update_ik_limit':6}
                (folder/'guide_budget_checks.json').write_text(json.dumps(identity['guide_budget_checks'][mode],ensure_ascii=False,indent=2),encoding='utf-8')
                print('Completed and checked guide-budget:',mode,flush=True)
                continue
            if not report.get("model_checks") or report.get("run_count") != 1548 or len(report["results"]) != 1548:
                raise RuntimeError("Incomplete model run")
            for row in report["results"]:
                if any(isinstance(x, float) and not math.isfinite(x) for x in row.values()):
                    raise RuntimeError("Non-finite result")
            reports[mode] = report
            identity['baseline_regression'][mode]=compare_legacy(out,mode,report,identity)
            identity['observation_regression'][mode]=compare_observation_only(out,mode,report,identity)
            identity['diagnostic_checks'][mode]=validate_diagnostics(report)
            identity['diagnostic_checks'][mode]['exports']=validate_diagnostic_exports(folder,report)
            if args.research:
                research_folder=folder/'research';research_folder.mkdir()
                print('Running preregistered ablation:',mode,flush=True)
                command=[str(executable),str(research_folder),'research']
                process=subprocess.run(command,cwd=research_folder,capture_output=True)
                (research_folder/'run.log').write_bytes(process.stdout+process.stderr)
                if process.returncode:raise RuntimeError('Research model failed; inspect '+str(research_folder/'run.log'))
                research=json.loads((research_folder/'results.json').read_text(encoding='utf-8'))
                if research.get('run_count')!=1176 or len(research['results'])!=1176:raise RuntimeError('Incomplete research matrix')
                expected={(c,case,var,f) for c in RESEARCH_EXPERIMENTS for case in RESEARCH_CASES for var in RESEARCH_VARIANTS for f in range(3)}
                actual={(r['controller'],r['case'],r['variant'],r['fixture']) for r in research['results']}
                if expected!=actual:raise RuntimeError('Research matrix selection differs from protocol')
                research_reports[mode]=research
                identity['research_checks'][mode]={'reference':research_reference_check(research,report),'diagnostics':validate_diagnostics(research),
                                                   'exports':validate_diagnostic_exports(research_folder,research),'command':command}
                (research_folder/'checks.json').write_text(json.dumps(identity['research_checks'][mode],ensure_ascii=False,indent=2),encoding='utf-8')
            print('Completed and checked:',mode,flush=True)
        if args.guide_budget_only:
            guide_budget_report(out,reports,identity)
        else:
            write_report(out, reports)
            comparison_report(out,reports,identity)
            slowdown_report(out,reports,identity)
            plots(out, modes)
            if args.research:research_report(out,research_reports,identity)
        status = 0
    except Exception as exc:
        identity["error"] = str(exc)
        print("ERROR:", exc, file=sys.stderr)
    finally:
        identity["after"], identity["formal_after"] = source_identity(), formal_identity()
        identity["sources_unchanged"] = identity["after"] == before
        identity["formal_artifacts_unchanged"] = identity["formal_after"] == formal_before
        identity['protected_after']={p.as_posix():digest(REPO/p) for p in PROTECTED}
        identity['protected_unchanged']=identity['protected_after']==protected_before
        if not identity["sources_unchanged"] or not identity["formal_artifacts_unchanged"] or not identity['protected_unchanged']:
            status = 3
        identity["exit_code"] = status
        identity['ended_utc']=datetime.now(timezone.utc).isoformat()
        (out / "source_identity.json").write_text(json.dumps(identity, ensure_ascii=False, indent=2), encoding="utf-8")
        evidence={p.relative_to(out).as_posix():{'sha256':digest(p),'bytes':p.stat().st_size} for p in sorted(out.rglob('*')) if p.is_file() and p.name!='evidence_manifest.json'}
        (out/'evidence_manifest.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
    print("Output:", out)
    print("Production/test source identity unchanged:", identity["sources_unchanged"])
    print("Existing formal artifacts unchanged:", identity["formal_artifacts_unchanged"])
    print("Existing instructions/discussion documents unchanged:",identity['protected_unchanged'])
    return status


if __name__ == "__main__":
    raise SystemExit(main())
