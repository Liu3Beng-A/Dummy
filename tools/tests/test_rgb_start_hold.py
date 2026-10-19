"""Exercise production RGB::Update/GetStatus STARTING scene continuity.

Compiles complete rgb.cpp with Windows Visual Studio C++ tools and deterministic
HAL/RTOS shims. Rendering, selection, blending, brightness and wire conversion
are production code. No Flash operation, device, DMA timing, RTOS concurrency or
STM32F405 execution time is exercised.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

from test_rgb_motion_firmware import HAL


HARNESS = r'''
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define private public
#include "rgb.hpp"
#undef private
TIM_HandleTypeDef htim2;
HAL_StatusTypeDef HAL_FLASH_Unlock() { std::abort(); }
HAL_StatusTypeDef HAL_FLASH_Lock() { std::abort(); }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef*, uint32_t*) { std::abort(); }
HAL_StatusTypeDef HAL_FLASH_Program(unsigned, uint32_t, uint32_t) { std::abort(); }
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef*, unsigned, uint32_t*, unsigned) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef*, unsigned) { return HAL_OK; }
namespace TcpTool { void RefreshStorageProof() { std::abort(); } }
#include "rgb.cpp"

static unsigned checks=0, pairs=0;
static void check(bool value, const char* message) {
    ++checks;
    if (!value) { std::fprintf(stderr,"FAIL: %s\n",message); std::exit(1); }
}
static void tick(RGB& rgb, RGB::State state, uint32_t now) {
    rgb.Update(state,now);
    if (rgb.dmaBusy) rgb.Interrupt(1);
}
static void apply(RGB& rgb, const RGB::Config& config) {
    uint32_t revision=0; rgb.GetConfig(&revision);
    check(rgb.ApplyConfig(config,revision),"configuration accepted");
}
static RGB::Config config(bool automatic=true) {
    auto c=RGB::Defaults(); c.automatic=automatic;
    c.fadeOnMs=c.fadeOffMs=c.brightnessMs=0;
    c.transitionMs=300;
    // A deliberately distinct legacy STARTING cue must never leak into output.
    c.bindings[RGB::STARTING]={RGB::PURE_COLOR_0,1,{13,79,237}};
    c.colors[0]={190,27,46};
    c.manualMode=RGB::RAINBOW;
    return c;
}
static void equalFrame(const RGB& a,const RGB& b,const char* message) {
    ++pairs;
    check(std::memcmp(a.frame,b.frame,sizeof(a.frame))==0,message);
    check(std::memcmp(a.lastWire,b.lastWire,sizeof(a.lastWire))==0,"wire remains identical to uninterrupted original scene");
    check(a.brightness==b.brightness,"brightness ramp remains identical");
}
static void closeFrame(const float (&actual)[RGB::LED_COUNT][3],
                       const float (&expected)[RGB::LED_COUNT][3],const char* message) {
    for (unsigned i=0;i<RGB::LED_COUNT;++i) for (unsigned ch=0;ch<3;++ch)
        check(std::fabs(actual[i][ch]-expected[i][ch])<0.0001f,message);
}
static void statusIs(RGB& rgb,RGB::State state,unsigned mode,unsigned source,const char* message) {
    const auto s=rgb.GetStatus(state);
    check(s.state==state && s.mode==mode && s.source==source,message);
}

static void testColdStarting() {
    for (bool automatic:{false,true}) {
        RGB actual, reference; auto c=config(automatic);
        apply(actual,c); apply(reference,c);
        const auto fallback=RGB::SelectScene(c,RGB::DISABLED);
        check(!actual.sceneValid && actual.bootPending,"cold instance has no rendered scene");
        statusIs(actual,RGB::STARTING,fallback.mode,fallback.source,"query before first RGB tick uses disabled/manual fallback");
        check(!actual.sceneValid && actual.bootPending,"status query does not create or advance scene");
        reference.EndBoot();
        for (unsigned now=0;now<=5000;now+=20) {
            tick(actual,RGB::STARTING,now); tick(reference,RGB::DISABLED,now);
            equalFrame(actual,reference,"cold STARTING renders disabled/manual exactly");
            statusIs(actual,RGB::STARTING,fallback.mode,fallback.source,"cold STARTING remains reported while fallback is visible");
        }
        check(actual.sceneAt==0 && actual.blendAt==0,"cold fallback phase does not reset on repeated STARTING ticks");
    }
}

static void testHeldScene(RGB::State previous,bool automatic,bool retainBoot=false) {
    RGB actual, reference; auto c=config(automatic);
    apply(actual,c); apply(reference,c);
    if (!retainBoot) { actual.EndBoot(); reference.EndBoot(); }
    for (unsigned now=0;now<=460;now+=20) {
        tick(actual,previous,now); tick(reference,previous,now);
    }
    const auto before=actual.scene;
    const auto sceneAt=actual.sceneAt, blendAt=actual.blendAt;
    const auto blendMs=actual.blendMs;
    actual.EndBoot(); // Accepted robot START can end boot before the RGB tick.
    statusIs(actual,RGB::STARTING,before.mode,before.source,"pre-tick STARTING query reports retained scene");
    for (unsigned now=480;now<=4600;now+=20) {
        tick(actual,RGB::STARTING,now); tick(reference,previous,now);
        equalFrame(actual,reference,"STARTING keeps original animation phase");
        check(RGB::SameScene(actual.scene,before),"STARTING does not select separate binding");
        check(actual.sceneAt==sceneAt && actual.blendAt==blendAt && actual.blendMs==blendMs,
              "STARTING does not restart phase or existing transition");
        statusIs(actual,RGB::STARTING,before.mode,before.source,"visible source and real STARTING state are distinct");
        check(actual.GetStatus().state==RGB::STARTING,"default query retains latest robot state");
    }
}

static void testInFlightBlend() {
    RGB actual, reference; const auto c=config();
    apply(actual,c); apply(reference,c); actual.EndBoot(); reference.EndBoot();
    tick(actual,RGB::DISABLED,0); tick(reference,RGB::DISABLED,0);
    tick(actual,RGB::DISABLED,80); tick(reference,RGB::DISABLED,80);
    tick(actual,RGB::READY_INT,100); tick(reference,RGB::READY_INT,100);
    tick(actual,RGB::READY_INT,160); tick(reference,RGB::READY_INT,160);
    check(actual.blendMs==300 && actual.blendAt==100,"fixture has an active scene transition");
    float old[RGB::LED_COUNT][3]; std::memcpy(old,actual.frame,sizeof(old));
    for (unsigned now=180;now<=900;now+=20) {
        tick(actual,RGB::STARTING,now); tick(reference,RGB::READY_INT,now);
        equalFrame(actual,reference,"in-flight transition completes unchanged through STARTING");
        check(actual.blendAt==100 && actual.sceneAt==100,"in-flight transition clock retained");
    }
    check(std::memcmp(old,actual.frame,sizeof(old))!=0,"held animation advances rather than freezing one frame");
}

static void testSuccessUsesExistingTransition() {
    RGB actual; auto c=config(); apply(actual,c); actual.EndBoot();
    tick(actual,RGB::DISABLED,0); tick(actual,RGB::DISABLED,440);
    tick(actual,RGB::STARTING,460); tick(actual,RGB::STARTING,980);
    float previous[RGB::LED_COUNT][3]; std::memcpy(previous,actual.frame,sizeof(previous));
    tick(actual,RGB::READY_INT,1000);
    closeFrame(actual.frame,previous,"successful enable starts transition from last displayed frame");
    check(actual.blendAt==1000 && actual.blendMs==c.transitionMs,"success uses configured transition duration");
    statusIs(actual,RGB::READY_INT,RGB::AUTO_INT_COLOR,RGB::READY_INT,"success status selects normal configured ready binding");
    float target[RGB::LED_COUNT][3];
    actual.Render(actual.scene,150,target);
    for (unsigned i=0;i<RGB::LED_COUNT;++i) for (unsigned ch=0;ch<3;++ch)
        target[i][ch]=(target[i][ch]+previous[i][ch])*0.5f;
    tick(actual,RGB::READY_INT,1150);
    closeFrame(actual.frame,target,"halfway scene transition preserves original smooth interpolation");
    actual.Render(actual.scene,300,target);
    tick(actual,RGB::READY_INT,1300);
    closeFrame(actual.frame,target,"success transition reaches configured ready scene");

    RGB manual; c=config(false); c.manualMode=RGB::PURE_COLOR_1;
    apply(manual,c); manual.EndBoot(); tick(manual,RGB::DISABLED,0);
    tick(manual,RGB::STARTING,50); const auto start=manual.sceneAt;
    tick(manual,RGB::READY_INT,100);
    check(manual.sceneAt==start,"manual scene remains continuous after successful enable");
    statusIs(manual,RGB::READY_INT,RGB::PURE_COLOR_1,RGB::STATE_COUNT,"success returns normal manual status");
}

static void testSafetyOverrides() {
    for (bool automatic:{false,true}) for (const auto safety:{RGB::FAULT,RGB::STOP}) {
        RGB actual; auto c=config(automatic); apply(actual,c); actual.EndBoot();
        tick(actual,RGB::DISABLED,0); tick(actual,RGB::DISABLED,180);
        tick(actual,RGB::STARTING,200);
        const auto selected=RGB::SelectScene(c,safety);
        statusIs(actual,safety,selected.mode,selected.source,"FAULT/STOP query takes priority before RGB tick");
        tick(actual,safety,220);
        check(RGB::SameScene(actual.scene,selected) && actual.blendMs==0,"FAULT/STOP immediately replace held scene without blending");
        float expected[RGB::LED_COUNT][3]; actual.Render(selected,0,expected);
        closeFrame(actual.frame,expected,"FAULT/STOP frame is not tinted by retained scene");
    }
}

static void testGlobalControlsWhileHeld() {
    for (bool automatic:{false,true}) {
        RGB actual, reference; auto c=config(automatic);
        c.fadeOnMs=200; c.fadeOffMs=180; c.brightnessMs=160;
        apply(actual,c); apply(reference,c); actual.EndBoot(); reference.EndBoot();
        tick(actual,RGB::DISABLED,0); tick(reference,RGB::DISABLED,0);
        for (unsigned now=20;now<=400;now+=20) {
            tick(actual,RGB::STARTING,now); tick(reference,RGB::DISABLED,now);
        }
        for (const unsigned level:{35U,0U,70U}) {
            c.brightness=uint8_t(level); apply(actual,c); apply(reference,c);
            for (unsigned now=420;now<=760;now+=20) {
                const unsigned at=now+(level==35?0:level==0?400:800);
                tick(actual,RGB::STARTING,at); tick(reference,RGB::DISABLED,at);
                equalFrame(actual,reference,"global brightness ramp still applies during STARTING");
            }
            check(actual.status.currentBrightness==level,"held brightness reaches requested percentage");
            if (!level) {
                statusIs(actual,RGB::STARTING,actual.scene.mode,RGB::STATE_COUNT+1,"zero brightness reports OFF source but real STARTING state");
                check(std::all_of(std::begin(actual.lastWire),std::end(actual.lastWire),[](uint8_t b){return !b;}),"zero brightness outputs all zero wire bytes");
            }
        }
        actual.SetEnabled(false); reference.SetEnabled(false);
        statusIs(actual,RGB::STARTING,actual.scene.mode,RGB::STATE_COUNT+1,"global off query takes effect before tick");
        for (unsigned now=1600;now<=1900;now+=20) {
            tick(actual,RGB::STARTING,now); tick(reference,RGB::DISABLED,now);
            equalFrame(actual,reference,"global off uses existing fade during STARTING");
        }
        check(actual.brightness==0,"global off fade reaches zero");
        check(std::all_of(std::begin(actual.lastWire),std::end(actual.lastWire),[](uint8_t b){return !b;}),"global off wire bytes are zero");
    }
}

static void testSceneEditsDoNotReplaceHeldCue() {
    RGB actual, reference; auto c=config(false);
    apply(actual,c); apply(reference,c); tick(actual,RGB::DISABLED,0); tick(reference,RGB::DISABLED,0);
    tick(actual,RGB::STARTING,100); tick(reference,RGB::DISABLED,100);
    const auto old=actual.scene;
    c.manualMode=RGB::PURE_COLOR_1; c.colors[1]={31,173,53}; apply(actual,c);
    statusIs(actual,RGB::STARTING,old.mode,old.source,"editing manual preset cannot replace retained scene before tick");
    for (unsigned now=120;now<=500;now+=20) {
        tick(actual,RGB::STARTING,now); tick(reference,RGB::DISABLED,now);
        equalFrame(actual,reference,"scene edits are deferred while STARTING retains previous cue");
    }
    tick(actual,RGB::READY_INT,520);
    statusIs(actual,RGB::READY_INT,RGB::PURE_COLOR_1,RGB::STATE_COUNT,"new manual preset applies after success");
}

int main() {
    testColdStarting();
    for (bool automatic:{false,true}) {
        testHeldScene(RGB::DISABLED,automatic);
        testHeldScene(RGB::RUN_INT,automatic);
    }
    testHeldScene(RGB::DISABLED,true,true);
    testInFlightBlend(); testSuccessUsesExistingTransition(); testSafetyOverrides();
    testGlobalControlsWhileHeld(); testSceneEditsDoNotReplaceHeldCue();
    std::printf("{\"checks\":%u,\"compared_frame_pairs\":%u,\"hardware\":false,\"mcu_timing\":false,\"rtos_concurrency\":false}\n",checks,pairs);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--source-dir", type=Path,
                        help="Optional frozen Bsp/gpio directory containing rgb.cpp/.hpp/rainbow.hpp")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    core = (args.source_dir or repo / "firmware_new/dummy-ref-core-fw/Bsp/gpio").resolve()
    inputs = [core / name for name in ("rgb.cpp", "rgb.hpp", "rgb_rainbow.hpp")]
    hashes = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    before = hashes()
    out = (args.output or Path(tempfile.mkdtemp(prefix="rgb-start-hold-"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    for p in inputs:
        (out / p.name).write_bytes(p.read_bytes())
    (out / "tim.h").write_text(HAL, encoding="utf-8")
    (out / "FreeRTOS.h").write_text(
        "#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n",
        encoding="utf-8")
    (out / "task.h").write_text("#pragma once\n", encoding="utf-8")
    cpp = out / "rgb_start_hold.cpp"
    cpp.write_text(HARNESS, encoding="utf-8")
    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    vs = subprocess.check_output([
        str(vswhere), "-latest", "-products", "*", "-requires",
        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"
    ], text=True).strip()
    vcvars = Path(vs) / "VC/Auxiliary/Build/vcvars64.bat"
    cmd = out / "build.cmd"
    cmd.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                   + f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise /I"{out}" "{cpp}" /Fe:rgb_start_hold.exe\nexit /b %errorlevel%\n')
    built = subprocess.run(["cmd.exe", "/d", "/c", str(cmd)], cwd=out, capture_output=True)
    (out / "build.log").write_bytes(built.stdout + built.stderr)
    if built.returncode:
        raise RuntimeError(f"Native compile failed; see {out / 'build.log'}\n{built.stdout.decode(errors='replace')}\n{built.stderr.decode(errors='replace')}")
    run = subprocess.run([str(out / "rgb_start_hold.exe")], cwd=out, capture_output=True, text=True)
    (out / "run.log").write_text(run.stdout + run.stderr, encoding="utf-8")
    if run.returncode:
        raise RuntimeError(f"Native checks failed; see {out / 'run.log'}\n{run.stdout}\n{run.stderr}")
    result = json.loads(run.stdout)
    assert before == hashes(), "Concurrent production edit; rerun against final sources"
    result["input_sha256"] = before
    result["compiler"] = "MSVC /std:c++17 /O2 /fp:precise"
    result["production_coverage"] = ["complete rgb.cpp", "Update", "GetStatus", "Render", "SendFrame"]
    result["scenarios"] = [
        "cold first STARTING auto and manual fallback", "automatic/manual continuing phase and BOOT retention",
        "in-flight scene blend completion", "query before next RGB tick", "normal READY transition",
        "FAULT/STOP immediate override", "global brightness and OFF ramps", "scene edits held until success",
    ]
    result["limitations"] = [
        "HAL/FreeRTOS and DMA completion are deterministic stubs",
        "No Flash operation, device connection or physical output",
        "No STM32F405 timing, RTOS scheduling or concurrent interleaving proof",
    ]
    (out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"Native output: {out}")


if __name__ == "__main__":
    main()
