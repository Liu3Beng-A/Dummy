#include "common_inc.h"

extern DummyRobot dummy;

extern RGB rgb;

static void PrintRGBStatus(StreamSink& response, bool full)
{
    const auto c = rgb.GetConfig();
    const auto s = rgb.GetStatus(dummy.GetRGBState());
    // Respond has a 62-byte payload limit; preserve the shared transport contract.
    Respond(response, "RGB_STATUS on=%u auto=%u bright=%u current=%u mode=%u",
        unsigned(c.enabled), unsigned(c.automatic), unsigned(c.brightness), unsigned(s.currentBrightness),
        unsigned(s.mode));
    Respond(response, "RGB_RUNTIME state=%s source=%s", RGB::StateName(s.state), RGB::StateName(s.source));
    Respond(response, "RGB_STORAGE dirty=%u storage=%u dma=%u", unsigned(s.dirty), unsigned(s.storage), unsigned(s.dmaBusy));
    Respond(response, "RGB_DMA frames=%lu errors=%lu", (unsigned long)s.frames, (unsigned long)s.errors);
    Respond(response, "RGB_TIMEOUT timeouts=%lu", (unsigned long)s.timeouts);
    if (!full) return;
    Respond(response, "RGB_CONFIG manual=%u fade_on=%u fade_off=%u",
        unsigned(c.manualMode), unsigned(c.fadeOnMs), unsigned(c.fadeOffMs));
    Respond(response, "RGB_TIMING bright_ms=%u transition=%u", unsigned(c.brightnessMs), unsigned(c.transitionMs));
    for (unsigned i = 0; i < 3; ++i)
        Respond(response, "RGB_COLOR %u %u %u %u", i, unsigned(c.colors[i].r), unsigned(c.colors[i].g), unsigned(c.colors[i].b));
    for (unsigned i = 0; i < RGB::AUTO_COLOR_COUNT; ++i)
        Respond(response, "RGB_AUTO_COLOR %u %u %u %u", i+1, unsigned(c.autoColors[i].r),
            unsigned(c.autoColors[i].g), unsigned(c.autoColors[i].b));
    for (unsigned i = 3; i < RGB::EFFECT_COUNT; ++i) {
        const auto& e = c.effects[i];
        Respond(response, "RGB_EFFECT %u %u %u %u %u %u %u %u %u %u %u %u", i,
            unsigned(e.first.r), unsigned(e.first.g), unsigned(e.first.b),
            unsigned(e.second.r), unsigned(e.second.g), unsigned(e.second.b),
            unsigned(e.periodMs), unsigned(e.reverse), unsigned(e.tail), unsigned(e.onMs), unsigned(e.offMs));
    }
    for (unsigned i = 0; i < RGB::STATE_COUNT; ++i) {
        const auto& b = c.bindings[i];
        Respond(response, "RGB_STATE %s %u %u %u %u %u", RGB::StateName(i), unsigned(b.mode),
            unsigned(b.customColor), unsigned(b.color.r), unsigned(b.color.g), unsigned(b.color.b));
    }
    Respond(response, "RGB_GET end");
}
static bool RGBUInt(const char* token, uint32_t& value)
{
    if (!*token) return false;
    value = 0;
    for (; *token; ++token) {
        if (*token < '0' || *token > '9') return false;
        const uint32_t digit = uint32_t(*token - '0');
        if (value > (0xFFFFFFFFUL - digit) / 10) return false;
        value = value * 10 + digit;
    }
    return true;
}
static bool HandleRGBCommand(const char* cmd, size_t length, StreamSink& response)
{
    while (length && (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n')) { ++cmd; --length; }
    const bool recognized = (length >= 5 && !std::strncmp(cmd, "!RGB_", 5)) ||
        (length >= 5 && !std::strncmp(cmd, "#RGB_", 5));
    if (!recognized) return false;
    char buffer[192]{};
    if (length >= sizeof(buffer) || std::memchr(cmd, 0, length)) { Respond(response, "error RGB syntax"); return true; }
    std::memcpy(buffer, cmd, length);
    char* tokens[16]{};
    unsigned count = 0;
    char* p = buffer;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
        if (!*p) break;
        if (count == 16) { Respond(response, "error RGB syntax"); return true; }
        tokens[count++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
        if (*p) *p++ = 0;
    }
    if (!count) return true;
    const char* verb = tokens[0];
    if (!std::strcmp(verb, "#RGB_STATUS") || !std::strcmp(verb, "#RGB_GET")) {
        if (count != 1) Respond(response, "error RGB syntax");
        else PrintRGBStatus(response, !std::strcmp(verb, "#RGB_GET"));
        return true;
    }
    if (!std::strcmp(verb, "!RGB_BRIGHT") && count == 1) {
        Respond(response, "RGB_BRIGHT %u", unsigned(rgb.GetConfig().brightness)); return true;
    }
    if (dummy.IsRGBSaveBusy()) { Respond(response, "error RGB save_busy"); return true; }
    bool save = count > 1 && !std::strcmp(tokens[count-1], "&");
    if (save) --count;
    uint32_t revision = 0, args[12]{};
    auto c = rgb.GetConfig(&revision);
    bool ok = true, mutate = true;
    auto numbers = [&](unsigned start, unsigned number) {
        if (count != start + number || number > 12) return false;
        for (unsigned i = 0; i < number; ++i) if (!RGBUInt(tokens[start+i], args[i])) return false;
        return true;
    };
    if (!std::strcmp(verb, "!RGB_ON") || !std::strcmp(verb, "!RGB_OFF")) {
        ok = count == 1; c.enabled = !std::strcmp(verb, "!RGB_ON");
    } else if (!std::strcmp(verb, "!RGB_ENABLE") || !std::strcmp(verb, "!RGB_AUTO")) {
        ok = numbers(1, 1) && args[0] <= 1;
        if (!std::strcmp(verb, "!RGB_ENABLE")) c.enabled = uint8_t(args[0]); else c.automatic = uint8_t(args[0]);
    } else if (!std::strcmp(verb, "!RGB_BRIGHT")) {
        ok = numbers(1, 1) && args[0] <= 100; c.brightness = uint8_t(args[0]);
    } else if (!std::strcmp(verb, "!RGB_MODE")) {
        ok = numbers(1, 1) && args[0] < RGB::MODE_COUNT; c.manualMode = uint8_t(args[0]); c.automatic = 0;
    } else if (!std::strcmp(verb, "!RGB_COLOR")) {
        ok = numbers(1, 4) && args[0] < 3 && args[1] <= 255 && args[2] <= 255 && args[3] <= 255;
        if (ok) c.colors[args[0]] = {uint8_t(args[1]), uint8_t(args[2]), uint8_t(args[3])};
    } else if (!std::strcmp(verb, "!RGB_AUTO_COLOR")) {
        ok = numbers(1, 4) && args[0] >= 1 && args[0] <= RGB::AUTO_COLOR_COUNT &&
            args[1] <= 255 && args[2] <= 255 && args[3] <= 255;
        if (ok) c.autoColors[args[0]-1] = {uint8_t(args[1]), uint8_t(args[2]), uint8_t(args[3])};
    } else if (!std::strcmp(verb, "!RGB_FADE")) {
        ok = numbers(1, 4) && args[0] <= 10000 && args[1] <= 10000 && args[2] <= 10000 && args[3] <= 5000;
        if (ok) { c.fadeOnMs = args[0]; c.fadeOffMs = args[1]; c.brightnessMs = args[2]; c.transitionMs = args[3]; }
    } else if (!std::strcmp(verb, "!RGB_EFFECT")) {
        ok = numbers(1, 12) && args[0] >= 3 && args[0] < RGB::EFFECT_COUNT;
        for (unsigned i=1; ok && i<=6; ++i) ok = args[i] <= 255;
        ok = ok && args[7] >= 250 && args[7] <= 30000 && args[8] <= 1 && args[9] >= 1 && args[9] <= 12 &&
            args[10] >= 20 && args[10] <= 30000 && args[11] >= 20 && args[11] <= 30000;
        if (ok) c.effects[args[0]] = {{uint8_t(args[1]),uint8_t(args[2]),uint8_t(args[3])},
            {uint8_t(args[4]),uint8_t(args[5]),uint8_t(args[6])}, uint16_t(args[7]), uint16_t(args[10]),
            uint16_t(args[11]), uint8_t(args[8]), uint8_t(args[9])};
    } else if (!std::strcmp(verb, "!RGB_SET_STATE")) {
        int state = count > 1 ? RGB::FindState(tokens[1]) : -1;
        ok = state >= 0 && state < RGB::FAULT && (count == 3 || count == 4 || count == 6) &&
            RGBUInt(tokens[2], args[0]) && args[0] < RGB::MODE_COUNT;
        uint8_t source = count == 6 ? 1 : 0;
        if (ok && count == 4) {
            source = !std::strcmp(tokens[3], "AUTO_SEQ") ? 2 : !std::strcmp(tokens[3], "AUTO_INT") ? 3 :
                !std::strcmp(tokens[3], "AUTO_TRJ") ? 4 : 0;
            ok = source != 0;
        }
        if (ok && count == 6) ok = numbers(2, 4) && args[1] <= 255 && args[2] <= 255 && args[3] <= 255;
        if (ok) c.bindings[state] = {uint8_t(args[0]), source,
            {uint8_t(args[1]), uint8_t(args[2]), uint8_t(args[3])}};
    } else if (!std::strcmp(verb, "!RGB_SET_START") || !std::strcmp(verb, "!RGB_SET_ENABLE") ||
               !std::strcmp(verb, "!RGB_SET_DISABLE")) {
        ok = numbers(1, 1) && args[0] < RGB::MODE_COUNT;
        if (ok) {
            unsigned lo = !std::strcmp(verb, "!RGB_SET_START") ? RGB::BOOT :
                !std::strcmp(verb, "!RGB_SET_DISABLE") ? RGB::DISABLED : RGB::READY_SEQ;
            unsigned hi = lo == RGB::READY_SEQ ? RGB::RUN_TUN : lo;
            for (unsigned s=lo; s<=hi; ++s) c.bindings[s] = {uint8_t(args[0]), 0, {0,0,0}};
        }
    } else if (!std::strcmp(verb, "!RGB_RESET")) {
        ok = count == 1; c = RGB::Defaults();
    } else if (!std::strcmp(verb, "!RGB_SAVE")) {
        ok = count == 1 && !save; mutate = false; save = true;
    } else ok = false;
    if (!ok || !RGB::Valid(c)) { Respond(response, "error RGB syntax_or_range"); return true; }
    if (mutate && !rgb.ApplyConfig(c, revision)) { Respond(response, "error RGB config_busy"); return true; }
    if (save) {
        const int result = dummy.SaveRGBConfig();
        if (result != 1) {
            Respond(response, result == 0 ? "error RGB save_requires_idle_disabled" : "error RGB save_failed");
            return true;
        }
        Respond(response, "ok RGB saved");
    } else Respond(response, "ok RGB applied");
    // Structured edits return the confirmed configuration themselves; the host
    // need not immediately send a second query (which used to duplicate replies).
    const bool full = !std::strcmp(verb, "!RGB_COLOR") || !std::strcmp(verb, "!RGB_AUTO_COLOR") ||
        !std::strcmp(verb, "!RGB_FADE") ||
        !std::strcmp(verb, "!RGB_EFFECT") || !std::strcmp(verb, "!RGB_SET_STATE") ||
        !std::strcmp(verb, "!RGB_SET_START") || !std::strcmp(verb, "!RGB_SET_ENABLE") ||
        !std::strcmp(verb, "!RGB_SET_DISABLE") || !std::strcmp(verb, "!RGB_RESET");
    PrintRGBStatus(response, full);
    return true;
}

static bool HandleStopCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t first = command.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return true;
    const size_t last = command.find_last_not_of(" \t\r\n");
    command = command.substr(first, last - first + 1);
    if (command.rfind("#MOTOR_DEFAULT", 0) == 0 && &response != usbStreamOutputPtr) {
        Respond(response, "error MOTOR_DEFAULT USB_only"); return true;
    }
    if (command == "!START")
    {
        if (dummy.IsRGBSaveBusy()) { Respond(response, "error START busy"); return true; }
        if (dummy.IsCalibrationBusy()) { Respond(response, "error CALIBRATION busy"); return true; }
        if (dummy.IsProtectionParameterBusy()) { Respond(response, "error START busy"); return true; }
        if (!dummy.RequestStart()) { Respond(response, "error START busy"); return true; }
        Respond(response, dummy.IsEnabled() && !dummy.IsStopLocked() ? "Started ok" : "Start pending");
        return true;
    }
    if (command == "!DISABLE")
    {
        dummy.RequestDisable();
        Respond(response, "Disable pending");
        return true;
    }
    if (command == "!STOP")
    {
        dummy.RequestSoftStop();
        Respond(response, "Stopping ok");
        return true;
    }
    // Only these read-only diagnostics may pass the protection command lock.
    if (command == "#MOTION_STATUS")
    {
        dummy.PrintMotionStatus(response);
        return true;
    }
    if (command == "#RGB_STATUS" || command == "#RGB_GET" || command == "!RGB_BRIGHT")
        return HandleRGBCommand(command.c_str(), command.size(), response);
    if (command.rfind("#STALL_STATUS", 0) == 0 || command.rfind("#CAL_STATUS", 0) == 0 ||
        command.rfind("#FW_VERSION", 0) == 0 || command.rfind("#CAN_DIAG", 0) == 0)
        return dummy.HandleProtectionCommand(command.c_str(), command.size(), response);
    if (dummy.IsStopLocked())
    {
        Respond(response, dummy.IsCalibrationBusy() ? "error CALIBRATION busy" : "error STOP_LOCKED");
        return true;
    }
    if (dummy.IsRGBSaveBusy()) { Respond(response, "error RGB save_busy"); return true; }
    if (dummy.HandleProtectionCommand(command.c_str(), command.size(), response)) return true;
    if (!dummy.IsEnabled() && (command[0] == '>' || command[0] == '@' || command[0] == '&'))
    {
        Respond(response, "error DISABLED");
        return true;
    }
    return false;
}

static bool HandleMotionCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t end = command.find_last_not_of(" \t\r\n");
    if (end != std::string::npos) command.resize(end + 1);
    if (command == "#MOTION_STATUS") { dummy.PrintMotionStatus(response); return true; }
    if (command.rfind("#GET_CMDMODE", 0) == 0) {
        unsigned long request = 0; char extra;
        if (command != "#GET_CMDMODE" && sscanf(command.c_str(), "#GET_CMDMODE %lu %c", &request, &extra) != 1) {
            Respond(response, "error CMDMODE query syntax"); return true;
        }
        Respond(response, "ok CMDMODE mode=%u busy=%u issued=%lu req=%lu", unsigned(dummy.commandMode),
                unsigned(dummy.IsMotionBusy() || dummy.commandHandler.IsBusy()), (unsigned long)dummy.LastIssuedMotionId(), request);
        return true;
    }
    if (command.rfind("#CMDMODE", 0) != 0) return false;
    unsigned long mode; char tail;
    if (sscanf(command.c_str(), "#CMDMODE %lu %c", &mode, &tail) != 1 || mode < 1 || mode > 4)
        Respond(response, "error CMDMODE range=1..4");
    else if (!dummy.TrySetCommandMode(mode)) Respond(response, "error CMDMODE busy");
    else Respond(response, "ok CMDMODE mode=%u", unsigned(dummy.commandMode));
    return true;
}

static void RespondStackStats(StreamSink& responseChannel)
{
    // Minimum unused stack since boot, in bytes (not current free stack).
    Respond(responseChannel, "ok STACK usb=%lu uart=%lu heap=%lu",
            (unsigned long) UsbServerStackFreeBytes(),
            (unsigned long) UartServerStackFreeBytes(),
            (unsigned long) xPortGetFreeHeapSize());
}

static bool HandleAccelerationSyncCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t first = command.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return false;
    const size_t last = command.find_last_not_of(" \t\r\n");
    command = command.substr(first, last - first + 1);
    if (command.rfind("#SYNC_ACC", 0) != 0) return false;
    if (command != "#SYNC_ACC") Respond(response, "error SYNC_ACC syntax: #SYNC_ACC");
    else if (!dummy.RequestMotorAccelerationSync(response)) Respond(response, "error SYNC_ACC busy");
    return true;
}

static bool HandleEncoderCalibrationCommand(const char* cmd, size_t len, StreamSink& response)
{
    std::string command(cmd, len);
    const size_t first = command.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return false;
    const size_t last = command.find_last_not_of(" \t\r\n");
    command = command.substr(first, last - first + 1);
    if (command.rfind("#ENC_CAL", 0) != 0) return false;
    if (response.channelType != StreamSink::CHANNEL_TYPE_USB)
    {
        Respond(response, "error ENC_CAL forbidden on UART4");
        return true;
    }

    // A single node digit avoids overflow/sign aliases accidentally selecting a motor.
    const size_t nodeAt = command.find_first_not_of(" \t", 8);
    if (command.size() <= 8 || (command[8] != ' ' && command[8] != '\t') ||
        nodeAt == std::string::npos || nodeAt + 1 != command.size() ||
        command[nodeAt] < '1' || command[nodeAt] > '8')
    {
        Respond(response, "error ENC_CAL syntax: #ENC_CAL <1..8> (0=ALL forbidden)");
        return true;
    }
    if (dummy.IsEnabled())
        Respond(response, "error ENC_CAL require DISABLE");
    else if (dummy.IsStopLocked() || dummy.IsCalibrationBusy() ||
             dummy.IsProtectionParameterBusy() || dummy.IsRGBSaveBusy() || dummy.IsMotionBusy() || dummy.commandHandler.IsBusy())
        Respond(response, "error ENC_CAL busy");
    else
    {
        const unsigned node = unsigned(command[nodeAt] - '0');
        CtrlStepMotor* motor = node == 8 ? dummy.hand : dummy.motorJ[node];
        const uint8_t frame[8] = {};
        // Use the checked, local-frame sender; 0x02 has no motor completion ACK.
        if (!motor || !motor->TrySendProtection(0x02, frame))
            Respond(response, "error ENC_CAL [%u] CAN send failed", node);
        else
            Respond(response, "ok ENC_CAL [%u] dispatched", node);
    }
    return true;
}

void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (HandleStopCommand(_cmd, _len, _responseChannel)) return;
    if (HandleRGBCommand(_cmd, _len, _responseChannel)) return;
    if (HandleMotionCommand(_cmd, _len, _responseChannel)) return;
    if (HandleAccelerationSyncCommand(_cmd, _len, _responseChannel)) return;
    if (HandleEncoderCalibrationCommand(_cmd, _len, _responseChannel)) return;
    uint8_t  i;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' )
    {
        std::string s(_cmd);
        if (s.find("HOME") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!HOME", &_responseChannel) == 0xFF ?
                    "error DISABLED" : "Home pending");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!CALIBRATION") == 0xFF ?
                    "error STOP_LOCKED" : "Calibration pending");
        } else if (s.find("RESET") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!RESET", &_responseChannel) == 0xFF ?
                    "error DISABLED" : "Reset pending");
        }
        // [Step 3] !HAND_* 12 条命令（运行时 4 条 + 配置 8 条；UART4 仅允许运行时 4 条）
        // 运行时 4 条：!HAND_O / !HAND_C / !HAND_POS / !HAND_I
        // 严格匹配：!HAND_O 是 !HAND_ONBOOT 的前缀，用 "== "||" rfind(... , 0) == 0" 避免 !HAND_ONBOOT 被抢先匹配
        else if (s == "!HAND_O" || s.rfind("!HAND_O ", 0) == 0)        // 张开方向电流（取负）
        {
            float A = 0;
            sscanf(_cmd, "!HAND_O %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(-A);
            Respond(_responseChannel, "ok hand open I=%.2f", A);
        }
        else if (s == "!HAND_C" || s.rfind("!HAND_C ", 0) == 0)        // 闭合方向电流（取正）
        {
            float A = 0;
            sscanf(_cmd, "!HAND_C %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(+A);
            Respond(_responseChannel, "ok hand close I=%.2f", A);
        }
        else if (s.rfind("!HAND_POS ", 0) == 0)                         // 位置命令 0~100
        {
            uint32_t pos;
            sscanf(_cmd, "!HAND_POS %lu", &pos);
            if (pos > 100) pos = 100;
            dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
            Respond(_responseChannel, "ok hand pos=%lu", pos);
        }
        else if (s.rfind("!HAND_I ", 0) == 0)                          // 运行时电流幅值（不发 CAN 帧）
        {
            float cu;
            sscanf(_cmd, "!HAND_I %f", &cu);
            if (cu > 0 && cu <= 2.5f)
            {
                dummy.hand->current = cu;
                Respond(_responseChannel, "ok hand I=%.2f", cu);
            }
            // 上限 2.5A / ≤0：静默拒绝，不响应（与决策 §2.1 一致）
        }
        // 配置 8 条：UART4 禁止；USB 调基类方法（EEPROM 持久化由基类 canBuf[4] 决定）
        else if (s.rfind("!HAND_VEL ", 0) == 0)
        {
            float v;
            sscanf(_cmd, "!HAND_VEL %f", &v);
            dummy.hand->SetVelocityLimit(v);
            Respond(_responseChannel, "ok hand set VEL=%.2f", v);
        }
        else if (s.rfind("!HAND_ACC ", 0) == 0)
        {
            float a;
            sscanf(_cmd, "!HAND_ACC %f", &a);
            dummy.hand->SetAcceleration(a);
            Respond(_responseChannel, "ok hand set ACC=%.2f", a);
        }
        else if (s.rfind("!HAND_KP ", 0) == 0)
        {
            int32_t p;
            sscanf(_cmd, "!HAND_KP %ld", &p);
            dummy.hand->SetDceKp(p);
            Respond(_responseChannel, "ok hand set KP=%ld", p);
        }
        else if (s.rfind("!HAND_KV ", 0) == 0)
        {
            int32_t v;
            sscanf(_cmd, "!HAND_KV %ld", &v);
            dummy.hand->SetDceKv(v);
            Respond(_responseChannel, "ok hand set KV=%ld", v);
        }
        else if (s.rfind("!HAND_KI ", 0) == 0)
        {
            int32_t i;
            sscanf(_cmd, "!HAND_KI %ld", &i);
            dummy.hand->SetDceKi(i);
            Respond(_responseChannel, "ok hand set KI=%ld", i);
        }
        else if (s.rfind("!HAND_KD ", 0) == 0)
        {
            int32_t d;
            sscanf(_cmd, "!HAND_KD %ld", &d);
            dummy.hand->SetDceKd(d);
            Respond(_responseChannel, "ok hand set KD=%ld", d);
        }
        else if (s == "!HAND_ONBOOT" || s.rfind("!HAND_ONBOOT ", 0) == 0)
        {
            uint32_t val = 1;  // 缺省 = 1（使能上电自启）；<0/1> 显式传参
            sscanf(_cmd, "!HAND_ONBOOT %lu", &val);
            dummy.hand->SetEnableOnBoot(val != 0);
            Respond(_responseChannel, "ok hand set ONBOOT=%lu", val);
        }
        else if (s == "!HAND_ERASE")
        {
            dummy.hand->EraseConfigs();
            Respond(_responseChannel, "ok hand set ERASE=1");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s == "#STACK")
        {
            RespondStackStats(_responseChannel);
        } else if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5],
                    dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
        } else if (s.find("GETRAILPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f", dummy.currentRailPos);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        }
        // [v2026-10-03] 查询端补齐：#GET_PID 1~8 + 0=ALL
        //   1~8 单节点 30ms 同步等（osDelay(1) 轮询 motorPidRcvdCount[idx] >= 4）
        //   0=ALL 立即 Respond "ALL dispatched"，32 帧回包由 OledTask PollPidPrint 分行打印
        else if (s.find("GET_PID") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GET_PID %lu", &node);
            if (node == 0) {
                // ALL 模式：8 节点 × 4 帧 = 32 帧，立即返回不阻塞
                // [v2026-10-04] ASCII 主动设 _pid_print_pending（ISR 已不再设）：让 OledTask PollPidPrint 异步打印
                for (int i = 1; i <= 7; i++) {
                    dummy.motorJ[i]->QueryDceKp();
                    dummy.motorJ[i]->QueryDceKv();
                    dummy.motorJ[i]->QueryDceKi();
                    dummy.motorJ[i]->QueryDceKd();
                    dummy._pid_print_pending[i] = 1;       // ASCII 主动置 flag → OledTask PollPidPrint
                    dummy._pid_pending_node[i] = i;        // 节点号用于 printf (idx → 实际 nodeID)
                }
                dummy.hand->QueryDceKp();
                dummy.hand->QueryDceKv();
                dummy.hand->QueryDceKi();
                dummy.hand->QueryDceKd();
                dummy._pid_print_pending[8] = 1;           // hand = idx 8
                dummy._pid_pending_node[8] = 8;
                Respond(_responseChannel, "ok GET_PID ALL dispatched (8 nodes x 4 = 32 frames; results printed by OledTask PollPidPrint in 1-2 s)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                if (node == 8) {
                    dummy.hand->QueryDceKp(); dummy.hand->QueryDceKv();
                    dummy.hand->QueryDceKi(); dummy.hand->QueryDceKd();
                } else {
                    dummy.motorJ[node]->QueryDceKp(); dummy.motorJ[node]->QueryDceKv();
                    dummy.motorJ[node]->QueryDceKi(); dummy.motorJ[node]->QueryDceKd();
                }
                // [v2026-10-04] 改用时间戳模式，与 ACC/I_LIMIT 同模式；不再共享 _pid_print_pending
                uint32_t old_ms = dummy.motorPidLastUpdateMs[idx];
                uint32_t waited = 0;
                while (dummy.motorPidLastUpdateMs[idx] == old_ms && waited < 10) {  // 30ms → 10ms
                    osDelay(1); waited++;
                }
                if (dummy.motorPidLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok PID [%lu] kp=%ld kv=%ld ki=%ld kd=%ld%s",
                            node, (long)dummy.motorDceKps[idx], (long)dummy.motorDceKvs[idx],
                            (long)dummy.motorDceKis[idx], (long)dummy.motorDceKds[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    uint8_t got = dummy.motorPidRcvdCount[idx];
                    Respond(_responseChannel, "error GET_PID [%lu] timeout (got %u/4, %lu ms)", node, got, waited);
                }
            } else {
                Respond(_responseChannel, "error GET_PID [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        } else if (s.find("SET_DCE_KP") != std::string::npos)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &kp);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, kp);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu] (HAND)", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KP [%lu] is wrong (range 1~8, reject 0)", node, kp);
            }
        } else if (s.find("SET_DCE_KI") != std::string::npos)
        {
            uint32_t ki;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &ki);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKi(ki);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, ki);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKi(ki);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu] (HAND)", node, ki);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KI [%lu] is wrong (range 1~8, reject 0)", node, ki);
            }
        } else if (s.find("SET_DCE_KD") != std::string::npos)
        {
            uint32_t kd;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &kd);
            if (node >= 1 && node <= 7){  // [Step 4] 范围 1~6 → 1~7（8 在下面单独处理）
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kd);
            }
            else if (node == 8){           // [Step 4] 夹爪走 dummy.hand
                dummy.hand->SetDceKd(kd);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu] (HAND)", node, kd);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong (range 1~8, reject 0)", node, kd);
            }
        }
        // [Step 4] 新增 #SET_DCE_KV / #I_LIMIT_J / #ACC_J 3 条 # 配置类命令
        else if (s.find("SET_DCE_KV") != std::string::npos)
        {
            int32_t kv;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KV %lu %ld", &node, &kv);
            if (node >= 1 && node <= 7){
                dummy.motorJ[node]->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%ld]", node, kv);
            }
            else if (node == 8){
                dummy.hand->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%ld] (HAND)", node, kv);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KV [%ld] is wrong (range 1~8, reject 0)", node, kv);
            }
        }
        // [v2026-10-03] 查询端补齐：#SET_PID 1~8（写 EEPROM 4 个 PID 值）
        //   放在 SET_DCE_KV 之后、I_LIMIT_J 之前（防 SET_DCE_K* 子串抢先匹配）
        else if (s.find("SET_PID") != std::string::npos)
        {
            uint32_t node, kp, kv, ki, kd;
            sscanf(_cmd, "#SET_PID %lu %lu %lu %lu %lu", &node, &kp, &kv, &ki, &kd);
            if (node >= 1 && node <= 7) {
                dummy.motorJ[node]->SetDceKp(kp);
                dummy.motorJ[node]->SetDceKv(kv);
                dummy.motorJ[node]->SetDceKi(ki);
                dummy.motorJ[node]->SetDceKd(kd);
                Respond(_responseChannel, "ok SET PID [%lu] kp=%lu kv=%lu ki=%lu kd=%lu%s",
                        node, kp, kv, ki, kd, node == 7 ? " (rail)" : "");
            } else if (node == 8) {
                dummy.hand->SetDceKp(kp);
                dummy.hand->SetDceKv(kv);
                dummy.hand->SetDceKi(ki);
                dummy.hand->SetDceKd(kd);
                Respond(_responseChannel, "ok SET PID [8] kp=%lu kv=%lu ki=%lu kd=%lu (HAND)", kp, kv, ki, kd);
            } else {
                Respond(_responseChannel, "error SET PID [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL not allowed)", node);
            }
        }
        else if (s.find("I_LIMIT_J") != std::string::npos)
        {
            float current;
            uint32_t node;
            char saveFlag = 0;
            int ret = sscanf(_cmd, "#I_LIMIT_J %lu %f %c", &node, &current, &saveFlag);
            if (ret >= 2) {
                bool persist = (saveFlag == '&');   // [v2026-10-04] & 决定是否写 EEPROM（与 ref 对齐，默认不写）
                if (node >= 1 && node <= 7){
                    dummy.motorJ[node]->SetCurrentLimit_persist(current, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] I_LIMIT [%.2f]%s", node, current,
                            persist ? " (saved)" : "");
                }
                else if (node == 8){
                    dummy.hand->SetCurrentLimit_persist(current, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] I_LIMIT [%.2f] (HAND)%s", node, current,
                            persist ? " (saved)" : "");
                }
                else {
                    Respond(_responseChannel, "error SET MOTOR [%lu] I_LIMIT [%.2f] is wrong (range 1~8, reject 0)", node, current);
                }
            } else {
                Respond(_responseChannel, "error I_LIMIT_J parse failed. Use: #I_LIMIT_J <node> <A> [&]");
            }
        }
        // [v2026-10-03] 查询端补齐：#GETJACC 1~8 + 0=ALL
        //   1~8 单节点 30ms 同步等 jointAccLastUpdateMs[idx] 变化
        //   0=ALL 调 SyncAllMotorAcceleration() 异步触发 8 个 0x2C
        //   放在 I_LIMIT_J 之后、ACC_J 之前
        else if (s.find("GETJACC") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GETJACC %lu", &node);
            if (node == 0) {
                dummy.SyncAllMotorAcceleration();
                Respond(_responseChannel, "ok GETJACC ALL dispatched (8 nodes; results printed by OledTask in <100 ms)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                uint32_t old_ms = dummy.jointAccLastUpdateMs[idx];
                if (node == 8) dummy.hand->QueryAcceleration();
                else           dummy.motorJ[node]->QueryAcceleration();
                uint32_t waited = 0;
                while (dummy.jointAccLastUpdateMs[idx] == old_ms && waited < 30) {
                    osDelay(1); waited++;
                }
                if (dummy.jointAccLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok ACC [%lu] = %.2f r/s^2%s", node, dummy.jointAccRuntime[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    Respond(_responseChannel, "error GETJACC [%lu] timeout (%lu ms)", node, waited);
                }
            } else {
                Respond(_responseChannel, "error GETJACC [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        }
        // [v2026-10-03] 查询端补齐：#GETI 1~8 + 0=ALL
        //   ⚠️ 必须放 ACC_J 分支之前（防 #GETI 1 被 ACC_J 抢先匹配 — 实际不会但按方案顺序约束）
        else if (s.find("GETI") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#GETI %lu", &node);
            if (node == 0) {
                dummy.SyncAllMotorCurrentLimit();
                Respond(_responseChannel, "ok GETI ALL dispatched (8 nodes, async; results printed by OledTask in <100 ms)");
            } else if (node >= 1 && node <= 8) {
                int idx = (node == 7) ? 7 : (node == 8) ? 8 : (int)node;
                uint32_t old_ms = dummy.jointCurrentLimitLastUpdateMs[idx];
                if (node == 8) dummy.hand->QueryCurrentLimit();
                else           dummy.motorJ[node]->QueryCurrentLimit();
                uint32_t waited = 0;
                while (dummy.jointCurrentLimitLastUpdateMs[idx] == old_ms && waited < 30) {
                    osDelay(1); waited++;
                }
                if (dummy.jointCurrentLimitLastUpdateMs[idx] != old_ms) {
                    Respond(_responseChannel, "ok I_LIMIT [%lu] = %.2f A%s", node, dummy.jointCurrentLimitRuntime[idx],
                            node == 8 ? " (HAND)" : (node == 7 ? " (rail)" : ""));
                } else {
                    Respond(_responseChannel, "error GETI [%lu] timeout (%lu ms)", node, waited);
                }
            } else {
                Respond(_responseChannel, "error GETI [%lu] wrong (use 7=rail, 1~6=joints, 8=hand, 0=ALL)", node);
            }
        }
        else if (s.find("ACC_J") != std::string::npos)
        {
            float acc;
            uint32_t node;
            char saveFlag = 0;
            int ret = sscanf(_cmd, "#ACC_J %lu %f %c", &node, &acc, &saveFlag);
            if (ret >= 2) {
                bool persist = (saveFlag == '&');   // [v2026-10-04] & 决定是否写 EEPROM（与 ref 对齐）
                if (node >= 1 && node <= 7){
                    dummy.motorJ[node]->SetAcceleration_persist(acc, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] ACC [%.2f]%s", node, acc,
                            persist ? " (saved)" : "");
                }
                else if (node == 8){
                    dummy.hand->SetAcceleration_persist(acc, persist);
                    Respond(_responseChannel, "ok SET MOTOR [%lu] ACC [%.2f] (HAND)%s", node, acc,
                            persist ? " (saved)" : "");
                }
                else {
                    Respond(_responseChannel, "error SET MOTOR [%lu] ACC [%.2f] is wrong (range 1~8, reject 0)", node, acc);
                }
            } else {
                Respond(_responseChannel, "error ACC_J parse failed. Use: #ACC_J <node> <r/s^2> [&]");
            }
        } else if (s.find("REBOOT") != std::string::npos)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node >= 1 && node <= 7)                  // J1~J7（含地轨）
            {
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else if (node == 8)                          // [Step 4] 夹爪独立 reboot
            {
                dummy.hand->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu] (HAND)", node);
            }
            else if (node == 0)                          // ALL 代理（J1~J7 + 夹爪 1 帧广播）
            {
                Respond(_responseChannel, "ok REBOOT ALL MOTORS + HAND");
                dummy.RebootAllMotors();
            }
            else
            {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong (range 1~8, 0=ALL)", node);
            }
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        if (dummy.commandHandler.Push(_cmd, &_responseChannel) == 0xFF)
            Respond(_responseChannel, "error motion queue full");
    }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (HandleStopCommand(_cmd, _len, _responseChannel)) return;
    if (HandleRGBCommand(_cmd, _len, _responseChannel)) return;
    if (HandleMotionCommand(_cmd, _len, _responseChannel)) return;
    if (HandleEncoderCalibrationCommand(_cmd, _len, _responseChannel)) return;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("HOME") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!HOME", &_responseChannel) == 0xFF ?
                    "error DISABLED" : "Home pending");
        } else if (s.find("CALIBRATION") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!CALIBRATION") == 0xFF ?
                    "error STOP_LOCKED" : "Calibration pending");
        } else if (s.find("RESET") != std::string::npos)
        {
            Respond(_responseChannel, dummy.commandHandler.PushSystem("!RESET", &_responseChannel) == 0xFF ?
                    "error DISABLED" : "Reset pending");
        }
        // [Step 3] UART4 仅允许 !HAND_O/C/POS/I 4 条运行时（顺序匹配避免 !HAND_ONBOOT 抢先匹配 !HAND_O）
        else if (s == "!HAND_O" || s.rfind("!HAND_O ", 0) == 0)
        {
            float A = 0;
            sscanf(_cmd, "!HAND_O %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(-A);
            Respond(_responseChannel, "ok hand open I=%.2f", A);
        }
        else if (s == "!HAND_C" || s.rfind("!HAND_C ", 0) == 0)
        {
            float A = 0;
            sscanf(_cmd, "!HAND_C %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(+A);
            Respond(_responseChannel, "ok hand close I=%.2f", A);
        }
        else if (s.rfind("!HAND_POS ", 0) == 0)
        {
            uint32_t pos;
            sscanf(_cmd, "!HAND_POS %lu", &pos);
            if (pos > 100) pos = 100;
            dummy.hand->SetAngleWithSpeedLimit(static_cast<float>(pos));
            Respond(_responseChannel, "ok hand pos=%lu", pos);
        }
        else if (s.rfind("!HAND_I ", 0) == 0)
        {
            float cu;
            sscanf(_cmd, "!HAND_I %f", &cu);
            if (cu > 0 && cu <= 2.5f)
            {
                dummy.hand->current = cu;
                Respond(_responseChannel, "ok hand I=%.2f", cu);
            }
            // 上限 2.5A / ≤0：静默拒绝，不响应
        }
        // [阻塞 3 关键] 其余 !HAND_*（8 条配置类）一律 forbidden——不让上位机超时卡死
        else if (s.substr(0, 5) == "!HAND")
            Respond(_responseChannel, "error forbidden on UART4");
        // 其它未知 ! 命令同样 forbidden
        else
            Respond(_responseChannel, "error forbidden on UART4");
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s == "#STACK")
        {
            RespondStackStats(_responseChannel);
        } else if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5],
                    dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
        } else if (s.find("GETRAILPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f", dummy.currentRailPos);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        if (dummy.commandHandler.Push(_cmd, &_responseChannel) == 0xFF)
            Respond(_responseChannel, "error motion queue full");
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (dummy.IsStopLocked()) return;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
