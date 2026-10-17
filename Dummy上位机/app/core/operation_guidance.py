"""Read-only explanations of existing action conditions.

These records are presentation hints, never authorization to write a command.
``blocked`` describes an existing dispatch or UI condition; ``attention`` is
advice for an action that the controller can still submit. In particular, the
ordinary motion dispatcher does not add a local motor-enabled requirement.
"""
from collections.abc import Mapping
import math


def _mapping(value):
    return value if isinstance(value, Mapping) else {}


def _item(label, status="available", reason="", next_step=""):
    return {"label": label, "available": status != "blocked", "status": status,
            "reason": reason, "next_step": next_step}


def operation_actions(state, gamepad=None, rgb=None):
    """Explain START/STOP/motion/claim/RGB save without changing any policy.

    Inputs are controller/session snapshots. Missing optional facts remain
    unknown; no timestamps, device access, parsing of logs, or commands occur.
    ``available`` is for descriptive lists only and must not drive button
    enablement. Dispatch remains the authority and rechecks live conditions.
    """
    state, gamepad, rgb = map(_mapping, (state, gamepad, rgb))
    connected = bool(state.get("connected", False))
    claimed = bool(gamepad.get("claimed", False) or state.get("control_owner") == "gamepad")
    rgb_busy = bool(state.get("rgb_save_busy", rgb.get("save_busy", False)))
    rgb_stage = state.get("rgb_save_stage", rgb.get("save_stage", ""))
    rgb_message = str(state.get("rgb_save_message") or rgb.get("save_message") or "")
    diagnostic_busy = bool(state.get("cartesian_busy", False))
    parameter_busy = bool(state.get("parameter_busy", False))
    sequence = bool(state.get("sequence_active", False))
    calibrating = bool(state.get("calibration_busy", False))
    locked = bool(state.get("stop_locked", False))
    motor = state.get("motor_enabled")
    native_tcp = bool(state.get("native_tcp", False))
    mode = state.get("cmdmode")

    def blocked(label, reason, next_step):
        return _item(label, "blocked", reason, next_step)

    def common(label, *, motion=False, safety=False):
        if not connected:
            return blocked(label, "未连接机械臂串口。", "选择串口并连接设备。")
        if safety:
            if rgb_busy and rgb_stage in ("waiting", "uncertain"):
                return _item(label, "attention", "RGB 保存结果尚未确认。",
                             "可请求停止或失能；若 Flash 已开始写入，将在保存确认后优先发送。")
            return None
        if rgb_busy:
            return blocked(label, rgb_message or "RGB 保存事务尚未结束。",
                           "等待保存确认；结果不确定时断开重连核实，不要重复保存。"
                           if rgb_stage == "uncertain" else "等待 RGB 保存事务结束；需要中断运动时仍可请求 STOP 或失能。")
        if diagnostic_busy:
            return blocked(label, str(state.get("cartesian_message") or "诊断或模式恢复正在处理。"),
                           "等待处理结束；需要停止时可请求 STOP 或失能。")
        if claimed:
            return blocked(label, "手柄已接管控制。", "先释放手柄控制，再使用界面发令；STOP 和失能仍可使用。")
        if parameter_busy:
            task = str(state.get("parameter_task") or "堵转 / 校准参数读写")
            return blocked(label, f"参数事务尚未结束：{task}。", "等待完整参数回执；需要停止时可请求 STOP 或失能。")
        if motion and sequence:
            return blocked(label, "示教点顺序任务尚未结束。", "等待顺序任务结束；需要中止时请求 STOP。")
        if calibrating:
            return blocked(label, "校准及保存尚未完成。", "等待校准结束；需要中断时可请求 STOP 或失能。")
        if motion and locked:
            return blocked(label, "当前处于 STOP 或保护锁定。",
                           "先查看锁定原因并确认现场状态，再请求 START 或失能；以设备回执为准。")
        if motion and mode == 7:
            return blocked(label, "主控实际处于诊断模式 7，普通运动不可用。",
                           "显式关闭诊断会话，恢复并确认普通主控模式后重试。")
        return None

    result = {}
    for key, label in (("stop", "STOP"), ("disable", "失能")):
        result[key] = common(label, safety=True) or _item(
            label, reason="可请求停止或失能。", next_step="发送后等待设备确认；发出请求不代表已经停稳。")

    start = common("START 使能")
    if start is None:
        operation = _mapping(state.get("operation"))
        if operation.get("kind") == "start" and operation.get("phase") not in (
                None, "idle", "completed", "error", "failed", "cancelled",
                "rejected", "locked", "disconnected"):
            start = _item("START 使能", "attention", "正在等待本次使能结果。", "等待设备的使能确认，避免重复点击。")
        elif locked:
            start = _item("START 使能", "attention", "当前处于 STOP 或保护锁定。",
                          "确认锁定原因和现场状态后再请求 START；等待主控完成检查并确认使能。")
        elif motor is True:
            start = _item("START 使能", "attention", "设备已确认使能。", "可发送单个运动目标，无需重复使能。")
        else:
            start = _item("START 使能", reason="可请求使能；由主控完成检查。", next_step="发送后等待使能成功回执。")
    result["start"] = start

    for key, label in (("movej", "MoveJ 单点"), ("movel", "MoveL 单点")):
        item = common(label, motion=True)
        if item is None and motor is not True:
            item = _item(label, "attention",
                         "电机已确认失能。" if motor is False else "电机使能状态尚未确认。",
                         "建议先 START 并等待确认，再发送目标。")
        if item is None:
            item = _item(label, reason="可提交单个运动目标；仍需主控受理。",
                         next_step="收到 accepted 仅表示受理；完成以对应设备回执或流程要求为准。")
        if key == "movel" and native_tcp and item["status"] == "available":
            item["next_step"] = "当前固件支持有限单点；连续拖发和连续手柄暂不可用，切换模式不能解除该限制。"
        result[key] = item

    label = "手柄接管"
    if native_tcp:
        claim = blocked(label, "当前固件仅支持有限笛卡尔目标，连续手柄暂不可用。",
                        "使用界面单点控制；切换 INT 或 TRJ 不能解除能力限制。")
    elif mode == 7:
        claim = blocked(label, "主控实际处于诊断模式 7。", "先显式退出诊断并确认普通模式，再接管手柄。")
    elif claimed:
        claim = blocked(label, "正在释放手柄控制。" if gamepad.get("release_pending") else "手柄已经接管。",
                        "等待停止确认及释放完成。" if gamepad.get("release_pending") else
                        str(gamepad.get("message") or "可使用手柄面板操作，或点击释放返回界面控制。"))
    elif diagnostic_busy:
        claim = blocked(label, "诊断或模式恢复尚未结束。", "等待处理结束，再接管手柄。")
    elif not connected:
        claim = blocked(label, "机械臂串口尚未连接。", "连接机械臂后检查有效手柄输入，再点击接管。")
    elif not gamepad.get("device_connected", False):
        detail = gamepad.get("error") or gamepad.get("message")
        claim = blocked(label, str(detail or "尚未确认有效手柄输入。"),
                        "点击检查输入，查看设备、按键和轴映射提示；有效输入恢复后再接管。")
    elif "input_age_s" in gamepad and (gamepad["input_age_s"] is None or
            isinstance(gamepad["input_age_s"], (int, float)) and
            (not math.isfinite(gamepad["input_age_s"]) or gamepad["input_age_s"] > .5)):
        claim = blocked(label, "手柄输入未更新或已超过 0.5 秒。", "重新检查输入，确认持续收到有效样本后再接管。")
    elif (state.get("position_sync_busy", False) or sequence or parameter_busy or
          state.get("acc_sync_waiting", False) or state.get("mode_request_busy", False) or
          state.get("mode_request") is not None or state.get("mode_pending") is not None or calibrating):
        claim = blocked(label, "当前任务或前台位置同步尚未结束。",
                        "等待示教、参数、加速度、模式、校准或位置同步任务结束，再接管。")
    else:
        claim = _item(label, reason="可主动请求接管；接管本身不使能或运动。",
                      next_step="接管后先使能并初始化；按流程确认实物到位后，再等待摇杆和扳机回中。")
    result["gamepad"] = claim

    save = common("保存 RGB")
    if save is None:
        if locked:
            save = blocked("保存 RGB", "设备仍处于 STOP 或保护锁定。",
                           "先确认锁定原因并请求失能，等待 Disabled ok 后再保存。")
        elif motor is not False or sequence or state.get("acc_sync_waiting", False):
            save = blocked("保存 RGB", "写入 EEPROM 需要已确认失能且没有未完成任务。",
                           "结束任务，点击失能并等待 Disabled ok，再保存 RGB。")
        else:
            save = _item("保存 RGB", reason="已确认失能，可请求保存配置。",
                         next_step="保存期间暂停查询，等待保存确认；结果不确定时不会自动重发。")
    result["rgb_save"] = save
    return result
