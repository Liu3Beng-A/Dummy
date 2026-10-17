"""Bounded presentation events for explicit configuration actions.

This observer never admits commands, queries a device or retries a write.  Wire
acknowledgements without request IDs are matched in admission order; expired
requests remain as tombstones so a late reply cannot confirm a newer action.
"""
from copy import deepcopy
import math
import re
import threading
import time


def configuration_action(command):
    """Describe user configuration writes, excluding all telemetry and motion."""
    tokens = str(command).strip().split()
    if not tokens:
        return None
    verb = tokens[0]
    saved = tokens[-1] == "&"
    if verb.startswith("!RGB_"):
        names = {"!RGB_SAVE": "保存RGB配置", "!RGB_BRIGHT": "应用亮度",
                 "!RGB_COLOR": "应用灯效颜色", "!RGB_AUTO_COLOR": "应用自动颜色",
                 "!RGB_FADE": "应用过渡时间", "!RGB_EFFECT": "应用动画参数",
                 "!RGB_SET_STATE": "应用状态绑定", "!RGB_MOTION": "应用自动动效",
                 "!RGB_MODE": "应用灯效", "!RGB_ON": "打开灯效", "!RGB_OFF": "关闭灯效",
                 "!RGB_AUTO": "应用自动联动", "!RGB_ENABLE": "应用灯效开关",
                 "!RGB_RESET": "恢复RGB默认配置", "!RGB_SET_START": "应用开机灯效",
                 "!RGB_SET_ENABLE": "应用使能灯效", "!RGB_SET_DISABLE": "应用失能灯效"}
        if verb not in names:
            return None
        return dict(scope="rgb", operation="保存RGB配置" if saved else names[verb],
                    target="RGB", family="RGB", save=saved or verb == "!RGB_SAVE",
                    external=saved or verb == "!RGB_SAVE")
    match = re.fullmatch(r"#(STALL|CAL)_(SET|SAVE)", verb)
    if match:
        kind, operation = match.groups()
        label = "堵转保护参数" if kind == "STALL" else "限位校准参数"
        return dict(scope="protection" if kind == "STALL" else "calibration",
                    operation=("保存" if operation == "SAVE" else "应用") + label,
                    target="全部节点" if len(tokens) > 1 and tokens[1] == "0" else
                           "节点 " + (tokens[1] if len(tokens) > 1 else "?"),
                    family=kind, save=operation == "SAVE", external=True)
    if verb in ("#SET_PID", "#ACC_J", "#I_LIMIT_J"):
        label = {"#SET_PID": "PID", "#ACC_J": "加速度", "#I_LIMIT_J": "电流限制"}[verb]
        save = saved or verb == "#SET_PID"
        return dict(scope="motor", operation=("保存" if save else "应用") + label,
                    target="节点 " + (tokens[1] if len(tokens) > 1 else "?"),
                    family=verb[1:], save=save, external=False)
    match = re.fullmatch(r"!HAND_(VEL|ACC|KP|KV|KI|KD|ONBOOT|ERASE|I)", verb)
    if match:
        key = match[1]
        names = {"VEL": "速度", "ACC": "加速度", "KP": "KP", "KV": "KV",
                 "KI": "KI", "KD": "KD", "ONBOOT": "上电设置", "ERASE": "默认配置", "I": "电流"}
        return dict(scope="gripper", operation=("恢复夹爪" if key == "ERASE" else "应用夹爪") + names[key],
                    target="夹爪", family="HAND_" + key, save=key not in ("VEL", "I"), external=False)
    if verb == "#SYNC_ACC":
        return dict(scope="motor", operation="同步加速度", target="全部节点",
                    family="SYNC_ACC", save=False, external=False)
    return None


class ActionFeedback:
    """Thread-safe state only; the controller owns timers and Qt delivery."""
    LIMIT = 128
    TIMEOUT_SECONDS = 6.0

    def __init__(self, emit, clock=time.monotonic):
        self._emit = emit
        self.clock = clock
        self.lock = threading.RLock()
        self._counter = 0
        self._latest = {}
        self._events = {}
        self._records = []
        self._ambiguous = set()

    def notify(self, scope, operation, target="", phase="blocked", message="",
               next_step="", evidence="local", task_id=None):
        with self.lock:
            if task_id is None:
                self._counter += 1
                task_id = self._counter
            event = dict(id=task_id, task_id=task_id, scope=str(scope), operation=str(operation),
                         target=str(target), phase=str(phase), message=str(message),
                         next_step=str(next_step), evidence=str(evidence), updated_at=self.clock())
            # Preserve a newer user action when an older task finally replies.
            latest = self._latest.get(event["scope"])
            if latest is None or task_id >= latest["task_id"]:
                self._latest[event["scope"]] = event
            self._events[task_id] = event
            while len(self._events) > self.LIMIT:
                del self._events[next(iter(self._events))]
            self._emit(deepcopy(event))
            return task_id

    def snapshot(self, scope=None):
        with self.lock:
            return deepcopy(self._latest.get(scope, {}) if scope is not None else self._latest)

    def _report(self, record, phase, message, next_step="", evidence="local"):
        spec = record["spec"]
        if message.startswith(spec["operation"]):
            message = message[len(spec["operation"]):]
        message = f"{spec['target']} · {spec['operation']}：{message}"
        return self.notify(spec["scope"], spec["operation"], spec["target"], phase,
                           message, next_step, evidence, record["id"])

    def requested(self, command, generation, connection):
        spec = configuration_action(command)
        if spec is None or (spec["external"] and spec["family"] == "RGB"):
            return None
        with self.lock:
            task_id = self.notify(spec["scope"], spec["operation"], spec["target"], "pending",
                                  f"{spec['target']} · {spec['operation']}：请求已排队。", evidence="local")
            self._records.append(dict(id=task_id, command=command, spec=spec,
                                      generation=generation, connection=connection,
                                      requested_at=self.clock(), admitted_at=None,
                                      terminal=False, consumed=False))
            while len(self._records) > self.LIMIT:
                old = self._records.pop(0)
                if old["admitted_at"] is not None and not old["consumed"]:
                    self._ambiguous.add((old["connection"], old["spec"]["family"]))
            return task_id

    def admitted(self, command, generation, connection):
        with self.lock:
            for record in self._records:
                if (record["command"] == command and record["generation"] == generation and
                        record["connection"] == connection and not record["terminal"] and
                        record["admitted_at"] is None):
                    record["admitted_at"] = self.clock()
                    return

    def written(self, command, generation, connection):
        with self.lock:
            for record in self._records:
                if (record["command"] == command and record["generation"] == generation and
                        record["connection"] == connection and record["admitted_at"] is not None and
                        not record["terminal"] and not record.get("written")):
                    record["written"] = True
                    self._report(record, "pending", f"{record['spec']['operation']}请求已发送，等待确认。",
                                 evidence="written")
                    return

    @staticmethod
    def _near(actual, expected):
        try:
            return (math.isfinite(float(actual)) and math.isfinite(float(expected)) and
                    abs(float(actual) - float(expected)) <= .011)
        except (TypeError, ValueError):
            return False

    def _matches(self, record, line):
        spec, tokens = record["spec"], record["command"].split()
        family = spec["family"]
        if spec["external"]:
            return None
        if family == "RGB":
            return True if line == "ok RGB applied" else False if line.startswith("error RGB ") else None
        if family == "SET_PID":
            match = re.fullmatch(r"ok SET PID \[([1-8])\] kp=(\d+) kv=(\d+) ki=(\d+) kd=(\d+)(?: \((?:rail|HAND)\))?", line)
            if match and len(tokens) == 6 and list(map(int, match.groups())) == list(map(int, tokens[1:])):
                return True
            return False if re.match(rf"error SET PID \[{re.escape(tokens[1])}\] ", line) else None
        if family in ("ACC_J", "I_LIMIT_J"):
            key = "ACC" if family == "ACC_J" else "I_LIMIT"
            match = re.fullmatch(rf"ok SET MOTOR \[([1-8])\] {key} \[(-?[\d.]+)\](?: \(HAND\))?( \(saved\))?", line)
            if (match and len(tokens) >= 3 and match[1] == tokens[1] and
                    self._near(match[2], tokens[2]) and bool(match[3]) == spec["save"]):
                return True
            return False if (line.startswith(f"error {family} ") or
                             re.match(rf"error SET MOTOR \[{re.escape(tokens[1])}\] {key} ", line)) else None
        if family.startswith("HAND_"):
            key = family[5:]
            match = re.fullmatch(rf"ok hand (?:set )?{key}=(-?[\d.]+)", line)
            expected = "1" if key == "ERASE" else tokens[1] if len(tokens) > 1 else "1"
            if match and self._near(match[1], expected):
                return True
            return False if line.startswith(f"error hand {key}") else None
        if family == "SYNC_ACC":
            return True if line == "ok SYNC_ACC finish" else False if line.startswith("error SYNC_ACC ") else None
        return None

    def _observe_parameter(self, record, line):
        """Independently verify SET values; the business task owns its gate."""
        spec, tokens = record["spec"], record["command"].split()
        if not spec["external"] or spec["family"] not in ("STALL", "CAL") or len(tokens) < 2:
            return
        family, operation = tokens[0][1:].split("_")
        if line == f"ok {family}_{operation} pending":
            record["parameter_accepted"] = True
            return
        if not record.get("parameter_accepted"):
            return
        node = int(tokens[1])
        nodes = {node} if node else ({7} if len(tokens) > 2 and tokens[2] == "BACK_MM" else
                                    set(range(1, 7)) if len(tokens) > 2 and tokens[2] == "BACK_DEG" else
                                    set(range(1, 8)))
        if operation == "SAVE":
            match = re.fullmatch(rf"ok {family}_SAVE \[([1-8])\] saved", line)
            if match and int(match[1]) in nodes:
                record.setdefault("verified_nodes", set()).add(int(match[1]))
        elif operation == "SET" and len(tokens) == 4:
            match = re.fullmatch(rf"ok {family}_SET \[([1-8])\] ([A-Z_0-9]+)=([^\s]+)(?:\s+.*)?", line)
            if not match or int(match[1]) not in nodes:
                return
            expected_key, expected_value = tokens[2], float(tokens[3])
            if expected_key == "BACK_DISTANCE_R":
                expected_key = "BACK_MM" if int(match[1]) == 7 else "BACK_DEG"
                expected_value *= 5 if int(match[1]) == 7 else 360 / (30 if int(match[1]) == 6 else 50)
            try:
                valid = match[2] == expected_key and math.isclose(float(match[3]), expected_value, rel_tol=1e-5, abs_tol=1e-6)
            except ValueError:
                valid = False
            if valid:
                record.setdefault("verified_nodes", set()).add(int(match[1]))
        record["verified_complete"] = record.get("verified_nodes", set()) == nodes

    def receive(self, line, generation, connection):
        with self.lock:
            for record in self._records:
                if (record["connection"] != connection or record["admitted_at"] is None or record["consumed"]):
                    continue
                self._observe_parameter(record, line)
                matched = self._matches(record, line)
                if matched is None:
                    continue
                record["consumed"] = True
                # Timed-out/cancelled admissions absorb their late replies.
                if (record["terminal"] or record["generation"] != generation or
                        (connection, record["spec"]["family"]) in self._ambiguous):
                    return
                record["terminal"] = True
                if not matched:
                    self._report(record, "error", f"{record['spec']['operation']}未完成：{line}",
                                 "请检查参数和设备状态后重试。", "device_rejected")
                elif record["spec"]["family"] in ("RGB", "SYNC_ACC", "HAND_I"):
                    self._report(record, "success", f"{record['spec']['operation']}已确认。",
                                 evidence="device_confirmed")
                else:
                    save = record["spec"]["save"]
                    result = "写入" if save else "应用"
                    boundary = "电机保存完成" if save else "电机已应用成功"
                    self._report(record, "success", f"主控已确认请求，电机{result}结果未回报。",
                                 f"可读取参数核对；此回执不代表{boundary}。", "request_confirmed")
                return

    def finish(self, task_id, success, detail="", uncertain=False):
        with self.lock:
            record = next((item for item in self._records if item["id"] == task_id), None)
            if record is None or record["terminal"]:
                return
            ambiguous = (record["connection"], record["spec"]["family"]) in self._ambiguous
            unverified = record["spec"]["external"] and not record.get("verified_complete")
            if success and (record["admitted_at"] is None or ambiguous or unverified):
                success, uncertain = False, True
                detail = f"{record['spec']['operation']}结果未确认，当前回执无法可靠关联本次请求。"
            if uncertain and record["admitted_at"] is not None and record["spec"]["external"]:
                self._ambiguous.add((record["connection"], record["spec"]["family"]))
                ambiguous = True
            record["terminal"] = True
            record["consumed"] = True
            self._report(record, "success" if success else "uncertain" if uncertain else "error",
                         f"{record['spec']['operation']}已确认。" if success else detail,
                         "" if success else
                         "本次回执无法可靠关联；请断开重连、读取配置后核实再操作。" if ambiguous else
                         "请检查设备状态后重新操作。",
                         "device_confirmed" if success else "unconfirmed" if uncertain else "device_rejected")

    def expire(self):
        with self.lock:
            now = self.clock()
            for record in self._records:
                if record["terminal"] or record["spec"]["external"]:
                    continue
                since = record["admitted_at"] if record["admitted_at"] is not None else record["requested_at"]
                if now - since >= self.TIMEOUT_SECONDS:
                    record["terminal"] = True
                    self._report(record, "uncertain", f"{record['spec']['operation']}结果未确认。",
                                 "请检查连接和设备状态；不会自动重发。", "unconfirmed")

    def invalidate(self, generation, connection, disconnected=False):
        with self.lock:
            for record in self._records:
                if record["terminal"] or (record["generation"] == generation and record["connection"] == connection and not disconnected):
                    continue
                record["terminal"] = True
                written = record["admitted_at"] is not None
                self._report(record, "uncertain" if written else "cancelled",
                             f"{record['spec']['operation']}" + ("结果未确认。" if written else "请求已取消。"),
                             "连接恢复并确认设备状态后再操作。" if disconnected else "请确认设备状态后重新操作。",
                             "unconfirmed" if written else "local")

    def new_connection(self):
        with self.lock:
            self._records.clear()
            self._ambiguous.clear()
