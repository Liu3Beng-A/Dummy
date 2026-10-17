"""Independent parameter drafts, source-addressed readback and sent snapshots."""
from copy import deepcopy
from decimal import Decimal, InvalidOperation
import re
import threading

from app.protocol.constants import STALL_PARAMETERS, CAL_PARAMETERS, RAIL_PID_DEFAULTS


def equivalent(left, right):
    """Formatting-only edits do not change a finite numeric parameter."""
    if str(left) == str(right):
        return True
    try:
        a, b = Decimal(str(left).strip()), Decimal(str(right).strip())
        return a.is_finite() and b.is_finite() and a == b
    except InvalidOperation:
        return False


def command_parameters(command):
    """Actual setter operands only; SAVE of existing MCU RAM is not a draft send."""
    tokens = str(command).strip().split()
    if not tokens:
        return ()
    verb = tokens[0]
    values = []
    node = int(tokens[1]) if len(tokens) > 1 and re.fullmatch(r"[0-8]", tokens[1]) else None
    if verb == "#SET_PID" and len(tokens) == 6:
        if node not in range(1, 9):
            return ()
        values = [("motor", node, key, value)
                  for key, value in zip(("KP", "KV", "KI", "KD"), tokens[2:])]
    elif verb in ("#ACC_J", "#I_LIMIT_J") and len(tokens) in (3, 4):
        if node not in range(1, 9):
            return ()
        values = [("motor", node, "ACC" if verb == "#ACC_J" else "I", tokens[2])]
    elif re.fullmatch(r"!HAND_(KP|KV|KI|KD|ACC|VEL|ONBOOT)", verb):
        if len(tokens) == 2 or (verb == "!HAND_ONBOOT" and len(tokens) == 1):
            values = [("motor", 8, verb[6:], tokens[1] if len(tokens) == 2 else "1")]
    elif verb in ("#STALL_SET", "#CAL_SET") and len(tokens) == 4:
        family = verb[1:].split("_")[0]
        keys = {row[0] for row in (STALL_PARAMETERS if family == "STALL" else CAL_PARAMETERS)}
        if family == "STALL":
            keys.add("BACK_DISTANCE_R")
        if ((family == "STALL" and node not in range(8)) or
                (family == "CAL" and node not in (7, 8)) or tokens[2] not in keys):
            return ()
        values = [(family, node, tokens[2], tokens[3])]
    return tuple(dict(family=family, node=node, key=key, value=str(value))
                 for family, node, key, value in values)


class ParameterEdits:
    """No I/O, validation gate or command submission lives in this model."""
    def __init__(self, changed=None):
        self._changed = changed or (lambda: None)
        self._items = {}
        self._feedback = {}
        self._connection = None
        self._connected = False
        self._lock = threading.RLock()

    @staticmethod
    def _address(family, node, key):
        return str(family), int(node), str(key)

    def ensure(self, family, node, key, default, queryable=True):
        address = self._address(family, node, key)
        with self._lock:
            if address not in self._items:
                if address[0] == "motor" and address[1] == 7:
                    default = RAIL_PID_DEFAULTS.get(address[2], default)
                self._items[address] = dict(draft=str(default), default=str(default), readback=None,
                                            dirty=False, edited=False, fresh=False,
                                            queryable=bool(queryable), sent=None)
            return self.snapshot(family, node, key)

    def snapshot(self, family, node, key):
        with self._lock:
            item = self._items.get(self._address(family, node, key))
            if item is None:
                return dict(draft="", readback=None, dirty=False, fresh=False,
                            queryable=True, sent=None)
            return deepcopy({name: item[name] for name in
                             ("draft", "readback", "dirty", "fresh", "queryable", "sent")})

    def edit(self, family, node, key, value):
        with self._lock:
            address = self._address(family, node, key)
            if address not in self._items:
                self.ensure(family, node, key, "")
            item = self._items[address]
            before = deepcopy(item)
            value = str(value)
            changed_value = not equivalent(value, item["draft"])
            item["draft"] = value
            if changed_value:
                item["edited"] = True
            item["dirty"] = (not equivalent(value, item["readback"]) if item["readback"] is not None
                             else item["edited"])
            changed = item != before
        if changed:
            self._changed()
        return self.snapshot(family, node, key)

    def receive(self, family, node, key, value):
        with self._lock:
            self.ensure(family, node, key, value)
            item = self._items[self._address(family, node, key)]
            before = deepcopy(item)
            was_dirty = item["dirty"]
            item["readback"] = str(value)
            item["fresh"] = True
            if not was_dirty:
                item["draft"] = str(value)
            item["dirty"] = not equivalent(item["draft"], item["readback"])
            item["edited"] = item["dirty"]
            changed = item != before
        if changed:
            self._changed()
        return self.snapshot(family, node, key)

    def restore(self, family, node, keys=None):
        wanted = None if keys is None else {keys} if isinstance(keys, str) else set(keys)
        restored = []
        with self._lock:
            for (item_family, item_node, key), item in self._items.items():
                if (item_family != str(family) or item_node != int(node) or
                        (wanted is not None and key not in wanted) or item["readback"] is None):
                    continue
                if item["draft"] != item["readback"] or item["dirty"]:
                    item["draft"], item["dirty"], item["edited"] = item["readback"], False, False
                    restored.append(key)
        if restored:
            self._changed()
        return tuple(restored)

    def set_connection(self, connected, identity=None):
        with self._lock:
            boundary = bool(connected) != self._connected or identity != self._connection
            self._connected, self._connection = bool(connected), identity
            if not boundary:
                return
            for item in self._items.values():
                item["fresh"] = False
                if item["sent"] is not None:
                    item["sent"]["fresh"] = False
        self._changed()

    def record_sent(self, command, task_id, connection=None):
        values = command_parameters(command)
        if not values or task_id is None:
            return False
        with self._lock:
            for value in values:
                family, node, key = value["family"], value["node"], value["key"]
                self.ensure(family, node, key, value["value"], queryable=key not in ("VEL", "ONBOOT"))
                self._items[self._address(family, node, key)]["sent"] = dict(
                    task_id=task_id, command=str(command), value=value["value"], values=deepcopy(values),
                    phase="pending", evidence="queued", message="请求已排队，尚未确认写出。",
                    connection=connection, fresh=self._connected and connection == self._connection,
                    updated_at=None)
            if task_id in self._feedback:
                self._update_feedback_locked(self._feedback[task_id])
        self._changed()
        return True

    def _update_feedback_locked(self, event):
        changed = False
        for item in self._items.values():
            sent = item["sent"]
            if sent is not None and sent["task_id"] == event.get("task_id", event.get("id")):
                for key in ("phase", "evidence", "message", "next_step", "updated_at"):
                    if key in event and sent.get(key) != event[key]:
                        sent[key] = deepcopy(event[key])
                        changed = True
        return changed

    def update_feedback(self, event):
        task_id = event.get("task_id", event.get("id"))
        if task_id is None:
            return
        with self._lock:
            previous = self._feedback.get(task_id, {})
            terminal = {"success", "error", "uncertain", "cancelled", "blocked", "invalid"}
            if previous.get("phase") in terminal and event.get("phase") == "pending":
                return
            old_stamp, new_stamp = previous.get("updated_at"), event.get("updated_at")
            if isinstance(old_stamp, (int, float)) and isinstance(new_stamp, (int, float)) and new_stamp < old_stamp:
                return
            self._feedback[task_id] = deepcopy(event)
            while len(self._feedback) > 128:
                del self._feedback[next(iter(self._feedback))]
            changed = self._update_feedback_locked(event)
        if changed:
            self._changed()
