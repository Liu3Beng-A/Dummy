"""Local configuration feedback stays visible without modal dialogs or writes."""
import unittest
from unittest.mock import patch

from app.core.business import RobotBusiness
from app.core.fields import ValueField
from app.protocol.constants import CAL_PARAMETERS, STALL_PARAMETERS


class FeedbackBusiness(RobotBusiness):
    """Only the established field/log/dispatch contract, with no device backend."""

    def __init__(self):
        self.logs, self.feedback, self.commands = [], [], []
        self.root = None
        self.is_connected = True
        self.serial_port = object()
        self._motor_state_known = True
        self._motor_enabled = False
        self._stop_locked = self._calibration_busy = self._seq_active = False
        self._acc_sync_waiting = False
        self.param_node = ValueField(1)
        self.ent_set_pid_kp, self.ent_set_pid_kv = ValueField("1"), ValueField("2")
        self.ent_set_pid_ki, self.ent_set_pid_kd = ValueField("3"), ValueField("4")
        self.ent_set_acc, self.ent_set_i = ValueField("5"), ValueField("1.5")
        self.hand_current = ValueField(1.0)
        self._rgb_bright = ValueField("50")
        self._rgb_color_vars = [ValueField("10") for _ in range(3)]
        self._rgb_auto_color_vars = [ValueField("20") for _ in range(3)]
        self._rgb_fades = [ValueField("300") for _ in range(4)]
        self._rgb_effect_first = [ValueField("40") for _ in range(3)]
        self._rgb_effect_second = [ValueField("0") for _ in range(3)]
        self._rgb_effect_params = [ValueField(value) for value in ("800", "3", "200", "1000")]
        self._rgb_color_idx, self._rgb_auto_idx, self._rgb_effect_mode = (ValueField() for _ in range(3))
        self._rgb_reverse = ValueField(False)
        self._rgb_state, self._rgb_state_mode, self._rgb_state_source = (ValueField() for _ in range(3))
        self._rgb_state_source.current(1)
        self._rgb_state_color = [ValueField("50") for _ in range(3)]
        self._rgb_motion_supported = True
        self._rgb_motion_idx, self._rgb_motion_kind = ValueField(), ValueField()
        self._rgb_motion_params = [ValueField("800"), ValueField("3")]
        self._rgb_motion_reverse = ValueField(False)
        self._protection_nodes = {"STALL": ValueField(1), "CAL": ValueField(7)}
        self._protection_entries = {
            kind: {row[0]: ValueField(str(row[3])) for row in descriptors}
            for kind, descriptors in (("STALL", STALL_PARAMETERS), ("CAL", CAL_PARAMETERS))
        }

    def log(self, message, tag):
        self.logs.append((message, tag))

    def notify_action(self, scope, operation, **details):
        self.feedback.append(dict(scope=scope, operation=operation, **details))

    def send_cmd(self, command):
        self.commands.append(command)
        return True


class BusinessFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.business = FeedbackBusiness()
        self.modal_error = patch("app.core.business.messagebox.showerror", side_effect=AssertionError("Unexpected modal error"))
        self.modal_warning = patch("app.core.business.messagebox.showwarning", side_effect=AssertionError("Unexpected modal warning"))
        self.modal_error.start()
        self.modal_warning.start()
        self.addCleanup(self.modal_error.stop)
        self.addCleanup(self.modal_warning.stop)

    def assert_problem(self, scope, phase="invalid"):
        event = self.business.feedback[-1]
        self.assertEqual(event["scope"], scope)
        self.assertEqual(event["phase"], phase)
        self.assertEqual(event["evidence"], "local")
        self.assertTrue(event["message"])
        self.assertTrue(event["next_step"])
        self.assertEqual(self.business.commands, [])
        self.assertEqual(self.business.logs[-1][1], "WARN")
        return event

    def test_each_rgb_apply_reports_the_invalid_field_without_sending(self):
        cases = (
            ("_rgb_apply_bright", "_rgb_bright", None),
            ("_rgb_apply_color", "_rgb_color_vars", 0),
            ("_rgb_apply_auto_color", "_rgb_auto_color_vars", 1),
            ("_rgb_apply_fades", "_rgb_fades", 3),
            ("_rgb_apply_effect", "_rgb_effect_params", 0),
            ("_rgb_apply_state", "_rgb_state_color", 2),
            ("_rgb_apply_motion", "_rgb_motion_params", 1),
        )
        for method, field, index in cases:
            with self.subTest(method=method):
                self.business = FeedbackBusiness()
                value = getattr(self.business, field)
                (value if index is None else value[index]).set("invalid")
                getattr(self.business, method)()
                event = self.assert_problem("rgb")
                self.assertTrue(event["target"])

    def test_unsupported_rgb_motion_reports_reason_and_retains_capability_gate(self):
        self.business._rgb_motion_supported = False
        self.business._rgb_apply_motion()
        self.assertIn("尚未确认支持", self.assert_problem("rgb")["message"])

    def test_rgb_save_preconditions_all_report_and_do_not_change_state(self):
        for field, value in (("_motor_state_known", False), ("_motor_enabled", True),
                             ("_stop_locked", True), ("_calibration_busy", True), ("_seq_active", True)):
            with self.subTest(field=field):
                self.business = FeedbackBusiness()
                setattr(self.business, field, value)
                self.business._rgb_save()
                event = self.assert_problem("rgb", "blocked")
                self.assertIn("DISABLE", event["next_step"])
                self.assertEqual(getattr(self.business, field), value)

    def test_valid_rgb_entry_dispatches_once_and_does_not_invent_success(self):
        self.business._rgb_apply_bright()
        self.business._rgb_save()
        self.assertEqual(self.business.commands, ["!RGB_BRIGHT 50", "!RGB_SAVE"])
        self.assertEqual(self.business.feedback, [])

    def test_motor_format_errors_are_nonmodal_and_name_the_operation(self):
        cases = (("_set_pid", "ent_set_pid_kp", "保存PID"),
                 ("_set_acc", "ent_set_acc", "应用加速度"),
                 ("_save_acc", "ent_set_acc", "保存加速度"),
                 ("_set_i", "ent_set_i", "应用电流限制"),
                 ("_save_i", "ent_set_i", "保存电流限制"))
        for method, field, operation in cases:
            with self.subTest(method=method):
                self.business = FeedbackBusiness()
                getattr(self.business, field).set("bad")
                getattr(self.business, method)()
                self.assertEqual(self.assert_problem("motor")["operation"], operation)

    def test_negative_acceleration_and_nonpositive_current_still_do_not_send(self):
        for method, field, value in (("_set_acc", "ent_set_acc", "-1"),
                                     ("_save_acc", "ent_set_acc", "-1"),
                                     ("_set_i", "ent_set_i", "0"),
                                     ("_save_i", "ent_set_i", "-1")):
            with self.subTest(method=method):
                self.business = FeedbackBusiness()
                getattr(self.business, field).set(value)
                getattr(self.business, method)()
                self.assert_problem("motor")

    def test_existing_motor_upper_clamps_and_persistence_suffix_stay_the_same(self):
        self.business.ent_set_acc.set("6000")
        self.business.ent_set_i.set("4")
        self.business._set_pid()
        self.business._set_acc()
        self.business._save_acc()
        self.business._set_i()
        self.business._save_i()
        self.assertEqual(self.business.commands, ["#SET_PID 1 1 2 3 4", "#ACC_J 1 5000.0",
                                                "#ACC_J 1 5000.0 &", "#I_LIMIT_J 1 3.0", "#I_LIMIT_J 1 3.0 &"])
        self.assertEqual(self.business.feedback, [])

    def test_protection_invalid_range_integer_and_nonfinite_values_report_locally(self):
        for kind, key, value, scope in (("STALL", "TRIGGER_MS", "20.5", "protection"),
                                        ("STALL", "SPEED_RPS", "0", "protection"),
                                        ("CAL", "CONTACT_RATIO", "2", "calibration"),
                                        ("CAL", "CURRENT_A", "nan", "calibration")):
            with self.subTest(kind=kind, key=key):
                self.business = FeedbackBusiness()
                self.business._protection_entries[kind][key].set(value)
                self.business._set_protection(kind, key)
                self.assert_problem(scope)

    def test_inapplicable_stall_parameters_remain_blocked_with_feedback(self):
        for node, key in ((8, "ENABLE"), (7, "BACK_DEG"), (1, "BACK_MM")):
            with self.subTest(node=node, key=key):
                self.business = FeedbackBusiness()
                self.business._protection_nodes["STALL"].set(node)
                self.business._set_protection("STALL", key)
                self.assert_problem("protection", "blocked")

    def test_protection_and_calibration_save_enabled_rejection_has_next_step(self):
        for kind, scope in (("STALL", "protection"), ("CAL", "calibration")):
            with self.subTest(kind=kind):
                self.business = FeedbackBusiness()
                self.business._motor_enabled = True
                self.business._save_protection(kind)
                self.assertIn("DISABLE", self.assert_problem(scope, "blocked")["next_step"])

    def test_unknown_motor_state_still_defers_protection_save_to_dispatch_gate(self):
        self.business._motor_state_known = False
        self.business._save_protection("CAL")
        self.assertEqual(self.business.commands, ["#CAL_SAVE 7"])
        self.assertEqual(self.business.feedback, [])

    def test_encoder_calibration_gate_reports_without_asking_confirmation(self):
        for field, value in (("_stop_locked", True), ("_calibration_busy", True),
                             ("_seq_active", True), ("_acc_sync_waiting", True),
                             ("_motor_state_known", False), ("_motor_enabled", True)):
            with self.subTest(field=field):
                self.business = FeedbackBusiness()
                setattr(self.business, field, value)
                with patch("app.core.business.messagebox.askyesno") as confirm:
                    self.business._cmd_encoder_calibration()
                confirm.assert_not_called()
                self.assert_problem("calibration", "blocked")

    def test_encoder_calibration_danger_confirmation_is_retained(self):
        with patch("app.core.business.messagebox.askyesno", return_value=False) as confirm:
            self.business._cmd_encoder_calibration()
        confirm.assert_called_once()
        self.assertEqual(self.business.commands, [])
        with patch("app.core.business.messagebox.askyesno", return_value=True) as confirm:
            self.business._cmd_encoder_calibration()
        confirm.assert_called_once()
        self.assertEqual(self.business.commands, ["#ENC_CAL 1"])
        self.assertEqual(self.business.feedback, [])

    def test_broadcast_encoder_calibration_is_rejected_locally(self):
        self.business.param_node.set(0)
        self.business._cmd_encoder_calibration()
        self.assertIn("ALL", self.assert_problem("calibration")["message"])

    def test_zero_calibration_disabled_requirement_is_nonmodal(self):
        self.business._motor_enabled = True
        with patch("app.core.business.messagebox.askyesno") as confirm:
            self.business._cmd_calibration()
        confirm.assert_not_called()
        self.assert_problem("calibration", "blocked")
        self.assertTrue(self.business._motor_enabled)

    def test_zero_calibration_motion_confirmation_is_retained(self):
        with patch("app.core.business.messagebox.askyesno", return_value=False) as confirm:
            self.business._cmd_calibration()
        confirm.assert_called_once()
        self.assertEqual(self.business.commands, [])

    def test_gripper_current_invalid_input_is_nonmodal_and_never_changes_field(self):
        for value in ("bad", float("nan"), float("inf")):
            with self.subTest(value=value):
                self.business = FeedbackBusiness()
                self.business.hand_current.set(value)
                self.business._cmd_hand_set_i()
                self.assert_problem("gripper")
                actual = self.business.hand_current.get()
                self.assertIs(actual, value)

    def test_gripper_runtime_current_clamp_still_dispatches_without_false_success(self):
        self.business.hand_current.set(3)
        self.business._cmd_hand_set_i()
        self.assertEqual(self.business.hand_current.get(), 2.5)
        self.assertEqual(self.business.commands, ["!HAND_I 2.50"])
        self.assertEqual(self.business.feedback, [])

    def test_log_only_adapter_does_not_require_controller_feedback_api(self):
        self.business.notify_action = None
        self.business._rgb_bright.set("bad")
        self.business._rgb_apply_bright()
        self.assertTrue(self.business.logs)
        self.assertEqual(self.business.commands, [])


if __name__ == "__main__":
    unittest.main()
