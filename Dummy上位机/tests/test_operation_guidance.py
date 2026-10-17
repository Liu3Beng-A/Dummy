"""Action explanations are read-only and distinguish advice from refusal."""
from copy import deepcopy
import unittest

from app.core.operation_guidance import operation_actions


class OperationGuidanceTests(unittest.TestCase):
    def setUp(self):
        self.state = {"connected": True, "motor_enabled": True, "cmdmode": 2,
                      "control_owner": "ui"}
        self.gamepad = {"device_connected": True, "input_age_s": .05, "centered": True}

    def actions(self, **updates):
        return operation_actions(dict(self.state, **updates), self.gamepad)

    def test_required_records_and_stable_shape(self):
        actions = self.actions()
        self.assertEqual(set(actions), {"start", "stop", "disable", "movej", "movel", "gamepad", "rgb_save"})
        for item in actions.values():
            self.assertEqual(set(item), {"label", "available", "status", "reason", "next_step"})
            self.assertEqual(item["available"], item["status"] != "blocked")
            self.assertTrue(item["reason"])
            self.assertTrue(item["next_step"])

    def test_missing_or_non_mapping_fake_snapshots_are_safe(self):
        class FakeSession:
            pass
        for state in ({}, None, FakeSession()):
            actions = operation_actions(state, FakeSession(), None)
            self.assertTrue(all(item["status"] == "blocked" for item in actions.values()))

    def test_snapshots_are_not_mutated(self):
        self.state["operation"] = {"kind": "start", "phase": "pending"}
        rgb = {"save_busy": True, "save_stage": "uncertain", "save_message": "等待核实"}
        before = deepcopy((self.state, self.gamepad, rgb))
        operation_actions(self.state, self.gamepad, rgb)
        self.assertEqual((self.state, self.gamepad, rgb), before)

    def test_disconnect_explains_all_normal_commands(self):
        actions = self.actions(connected=False)
        for key in ("start", "stop", "disable", "movej", "movel", "rgb_save"):
            self.assertFalse(actions[key]["available"])
            self.assertIn("连接", actions[key]["next_step"])

    def test_disabled_and_unknown_motion_are_advice_not_new_policy(self):
        for motor in (False, None):
            actions = self.actions(motor_enabled=motor)
            for key in ("movej", "movel"):
                self.assertTrue(actions[key]["available"])
                self.assertEqual(actions[key]["status"], "attention")
                self.assertEqual(actions[key]["next_step"], "建议先 START 并等待确认，再发送目标。")
            self.assertTrue(actions["start"]["available"])

    def test_start_wait_is_advice_and_not_new_gate(self):
        action = self.actions(operation={"kind": "start", "phase": "pending"})["start"]
        self.assertTrue(action["available"])
        self.assertEqual(action["status"], "attention")
        self.assertIn("等待", action["reason"])

    def test_completed_start_does_not_remain_pending(self):
        action = self.actions(operation={"kind": "start", "phase": "completed"})["start"]
        self.assertIn("已确认使能", action["reason"])

    def test_terminal_start_phases_do_not_remain_pending(self):
        for phase in ("cancelled", "rejected", "locked", "disconnected"):
            action = self.actions(operation={"kind": "start", "phase": phase})["start"]
            self.assertNotIn("正在等待本次使能结果", action["reason"], phase)
            self.assertIn("已确认使能", action["reason"], phase)

    def test_lock_blocks_motion_but_not_start_or_safety(self):
        actions = self.actions(stop_locked=True)
        for key in ("movej", "movel"):
            self.assertEqual(actions[key]["status"], "blocked")
            self.assertIn("锁定", actions[key]["reason"])
        for key in ("start", "stop", "disable"):
            self.assertTrue(actions[key]["available"])

    def test_parameter_calibration_and_diagnostic_tasks_leave_safety_available(self):
        for field in ("parameter_busy", "calibration_busy", "cartesian_busy"):
            actions = self.actions(**{field: True})
            for key in ("start", "movej", "movel", "rgb_save"):
                self.assertFalse(actions[key]["available"], (field, key))
            for key in ("stop", "disable"):
                self.assertTrue(actions[key]["available"])

    def test_preparing_save_blocks_normal_commands_but_not_stop(self):
        actions = self.actions(rgb_save_busy=True, rgb_save_stage="preparing")
        self.assertEqual(actions["start"]["status"], "blocked")
        self.assertEqual(actions["stop"]["status"], "available")

    def test_uncertain_save_never_suggests_repeating_the_write(self):
        actions = self.actions(rgb_save_busy=True, rgb_save_stage="uncertain")
        self.assertIn("不要重复保存", actions["rgb_save"]["next_step"])
        self.assertIn("重连核实", actions["rgb_save"]["next_step"])
        for key in ("stop", "disable"):
            self.assertEqual(actions[key]["status"], "attention")
            self.assertIn("Flash", actions[key]["next_step"])

    def test_rgb_snapshot_fallback_and_explicit_state_precedence(self):
        rgb = {"save_busy": True, "save_stage": "waiting", "save_message": "正在写入"}
        actions = operation_actions(self.state, self.gamepad, rgb)
        self.assertEqual(actions["movej"]["reason"], "正在写入")
        actions = operation_actions(dict(self.state, rgb_save_busy=False), self.gamepad, rgb)
        self.assertTrue(actions["movej"]["available"])

    def test_sequential_task_blocks_external_motion_not_start(self):
        actions = self.actions(sequence_active=True)
        self.assertFalse(actions["movej"]["available"])
        self.assertFalse(actions["movel"]["available"])
        self.assertTrue(actions["start"]["available"])

    def test_mode_seven_blocks_plain_motion_and_claim(self):
        actions = self.actions(cmdmode=7)
        for key in ("movej", "movel", "gamepad"):
            self.assertFalse(actions[key]["available"])
            self.assertIn("7", actions[key]["reason"])

    def test_native_capability_blocks_claim_not_finite_movel(self):
        actions = self.actions(native_tcp=True)
        self.assertFalse(actions["gamepad"]["available"])
        self.assertIn("不能解除", actions["gamepad"]["next_step"])
        self.assertTrue(actions["movel"]["available"])
        self.assertIn("有限单点", actions["movel"]["next_step"])

    def test_unknown_firmware_keeps_legacy_gamepad_path(self):
        self.assertTrue(self.actions()["gamepad"]["available"])

    def test_claim_does_not_require_centered_enabled_or_unlocked(self):
        self.gamepad["centered"] = False
        action = self.actions(motor_enabled=False, stop_locked=True)["gamepad"]
        self.assertTrue(action["available"])
        self.assertIn("确认实物到位后", action["next_step"])

    def test_claim_uses_session_error_without_guessing_mapping(self):
        self.gamepad.update(device_connected=False, error="设备布局与当前配置不符")
        action = self.actions()["gamepad"]
        self.assertFalse(action["available"])
        self.assertEqual(action["reason"], "设备布局与当前配置不符")
        self.gamepad.pop("error")
        self.gamepad["message"] = "请检查有效输入"
        self.assertEqual(self.actions()["gamepad"]["reason"], "请检查有效输入")

    def test_claim_input_age_boundary_and_missing_facts(self):
        for age in (None, .50001, float("inf"), float("nan")):
            self.gamepad["input_age_s"] = age
            self.assertFalse(self.actions()["gamepad"]["available"])
        self.gamepad["input_age_s"] = .5
        self.assertTrue(self.actions()["gamepad"]["available"])
        self.gamepad.pop("input_age_s")
        self.assertTrue(self.actions()["gamepad"]["available"])

    def test_claim_busy_tasks_but_not_background_position_poll(self):
        fields = ("position_sync_busy", "sequence_active", "parameter_busy", "acc_sync_waiting", "mode_request_busy", "calibration_busy")
        for field in fields:
            self.assertFalse(self.actions(**{field: True})["gamepad"]["available"], field)
        self.assertFalse(self.actions(mode_pending=2)["gamepad"]["available"])
        self.assertFalse(self.actions(mode_request={"id": 1})["gamepad"]["available"])
        self.assertTrue(self.actions(polling_enabled=True, position_sync_busy=False)["gamepad"]["available"])

    def test_claimed_session_explains_main_ui_ownership(self):
        self.gamepad.update(claimed=True, message="等待确认到位")
        actions = self.actions()
        self.assertEqual(actions["gamepad"]["next_step"], "等待确认到位")
        for key in ("start", "movej", "movel", "rgb_save"):
            self.assertFalse(actions[key]["available"])
        self.assertTrue(actions["stop"]["available"])

    def test_control_owner_without_full_session_still_explains_ownership(self):
        actions = self.actions(control_owner="gamepad")
        self.assertFalse(actions["movej"]["available"])
        self.assertFalse(actions["gamepad"]["available"])

    def test_pending_release_explains_wait_not_claim_again(self):
        self.gamepad.update(claimed=True, release_pending=True)
        action = self.actions()["gamepad"]
        self.assertIn("释放", action["reason"])
        self.assertIn("停止确认", action["next_step"])

    def test_rgb_save_requires_known_disabled_and_completed_tasks(self):
        for motor in (True, None):
            self.assertFalse(self.actions(motor_enabled=motor)["rgb_save"]["available"])
        self.assertTrue(self.actions(motor_enabled=False)["rgb_save"]["available"])
        for field in ("sequence_active", "acc_sync_waiting"):
            self.assertFalse(self.actions(motor_enabled=False, **{field: True})["rgb_save"]["available"])

    def test_locked_rgb_is_existing_dispatch_gate_before_disabled_check(self):
        action = self.actions(motor_enabled=False, stop_locked=True)["rgb_save"]
        self.assertFalse(action["available"])
        self.assertEqual(action["status"], "blocked")
        self.assertIn("锁定", action["reason"])
        self.assertIn("锁定", self.actions(motor_enabled=True, stop_locked=True)["rgb_save"]["reason"])

    def test_combined_busy_conditions_have_dispatch_order(self):
        action = self.actions(rgb_save_busy=True, parameter_busy=True, cartesian_busy=True,
                              stop_locked=True, calibration_busy=True)["movej"]
        self.assertIn("RGB", action["reason"])
        action = self.actions(parameter_busy=True, cartesian_busy=True, stop_locked=True)["movej"]
        self.assertIn("诊断", action["reason"])


if __name__ == "__main__":
    unittest.main()
