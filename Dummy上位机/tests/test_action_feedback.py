"""Pure feedback correlation: no queues, serial ports, dialogs or device calls."""
import unittest

from app.core.action_feedback import ActionFeedback, configuration_action


class ActionFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.events = []
        self.feedback = ActionFeedback(self.events.append, clock=lambda: self.now)

    def request(self, command, generation=1, connection=1, admitted=True):
        task_id = self.feedback.requested(command, generation, connection)
        if admitted:
            self.feedback.admitted(command, generation, connection)
        return task_id

    def test_only_configuration_writes_have_descriptors(self):
        for command in ("#GETJPOS", "#RGB_STATUS", "#RGB_GET", "#GET_PID 1", "#STALL_GET 1",
                        "#CAL_GET 7", ">0,10,80,0,90,0,0,35", "!HAND_POS 25", "!STOP", ""):
            self.assertIsNone(configuration_action(command), command)
        for command in ("!RGB_RESET", "!RGB_MODE 3", "!RGB_ENABLE 1", "#SET_PID 1 1 2 3 4",
                        "#ACC_J 7 12 &", "#CAL_SAVE 7", "!HAND_ONBOOT 0", "!HAND_I 1"):
            self.assertIsNotNone(configuration_action(command), command)

    def test_cache_is_copied_and_older_result_cannot_replace_newer_action(self):
        old = self.feedback.notify("motor", "保存PID", phase="pending", message="one")
        new = self.feedback.notify("motor", "应用加速度", phase="invalid", message="two")
        self.feedback.notify("motor", "保存PID", phase="success", task_id=old)
        snapshot = self.feedback.snapshot("motor")
        self.assertEqual(snapshot["id"], new)
        snapshot["message"] = "mutated"
        self.assertEqual(self.feedback.snapshot("motor")["message"], "two")

    def test_written_does_not_report_success_and_rx_may_precede_written(self):
        command = "#SET_PID 1 100 100 0 0"
        task_id = self.request(command)
        self.feedback.receive("ok SET PID [1] kp=100 kv=100 ki=0 kd=0", 1, 1)
        self.feedback.written(command, 1, 1)
        self.assertEqual(self.events[-1]["id"], task_id)
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["evidence"], "request_confirmed")
        self.assertIn("电机写入结果未回报", self.events[-1]["message"])
        self.assertIn("节点 1 · 保存PID", self.events[-1]["message"])
        self.assertIn("此回执不代表电机保存完成", self.events[-1]["next_step"])

    def test_no_receipt_before_admission_can_confirm_action(self):
        self.request("!RGB_BRIGHT 30", admitted=False)
        self.feedback.receive("ok RGB applied", 1, 1)
        self.assertEqual(self.events[-1]["phase"], "pending")

    def test_rgb_status_is_not_an_apply_receipt(self):
        self.request("!RGB_MODE 8")
        for line in ("RGB_STATUS on=1 auto=1 bright=100 current=100 mode=8", "RGB_GET end", "ok RGB saved"):
            self.feedback.receive(line, 1, 1)
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.feedback.receive("ok RGB applied", 1, 1)
        self.assertEqual(self.events[-1]["evidence"], "device_confirmed")

    def test_background_and_unmatched_and_duplicate_receipts_emit_nothing(self):
        self.request("#ACC_J 1 12 &")
        count = len(self.events)
        for line in ("ok SET MOTOR [2] ACC [12.00] (saved)", "ok SET MOTOR [1] ACC [13.00] (saved)",
                     "ok SET MOTOR [1] ACC [12.00]", "ok ACC [1] = 12.00 r/s^2"):
            self.feedback.receive(line, 1, 1)
        self.assertEqual(len(self.events), count)
        receipt = "ok SET MOTOR [1] ACC [12.00] (saved)"
        self.feedback.receive(receipt, 1, 1)
        self.feedback.receive(receipt, 1, 1)
        self.assertEqual(len(self.events), count + 1)
        self.assertEqual(self.events[-1]["evidence"], "request_confirmed")

    def test_timeout_tombstone_absorbs_late_reply_without_confirming_new_rgb(self):
        self.request("!RGB_BRIGHT 20")
        self.now += 6.1
        self.feedback.expire()
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        new = self.request("!RGB_BRIGHT 40")
        self.feedback.receive("ok RGB applied", 1, 1)
        self.assertEqual(self.feedback.snapshot("rgb")["id"], new)
        self.assertEqual(self.feedback.snapshot("rgb")["phase"], "pending")
        self.feedback.receive("ok RGB applied", 1, 1)
        self.assertEqual(self.feedback.snapshot("rgb")["phase"], "success")

    def test_generation_retirement_and_connection_identity_never_confirm_new_task(self):
        command = "!HAND_VEL 12"
        self.request(command)
        self.feedback.invalidate(2, 1)
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        self.request(command, generation=2)
        self.feedback.receive("ok hand set VEL=12.00", 2, 1)
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.feedback.receive("ok hand set VEL=12.00", 2, 99)
        self.assertEqual(self.events[-1]["phase"], "pending")

    def test_unwritten_cancellation_is_not_device_failure(self):
        self.request("!RGB_RESET", admitted=False)
        self.feedback.invalidate(2, 1)
        self.assertEqual(self.events[-1]["phase"], "cancelled")
        self.assertEqual(self.events[-1]["evidence"], "local")

    def test_gripper_each_ack_matches_its_actual_value(self):
        for command, reply, evidence in (
                ("!HAND_I 1.2", "ok hand I=1.20", "device_confirmed"),
                ("!HAND_ACC 20", "ok hand set ACC=20.00", "request_confirmed"),
                ("!HAND_KD 90", "ok hand set KD=90", "request_confirmed"),
                ("!HAND_ONBOOT 0", "ok hand set ONBOOT=0", "request_confirmed"),
                ("!HAND_ERASE", "ok hand set ERASE=1", "request_confirmed")):
            self.request(command)
            self.feedback.receive(reply, 1, 1)
            self.assertEqual(self.events[-1]["phase"], "success", command)
            self.assertEqual(self.events[-1]["evidence"], evidence, command)

    def test_external_parameter_success_needs_admission_and_ambiguity_is_conservative(self):
        task = self.request("#STALL_SAVE 1", admitted=False)
        self.feedback.finish(task, True)
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        old = self.request("#STALL_SAVE 1")
        self.feedback.finish(old, False, "超时", uncertain=True)
        new = self.request("#STALL_SAVE 1")
        self.feedback.finish(new, True)
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        self.assertIn("断开重连", self.events[-1]["next_step"])
        self.assertIn("读取配置后核实", self.events[-1]["next_step"])
        self.feedback.new_connection()
        new = self.request("#STALL_SAVE 1", connection=2)
        self.feedback.receive("ok STALL_SAVE pending", 1, 2)
        self.feedback.receive("ok STALL_SAVE [1] saved", 1, 2)
        self.feedback.finish(new, True)
        self.assertEqual(self.events[-1]["evidence"], "device_confirmed")

    def test_motor_apply_feedback_names_node_action_and_request_boundary(self):
        self.request("#ACC_J 7 12")
        self.feedback.receive("ok SET MOTOR [7] ACC [12.00]", 1, 1)
        self.assertIn("节点 7 · 应用加速度", self.events[-1]["message"])
        self.assertIn("电机应用结果未回报", self.events[-1]["message"])
        self.assertIn("不代表电机已应用成功", self.events[-1]["next_step"])

    def test_parameter_set_value_mismatch_cannot_confirm_even_if_business_finishes(self):
        task = self.request("#CAL_SET 7 SEEK_RPS 2")
        self.feedback.receive("ok CAL_SET pending", 1, 1)
        self.feedback.receive("ok CAL_SET [7] SEEK_RPS=3", 1, 1)
        self.feedback.finish(task, True)
        self.assertEqual(self.events[-1]["phase"], "uncertain")

    def test_bounded_history_does_not_treat_evicted_unanswered_receipt_as_new_success(self):
        for _ in range(self.feedback.LIMIT + 1):
            self.request("!RGB_MODE 3")
        self.assertLessEqual(len(self.feedback._records), self.feedback.LIMIT)
        self.feedback.receive("ok RGB applied", 1, 1)
        self.assertNotEqual(self.events[-1]["phase"], "success")


if __name__ == "__main__":
    unittest.main()
