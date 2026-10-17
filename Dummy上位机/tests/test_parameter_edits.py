"""Draft/readback separation and immutable configuration command snapshots."""
import unittest

from app.core.parameter_edits import ParameterEdits, command_parameters


class ParameterEditsTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.store = ParameterEdits(lambda: self.events.append(True))

    def test_ensure_default_is_clean_and_not_a_readback_and_does_not_notify(self):
        state = self.store.ensure("motor", 1, "ACC", "150")
        self.assertEqual(state["draft"], "150")
        self.assertIsNone(state["readback"])
        self.assertFalse(state["dirty"])
        self.assertFalse(state["fresh"])
        self.assertEqual(self.events, [])
        self.assertEqual(self.store.restore("motor", 1), ())

    def test_rail_pid_defaults_are_local_clean_and_only_used_for_new_rail_pid_fields(self):
        expected = {"KP": "150", "KV": "180", "KI": "25", "KD": "50"}
        for key, value in expected.items():
            state = self.store.ensure("motor", 7, key, "0")
            self.assertEqual(state["draft"], value)
            self.assertIsNone(state["readback"])
            self.assertIsNone(state["sent"])
            self.assertFalse(state["dirty"])
            self.assertFalse(state["fresh"])
            self.assertEqual(self.store.ensure("motor", 1, key, "0")["draft"], "0")
            self.assertEqual(self.store.ensure("motor", 8, key, "0")["draft"], "0")
        self.assertEqual(self.store.ensure("motor", 7, "ACC", "150")["draft"], "150")
        self.assertEqual(self.store.ensure("CAL", 7, "KP", "99")["draft"], "99")
        self.assertEqual(self.events, [])

    def test_rail_default_cannot_override_user_draft_or_an_actual_returned_value(self):
        self.store.ensure("motor", 7, "KP", "0")
        self.store.edit("motor", 7, "KP", "-")
        self.store.ensure("motor", 7, "KP", "0")
        self.assertEqual(self.store.snapshot("motor", 7, "KP")["draft"], "-")
        self.store.receive("motor", 7, "KV", "201")
        state = self.store.ensure("motor", 7, "KV", "0")
        self.assertEqual((state["draft"], state["readback"]), ("201", "201"))
        self.assertTrue(state["fresh"])

    def test_receive_updates_clean_fields_and_preserves_individual_dirty_field(self):
        self.store.ensure("motor", 1, "KP", "100")
        self.store.ensure("motor", 1, "KV", "100")
        self.store.edit("motor", 1, "KP", "-")
        self.store.receive("motor", 1, "KP", "12")
        self.store.receive("motor", 1, "KV", "25")
        kp = self.store.snapshot("motor", 1, "KP")
        self.assertEqual((kp["draft"], kp["readback"], kp["dirty"], kp["fresh"]), ("-", "12", True, True))
        self.assertEqual(self.store.snapshot("motor", 1, "KV")["draft"], "25")

    def test_numeric_formatting_is_equal_and_invalid_raw_draft_is_retained(self):
        self.store.ensure("motor", 1, "ACC", "150")
        self.assertFalse(self.store.edit("motor", 1, "ACC", "1.50e2")["dirty"])
        self.store.receive("motor", 1, "ACC", "150.00")
        self.assertFalse(self.store.edit("motor", 1, "ACC", "150.000")["dirty"])
        self.assertTrue(self.store.edit("motor", 1, "ACC", "1e")["dirty"])
        self.store.receive("motor", 1, "ACC", "100")
        self.assertEqual(self.store.snapshot("motor", 1, "ACC")["draft"], "1e")

    def test_without_baseline_user_change_stays_dirty_even_after_return_to_default(self):
        self.store.ensure("motor", 1, "ACC", "150")
        self.store.edit("motor", 1, "ACC", "50")
        self.assertTrue(self.store.edit("motor", 1, "ACC", "150")["dirty"])

    def test_restore_can_use_stale_last_readback_but_does_not_make_it_fresh(self):
        self.store.set_connection(True, 1)
        self.store.receive("motor", 1, "ACC", "25")
        self.store.edit("motor", 1, "ACC", "50")
        self.store.set_connection(False, 2)
        self.assertEqual(self.store.restore("motor", 1, ("ACC",)), ("ACC",))
        state = self.store.snapshot("motor", 1, "ACC")
        self.assertEqual(state["draft"], "25")
        self.assertFalse(state["dirty"])
        self.assertFalse(state["fresh"])

    def test_node_and_family_drafts_are_independent_and_all_has_no_fake_baseline(self):
        for family, node, key, value in (("motor", 1, "KP", "5"), ("motor", 2, "KP", "9"),
                                         ("STALL", 0, "ENABLE", "0"), ("STALL", 1, "ENABLE", "1"),
                                         ("CAL", 7, "CURRENT_A", "2")):
            self.store.ensure(family, node, key, value)
        self.store.receive("STALL", 1, "ENABLE", "1")
        self.store.edit("motor", 1, "KP", "7")
        self.assertEqual(self.store.snapshot("motor", 2, "KP")["draft"], "9")
        self.assertIsNone(self.store.snapshot("STALL", 0, "ENABLE")["readback"])

    def test_reconnect_retains_dirty_draft_and_marks_readback_and_sent_historical(self):
        self.store.set_connection(True, 3)
        self.store.receive("motor", 8, "ACC", "10")
        self.store.edit("motor", 8, "ACC", "20")
        self.store.record_sent("!HAND_ACC 20", 1, 3)
        self.store.set_connection(True, 4)
        state = self.store.snapshot("motor", 8, "ACC")
        self.assertFalse(state["fresh"])
        self.assertFalse(state["sent"]["fresh"])
        self.assertEqual(state["sent"]["connection"], 3)
        self.store.receive("motor", 8, "ACC", "30")
        self.assertEqual(self.store.snapshot("motor", 8, "ACC")["draft"], "20")

    def test_ack_updates_only_matching_sent_metadata_and_does_not_clean_new_draft(self):
        self.store.receive("motor", 1, "ACC", "10")
        self.store.edit("motor", 1, "ACC", "20")
        self.store.record_sent("#ACC_J 1 20 &", 7, 1)
        self.store.edit("motor", 1, "ACC", "30")
        self.store.update_feedback(dict(task_id=6, phase="success", message="old"))
        self.assertEqual(self.store.snapshot("motor", 1, "ACC")["sent"]["phase"], "pending")
        self.store.update_feedback(dict(task_id=7, phase="success", evidence="request_confirmed", message="forwarded"))
        state = self.store.snapshot("motor", 1, "ACC")
        self.assertEqual((state["draft"], state["readback"], state["dirty"]), ("30", "10", True))
        self.assertEqual(state["sent"]["value"], "20")
        self.assertEqual(state["sent"]["evidence"], "request_confirmed")

    def test_sent_snapshot_copy_and_early_receipt_are_safe(self):
        self.store.update_feedback(dict(task_id=1, phase="success", evidence="request_confirmed"))
        self.store.record_sent("#SET_PID 8 1 2 3 4", 1, 5)
        state = self.store.snapshot("motor", 8, "KP")
        self.assertEqual(state["sent"]["phase"], "success")
        state["sent"]["values"][0]["value"] = "999"
        self.assertEqual(self.store.snapshot("motor", 8, "KP")["sent"]["value"], "1")
        self.assertEqual(self.store.snapshot("motor", 8, "KP")["sent"]["values"][0]["value"], "1")

    def test_unqueryable_defaults_are_not_fabricated_as_readback(self):
        self.store.ensure("motor", 8, "VEL", "30", queryable=False)
        self.store.record_sent("!HAND_VEL 20", 2, 1)
        state = self.store.snapshot("motor", 8, "VEL")
        self.assertFalse(state["queryable"])
        self.assertIsNone(state["readback"])

    def test_sent_feedback_cannot_regress_from_old_pending_or_old_timestamp(self):
        self.store.record_sent("#ACC_J 1 20", 1, 1)
        self.store.update_feedback(dict(task_id=1, phase="success", updated_at=20, message="confirmed"))
        self.store.update_feedback(dict(task_id=1, phase="pending", updated_at=21, message="late written"))
        self.store.update_feedback(dict(task_id=1, phase="error", updated_at=19, message="old"))
        sent = self.store.snapshot("motor", 1, "ACC")["sent"]
        self.assertEqual((sent["phase"], sent["message"]), ("success", "confirmed"))

    def test_save_ram_and_queries_do_not_claim_to_have_sent_unapplied_drafts(self):
        self.store.ensure("STALL", 1, "ENABLE", "1")
        self.store.edit("STALL", 1, "ENABLE", "0")
        for command in ("#STALL_SAVE 1", "#CAL_SAVE 7", "#GET_PID 1", "!HAND_ERASE", "!HAND_I 1"):
            self.assertEqual(command_parameters(command), (), command)
            self.assertFalse(self.store.record_sent(command, 1, 1), command)
        self.assertIsNone(self.store.snapshot("STALL", 1, "ENABLE")["sent"])

    def test_malformed_or_unknown_console_addresses_are_not_materialized(self):
        for command in ("#STALL_SET X ENABLE 1", "#CAL_SET 99 SEEK_RPS 1",
                        "#STALL_SET 1 UNKNOWN_KEY 1", "#SET_PID X 1 2 3 4",
                        "#ACC_J 99 1", "#CAL_SET 0 SEEK_RPS 1"):
            self.assertEqual(command_parameters(command), (), command)
            self.assertFalse(self.store.record_sent(command, 99), command)
        self.assertEqual(self.store._items, {})


if __name__ == "__main__":
    unittest.main()
