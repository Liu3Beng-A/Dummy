"""Protocol evidence for presentation only: no writer, queries, or motion gates."""
import unittest

from app.core.operation_tracker import OperationTracker


MOVEJ = ">0,10,80,0,90,0,0,35"
MOVEL = "@146.52,0,288.92,180,0,180,35"


class OperationTrackerTests(unittest.TestCase):
    def setUp(self):
        self.now = 20.
        self.t = OperationTracker(lambda: self.now)
        self.context = {"connected": True, "generation": 1, "motor_enabled": True}

    def enqueue(self, command=MOVEJ, generation=1, source="ui"):
        self.t.requested(command, generation, 0, source)

    def admit(self, command=MOVEJ, generation=1):
        self.t.admitted(command, generation, 0)

    def rx(self, line, generation=1, connection=0, **feedback):
        self.t.receive(line, generation, connection, **feedback)

    def view(self, **changes):
        return self.t.snapshot({**self.context, **changes})

    def numbered(self, command=MOVEJ, number=1, kind="MOVEJ", mode="INT"):
        self.enqueue(command)
        self.admit(command)
        self.t.written(command, 1, 0)
        self.rx(f"queued {kind} id={number} mode={mode}")

    def feedback(self, pose=(0., 10., 80., 0., 90., 0., 0.), **changes):
        self.context.update(actual_pose=list(pose), actual_received_at=self.now,
                            actual_feedback_sequence=self.context.get("actual_feedback_sequence", 0)+1,
                            actual_stale=False, polling_paused=False)
        self.context.update(changes)

    def accepted_movej(self, **numbered):
        self.numbered(**numbered)
        number = numbered.get("number", 1)
        self.rx(f"accepted MOVEJ id={number}",
                feedback_sequence=self.context.get("actual_feedback_sequence", 0), feedback_after=self.now)

    def test_schema_and_unconfirmed_connection(self):
        state = self.view(motor_enabled=None)
        self.assertEqual(state["phase"], "uncertain")
        self.assertTrue({"kind", "phase", "title", "reason", "next_step", "tone", "elapsed_s",
                         "command", "motion_id", "source"} <= state.keys())

    def test_no_request_or_timing_origin_leaves_elapsed_unknown(self):
        for changes in ({"motor_enabled": None}, {"connected": False},
                        {"calibration_busy": True},
                        {"cartesian_busy": True, "cartesian_message": "等待诊断结果"}):
            with self.subTest(changes=changes):
                self.assertIsNone(self.view(**changes)["elapsed_s"])
        self.enqueue("!START")
        self.now += .4
        self.assertEqual(self.view()["elapsed_s"], .4)

    def test_queue_is_not_written_or_accepted(self):
        self.enqueue()
        self.assertEqual(self.view()["phase"], "queued")
        self.rx("queued MOVEJ id=1 mode=INT")
        self.assertIsNone(self.view()["motion_id"])
        self.admit()
        self.assertEqual(self.view()["phase"], "sending")
        self.t.written(MOVEJ, 1, 0)
        self.assertEqual(self.view()["phase"], "waiting")

    def test_rx_may_precede_qt_written_and_requested(self):
        self.admit()
        self.rx("queued MOVEJ id=4 mode=INT")
        self.rx("accepted MOVEJ id=4")
        self.enqueue(source="gamepad")
        self.t.written(MOVEJ, 1, 0)
        self.assertEqual(len(self.t.records), 1)
        self.assertEqual(self.view()["phase"], "accepted")
        self.assertEqual(self.view()["source"], "gamepad")

    def test_accepted_is_never_completion_even_after_time(self):
        self.numbered()
        self.rx("accepted MOVEJ id=1")
        self.now += 300
        self.assertEqual(self.view()["phase"], "accepted")
        self.assertIn("尚未确认到位", self.view()["reason"])

    def test_new_feedback_observes_int_target_without_protocol_completion(self):
        self.accepted_movej()
        self.feedback()
        view = self.view()
        self.assertEqual(view["phase"], "observed_reached")
        self.assertIn("回读位置已到目标", view["title"])
        self.assertEqual(view["tone"], "success")
        self.assertEqual(self.t.latest["phase"], "accepted")
        self.assertEqual(self.t.latest["position_target"], (0., 10., 80., 0., 90., 0., 0.))

    def test_cached_matching_position_and_late_old_sample_do_not_observe_target(self):
        self.feedback()
        self.now += 1.
        self.accepted_movej()
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback(actual_received_at=self.now-.01)
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_queued_target_needs_acceptance_and_post_acceptance_sample(self):
        self.numbered()
        self.feedback()
        self.assertEqual(self.view()["phase"], "queued")
        self.rx("accepted MOVEJ id=1", feedback_sequence=1, feedback_after=self.now)
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_int_position_matching_reuses_display_tolerances_and_release_hysteresis(self):
        self.accepted_movej()
        self.feedback((.1, 10.1, 80.1, .1, 90.1, .1, .1))
        self.assertEqual(self.view()["phase"], "observed_reached")
        self.feedback((.2, 10.2, 80.2, .2, 90.2, .2, .2))
        self.assertEqual(self.view()["phase"], "observed_reached")
        self.feedback((0., 10., 80., 0., 90., 0., .20001))
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback((0., 10., 80., 0., 90., 0., .15))
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_real_joint_angles_are_not_wrapped_or_clipped_for_observation(self):
        self.accepted_movej()
        self.feedback((360., 10., 80., 0., 90., 0., 0.))
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback((0., 10., 80., 0., 90., 0., float("nan")))
        self.assertEqual(self.view()["phase"], "accepted")

    def test_stale_or_paused_position_withdraws_only_observed_state(self):
        for condition in ("actual_stale", "polling_paused"):
            with self.subTest(condition=condition):
                self.setUp()
                self.accepted_movej()
                self.feedback()
                self.assertEqual(self.view()["phase"], "observed_reached")
                self.assertEqual(self.view(**{condition: True})["phase"], "accepted")
                self.assertEqual(self.t.latest["phase"], "accepted")

    def test_pause_while_rgb_status_is_visible_resets_position_hysteresis(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")
        view = self.view(polling_paused=True, rgb_save={"active": True, "stage": "waiting"})
        self.assertEqual(view["kind"], "rgb_save")
        self.feedback((0., 10., 80., 0., 90., 0., .15))
        self.assertEqual(self.view()["phase"], "accepted")

    def test_device_completion_and_failure_remain_authoritative(self):
        for receipt, phase in (("ok MOVEJ id=1", "completed"),
                               ("error MOVEJ id=1 reason=PLAN_FAILED", "rejected"),
                               ("error MOVEJ id=1 reason=CANCELLED", "cancelled")):
            with self.subTest(receipt=receipt):
                self.setUp()
                self.accepted_movej()
                self.feedback()
                self.assertEqual(self.view()["phase"], "observed_reached")
                self.rx(receipt)
                self.assertEqual(self.view()["phase"], phase)

    def test_late_accepted_does_not_reset_observed_position(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")
        self.now += .2
        self.rx("accepted MOVEJ id=1", feedback_sequence=1, feedback_after=self.now)
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_newer_int_target_cannot_reuse_previous_acceptance_or_feedback(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")
        self.enqueue()
        self.admit()
        self.rx("queued MOVEJ id=2 mode=INT")
        self.rx("accepted MOVEJ id=1")
        self.assertEqual(self.view()["phase"], "queued")
        self.rx("accepted MOVEJ id=2", feedback_sequence=1, feedback_after=self.now)
        self.assertEqual(self.view()["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_generation_change_stop_and_protection_invalidate_position_observation(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")
        self.assertNotEqual(self.view(generation=2)["phase"], "observed_reached")
        self.rx("STOP fault mask=0x020")
        self.assertEqual(self.view(stop_locked=True)["kind"], "protection")
        self.rx("Disabled ok")
        self.feedback()
        self.assertEqual(self.view()["phase"], "accepted")

    def test_device_stop_confirmation_cannot_later_revive_observed_target(self):
        self.accepted_movej()
        self.feedback()
        self.rx("STOP locked")
        self.assertEqual(self.view(stop_locked=True, stop_confirmed=True)["phase"], "locked")
        self.feedback()
        self.assertEqual(self.view(stop_locked=False)["phase"], "accepted")

    def test_other_modes_and_motion_types_keep_device_receipt_semantics(self):
        for command, kind, mode in ((MOVEJ, "MOVEJ", "SEQ"), (MOVEJ, "MOVEJ", "TRJ"),
                                    (MOVEL, "MOVEL", "INT"), ("!HOME", "HOME", "INT"),
                                    ("!RESET", "RESET", "INT")):
            with self.subTest(command=command, mode=mode):
                self.setUp()
                self.numbered(command=command, kind=kind, mode=mode)
                self.rx(f"accepted {kind} id=1")
                self.feedback()
                self.assertEqual(self.view()["phase"], "accepted")

    def test_ampersand_movej_has_the_same_degree_and_rail_target_as_greater_than(self):
        self.accepted_movej(command="&0,10,80,0,90,0,0,35")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_protocol_optional_speed_does_not_remove_seventh_axis_from_snapshot(self):
        self.accepted_movej(command=">0,10,80,0,90,0,0")
        self.feedback()
        self.assertEqual(self.view()["phase"], "observed_reached")

    def test_observation_does_not_replace_gamepad_manual_confirmation(self):
        self.accepted_movej()
        self.feedback()
        view = self.view(gamepad={"claimed": True, "stage": "init_confirmation", "message": "观察实物"})
        self.assertEqual(view["phase"], "manual_confirmation")
        self.assertEqual(self.t.latest["phase"], "accepted")

    def test_exact_type_id_and_valid_duration_required(self):
        self.numbered(command=MOVEL, number=55, kind="MOVEL")
        for line in ("ok MOVEJ id=55", "ok MOVEL id=54", "ok MOVEL id=55 ms=4294967296",
                     "ok MOVEL id=55 ms=-1", "ok MOVEL id=55 trailing"):
            self.rx(line)
            self.assertNotEqual(self.view()["phase"], "completed", line)
        self.rx("ok MOVEL id=55 ms=1100")
        self.assertEqual(self.view()["phase"], "completed")

    def test_old_generation_queued_tombstone_cannot_complete_new_target(self):
        self.enqueue()
        self.admit()
        self.context["generation"] = 2
        self.enqueue(generation=2)
        self.admit(generation=2)
        self.rx("queued MOVEJ id=5 mode=INT", generation=2)
        self.rx("ok MOVEJ id=5", generation=2)
        self.assertIsNone(self.view()["motion_id"])
        self.rx("queued MOVEJ id=6 mode=INT", generation=2)
        self.rx("ok MOVEJ id=6", generation=2)
        self.assertEqual(self.view()["phase"], "completed")
        self.assertEqual(self.view()["motion_id"], 6)

    def test_old_type_and_duplicate_number_do_not_replace_latest(self):
        self.numbered(number=40)
        self.enqueue(MOVEL)
        self.admit(MOVEL)
        self.rx("queued MOVEJ id=40 mode=INT")
        self.rx("ok MOVEJ id=40")
        self.assertNotEqual(self.view()["phase"], "completed")
        self.rx("queued MOVEL id=41 mode=INT")
        self.rx("ok MOVEL id=41")
        self.rx("accepted MOVEL id=41")
        self.assertEqual(self.view()["phase"], "completed")

    def test_sequence_wrap_uses_uint32_serial_order(self):
        self.numbered(number=0xFFFFFFFF)
        self.enqueue(MOVEL)
        self.admit(MOVEL)
        self.rx("queued MOVEL id=1 mode=SEQ")
        self.rx("ok MOVEL id=1")
        self.assertEqual(self.view()["phase"], "completed")

    def test_reconnect_drops_all_old_identity_and_protection(self):
        self.numbered()
        self.rx("PROTECT J5 failed reason=TIMEOUT")
        self.t.reset(2)
        self.t.requested(MOVEJ, 3, 2)
        self.t.admitted(MOVEJ, 3, 2)
        self.rx("ok MOVEJ id=1", generation=3, connection=0)
        self.assertEqual(self.view(generation=3)["phase"], "sending")
        self.assertIsNone(self.t.protection)

    def test_missing_receipt_is_unknown_not_done_or_stopped(self):
        self.enqueue("!STOP")
        self.admit("!STOP")
        self.now += 11
        view = self.view(stop_locked=True, stop_confirmed=False)
        self.assertEqual(view["phase"], "uncertain")
        self.assertIn("不能据此判断", view["reason"])
        self.assertNotEqual(view["phase"], "completed")

    def test_start_pending_can_follow_existing_generation_invalidation(self):
        self.enqueue("!START")
        self.admit("!START")
        self.rx("Start pending")
        self.t.carry_start_generation(1, 2)
        self.context["generation"] = 2
        self.assertEqual(self.view(stop_locked=True)["kind"], "start")
        self.rx("Started ok", generation=2)
        self.assertEqual(self.view()["title"], "已使能 · 待命")

    def test_stop_request_not_relabelled_by_old_start_receipt(self):
        self.enqueue("!START")
        self.admit("!START")
        self.enqueue("!STOP", generation=2)
        self.admit("!STOP", generation=2)
        self.rx("Start pending", generation=2)
        self.t.carry_start_generation(1, 2)
        self.assertEqual(self.view(generation=2)["kind"], "stop")

    def test_first_explicit_protection_reason_survives_lock_receipts(self):
        self.rx("PROTECT J5 failed reason=PROTECTION_STATUS_TIMEOUT")
        self.rx("STOP fault mask=0x020")
        self.rx("error STOP_LOCKED")
        self.rx("STOP locked")
        view = self.view(stop_locked=True, stop_confirmed=True)
        self.assertEqual(view["kind"], "protection")
        self.assertIn("PROTECTION_STATUS_TIMEOUT", view["reason"])
        self.rx("Disabled ok")
        self.assertIsNone(self.t.protection)

    def test_idle_stall_snapshot_does_not_assert_protection(self):
        for line in ("STALL J5 phase=IDLE", "STALL J5 phase=0", "STALL J8 EXCLUDED"):
            self.rx(line)
        self.assertIsNone(self.t.protection)
        self.assertEqual(self.view()["title"], "已使能 · 待命")

    def test_stop_fault_takes_priority_over_waiting_user_stop(self):
        self.enqueue("!STOP")
        self.admit("!STOP")
        self.rx("STOP fault mask=0x020")
        self.assertEqual(self.view(stop_locked=True)["kind"], "protection")

    def test_new_protection_takes_priority_over_a_recovery_start(self):
        self.rx("PROTECT J5 failed reason=TIMEOUT")
        self.enqueue("!START")
        self.admit("!START")
        self.assertEqual(self.view(stop_locked=True)["kind"], "start")
        self.rx("STOP fault mask=0x020")
        self.assertEqual(self.view(stop_locked=True)["kind"], "protection")
        self.assertIn("TIMEOUT", self.view(stop_locked=True)["reason"])

    def test_stop_origin_is_channel_not_an_invented_manual_button(self):
        self.enqueue("!STOP", source="gamepad")
        self.admit("!STOP")
        state = self.view(gamepad={"last_stop": {"reason": "输入过期"}})
        self.assertIn("手柄侧", state["reason"])
        self.assertIn("输入过期", state["reason"])

    def test_numbered_sequence_queue_does_not_time_out_as_missing_receipt(self):
        self.numbered(mode="SEQ")
        self.now += 60
        self.assertEqual(self.view()["phase"], "queued")

    def test_motion_status_phase_enum_and_mode_must_match(self):
        self.numbered()
        self.rx("MOTION id=1 kind=MOVEJ phase=1 mode=2 ms=20")
        self.assertEqual(self.view()["phase"], "waiting_confirmation")
        self.rx("MOTION id=1 kind=MOVEJ phase=2 mode=2 ms=40")
        self.assertEqual(self.view()["phase"], "executing")
        self.rx("MOTION id=1 kind=MOVEJ phase=3 mode=1 ms=60")
        self.assertEqual(self.view()["phase"], "executing")
        self.rx("MOTION id=1 kind=MOVEJ phase=3 mode=2 ms=60")
        self.assertEqual(self.view()["phase"], "completed")

    def test_late_progress_and_failure_cannot_regress_terminal_motion(self):
        for terminal in ("ok MOVEJ id=1", "error MOVEJ id=1 reason=PLAN_FAILED", "error MOVEJ id=1 reason=CANCELLED"):
            with self.subTest(terminal=terminal):
                self.setUp()
                self.numbered()
                self.rx(terminal)
                phase = self.view()["phase"]
                for stale in ("MOTION id=1 kind=MOVEJ phase=1 mode=2 ms=20", "MOTION id=1 kind=MOVEJ phase=2 mode=2 ms=40",
                              "accepted MOVEJ id=1", "MOTION fail id=1 reason=OLD"):
                    self.rx(stale)
                    self.assertEqual(self.view()["phase"], phase)

    def test_late_accepted_and_dispatch_status_cannot_regress_running(self):
        self.numbered()
        self.rx("MOTION id=1 kind=MOVEJ phase=2 mode=2 ms=40")
        self.rx("accepted MOVEJ id=1")
        self.rx("MOTION id=1 kind=MOVEJ phase=1 mode=2 ms=20")
        self.assertEqual(self.view()["phase"], "executing")

    def test_unrelated_query_errors_and_rgb_do_not_finish_motion(self):
        self.numbered()
        for line in ("error RGB bad", "error GET_PID failed", "RGB_RUNTIME state=READY_INT source=AUTO",
                     "ok hand pos=100", "ok 0 10 80 0 90 0 0"):
            self.rx(line)
        self.assertEqual(self.view()["phase"], "queued")

    def test_gamepad_manual_confirmation_uses_session_not_pose_comparison(self):
        self.numbered()
        self.rx("accepted MOVEJ id=1")
        view = self.view(gamepad={"claimed": True, "stage": "init_confirmation", "message": "已受理，观察实物"})
        self.assertEqual(view["phase"], "manual_confirmation")
        self.assertIn("确认到位", view["next_step"])

    def test_rgb_uncertain_is_fenced_and_never_claims_saved(self):
        self.now = 30.
        view = self.view(rgb_save={"active": True, "stage": "uncertain", "message": "6秒未确认", "started_at": 20.})
        self.assertEqual(view["phase"], "uncertain")
        self.assertEqual(view["elapsed_s"], 10.)
        self.assertIn("不会自动重发", view["next_step"])

    def test_unassociated_unnumbered_error_cannot_assign_future_number(self):
        self.enqueue()
        self.admit()
        self.rx("error DISABLED")
        self.rx("queued MOVEJ id=1 mode=INT")
        self.rx("ok MOVEJ id=1")
        self.assertEqual(self.view()["phase"], "uncertain")
        self.assertIsNone(self.view()["motion_id"])

    def test_overflow_disables_kind_binding_instead_of_reattributing_old_reply(self):
        for _ in range(self.t.LIMIT + 1):
            self.enqueue()
            self.admit()
        self.rx("queued MOVEJ id=1 mode=INT")
        self.rx("ok MOVEJ id=1")
        self.assertIsNone(self.view()["motion_id"])
        self.assertIn("MOVEJ", self.t.ambiguous)


if __name__ == "__main__":
    unittest.main()
