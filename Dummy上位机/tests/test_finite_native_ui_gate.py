"""New finite Cartesian capability cannot enter legacy continuous controls."""
import queue
import unittest
from unittest.mock import patch
from tests.test_ui import APPLICATION
from tests.test_gamepad_session import GamepadFixture
from tests.test_cartesian_ui import CartesianUiTests

CAP='KIN v=2 xyz=mm abc=deg joints=deg rail=mm'

class FiniteNativeGamepadGateTests(GamepadFixture,unittest.TestCase):
    def setUp(self):self.setup_gamepad()
    def tearDown(self):self.c.close()
    def test_new_capability_rejects_claim_without_motion_or_owner_change(self):
        self.feed(CAP);generation=self.c._tx_generation
        self.assertFalse(self.g.claim());self.assertFalse(self.g.claimed)
        self.assertEqual(self.c._control_owner,'ui');self.assertEqual(self.c._tx_generation,generation)
        self.assertEqual(self.take(),[]);self.assertFalse(self.g.ready)
        self.assertIn('连续操控暂不可用',self.g.message)
    def test_ready_and_stream_gate_recheck_capability(self):
        self.ready_session();self.g.set_target(self.target(1));self.g.tick()
        item=next(item for item in self.take()if item[1].startswith('@'))
        self.assertTrue(self.g.can_write_stream(*item))
        # Directly set the capability to isolate local property/writer checks
        # from the separate late-response release contract below.
        self.c.cartesian.native_tcp=True
        self.assertFalse(self.g.ready);self.assertFalse(self.g.can_write_stream(*item))
        self.assertFalse(self.c._can_write(*item))
        self.assertTrue(self.c.send_cmd('!STOP',source='gamepad'))
        stop=next(item for item in self.take()if item[1]=='!STOP')
        self.assertTrue(self.c._can_write(*stop))
    def test_late_capability_releases_old_claim_and_preserves_stop(self):
        self.ready_session();self.g.set_target(self.target(1));self.g.tick()
        old=next(item for item in self.take()if item[1].startswith('@'))
        self.feed(CAP)
        self.assertTrue(self.c.cartesian.native_tcp)
        self.assertTrue(self.g.release_pending or not self.g.claimed)
        self.assertFalse(self.c._can_write(*old))
        pending=self.take();self.assertIn('!STOP',[command for _,command in pending])
        for item in pending:
            if item[1]=='!STOP':self.write([item])
        self.feed('Stopping ok','STOP locked')
        self.assertFalse(self.g.claimed);self.assertEqual(self.c._control_owner,'ui')
    def test_legacy_capability_keeps_ready_continuous_path(self):
        self.ready_session();self.assertTrue(self.g.ready)
        self.g.set_target(self.target(1));self.g.tick()
        item=next(item for item in self.take()if item[1].startswith('@'))
        self.assertTrue(self.g.can_write_stream(*item));self.assertTrue(self.c._can_write(*item))

class FiniteNativeUIGateTests(unittest.TestCase):
    setUp=CartesianUiTests.setUp
    tearDown=CartesianUiTests.tearDown
    def commands(self):
        result=[]
        while not self.c._tx_queue.empty():
            item=self.c._tx_queue.get_nowait()
            if item:result.append(item[1])
        return result
    def capability(self,enabled=True):
        self.c.cartesian.native_tcp=enabled
        self.w._update_state(self.c.state())
    def test_move_l_drag_is_blocked_and_preview_still_changes(self):
        self.c.current_cmdmode=2;self.capability()
        self.w.motion_tabs.setCurrentIndex(1)
        self.assertFalse(self.w.l_drag.isEnabled())
        self.assertFalse(self.w.gamepad_panel.claim_button.isEnabled())
        self.w.pose_spins[0].setValue(223.3);self.w._target_edited('l')
        self.assertAlmostEqual(self.w.viewer.cartesian[0],223.3)
        self.assertNotIn('l',self.w._drag_pending);self.assertEqual(self.commands(),[])
        self.assertFalse(self.w._drag_enabled('l'))
    def test_joint_hand_and_joint_teaching_drag_remain_available(self):
        self.c.current_cmdmode=2;self.capability()
        self.c.movej_drag_enable.set(True);self.c.hand_pos_drag_enable.set(True)
        self.w.teaching_tabs.setCurrentIndex(0);self.w.teaching_drag.setChecked(True)
        self.assertTrue(self.w._drag_enabled('j'));self.assertTrue(self.w._drag_enabled('hand'))
        self.assertTrue(self.w._drag_enabled('teaching'));self.assertTrue(self.w.teaching_drag.isEnabled())
        self.w.teaching_tabs.setCurrentIndex(1)
        self.assertFalse(self.w._drag_enabled('teaching'));self.assertFalse(self.w.teaching_drag.isEnabled())
        self.w.teaching_pose_spins[0].setValue(220)
        self.assertNotIn('teaching',self.w._drag_pending)
        self.w.teaching_tabs.setCurrentIndex(0)
        self.assertTrue(self.w.teaching_drag.isEnabled())
    def test_old_firmware_restores_controls_without_automatic_drag(self):
        self.c.current_cmdmode=2;self.capability();self.capability(False)
        self.assertTrue(self.w.l_drag.isEnabled());self.assertTrue(self.w.gamepad_panel.claim_button.isEnabled())
        self.assertFalse(self.c.movel_drag_enable.get());self.assertEqual(self.commands(),[])
    def test_late_capability_cancels_only_cartesian_pending_drag(self):
        self.c.current_cmdmode=2;self.c.movel_drag_enable.set(True)
        self.w._queue_drag('l');self.assertIn('l',self.w._drag_pending)
        self.capability();self.assertNotIn('l',self.w._drag_pending)
        self.w._flush_drag('l');self.assertEqual(self.commands(),[])
        self.assertFalse(self.w._drag_timers['l'].isActive())
    def test_reader_status_cannot_reenable_new_firmware_claim(self):
        self.capability();self.w._gamepad_reader_status({'running':False,'name':'offline'})
        self.assertFalse(self.w.gamepad_panel.claim_button.isEnabled())
        self.assertTrue(self.w.gamepad_panel.check_button.isEnabled())
        self.assertFalse(self.w._gamepad_action('claim'));self.assertEqual(self.commands(),[])
    def test_finite_send_preserved_but_direct_continuous_rejected(self):
        from tests.test_controller import FakePort
        self.c.is_connected=True;self.c.serial_port=FakePort();self.c.current_cmdmode=2
        self.capability();point=dict(name='finite',type='MoveL',pose=[220,0,180,180,0,180],speed=35,gripper=100)
        self.assertFalse(self.c.send_typed_position(point,continuous=True));self.assertEqual(self.commands(),[])
        self.assertTrue(self.c.send_typed_position(point));self.assertEqual(self.commands(),['@220,0,180,180,0,180,35'])
        joint=dict(name='joint',type='MoveJ',joints=[0,10,80,0,90,0,0],speed=35,gripper=100)
        self.assertTrue(self.c.send_typed_position(joint,continuous=True));self.assertEqual(self.commands(),['>0,10,80,0,90,0,0,35'])

if __name__=='__main__':unittest.main()
