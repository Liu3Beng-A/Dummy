"""Actual firmware mode remains authoritative at dispatch and writer admission."""
import unittest
from unittest.mock import patch
from app.core.cartesian_backend import CartesianBackend
from tests import test_controller as controller_tests


class StreamModeGateTests(unittest.TestCase):
    setUpClass=controller_tests.ControllerTests.setUpClass
    setUp=controller_tests.ControllerTests.setUp
    tearDown=controller_tests.ControllerTests.tearDown
    commands=controller_tests.ControllerTests.commands
    motions=('@146.5,0,288.9,180,0,180,35','>0,10,80,0,90,0,0,35','&0,10,80,0,90,0,0,35')

    def confirm_mode(self, mode):
        self.controller._mode_request=123
        self.controller._handle_motion_receipt(f'ok CMDMODE mode={mode} busy=0 issued=0 req=123')

    def test_actual_mode7_rejects_plain_motion_at_dispatch_for_every_source(self):
        c=self.controller;self.confirm_mode(7)
        c.gamepad.claimed=True
        for source in ('ui','gamepad','cartesian'):
            for command in self.motions:
                with self.subTest(source=source,command=command):
                    self.assertFalse(c.send_cmd(command,source=source))
        self.assertEqual(self.commands(),[])
        self.assertTrue(any('主控实际处于模式 7' in message for message,_ in self.logs))

    def test_mode7_receipt_discards_native_motion_already_queued(self):
        c=self.controller;self.confirm_mode(2)
        for command in self.motions:self.assertTrue(c.send_cmd(command))
        self.confirm_mode(7)
        for command in self.motions:
            generation,queued=c._tx_queue.get_nowait()
            self.assertEqual(queued,command)
            self.assertFalse(c._can_write(generation,queued))
        self.assertTrue(c._wire_admitted.empty())
        self.assertEqual(c.serial_port.written,[])

    def test_removed_stream_selection_does_not_bypass_actual_mode7(self):
        c=self.controller;self.confirm_mode(7)
        self.assertFalse(c.set_cartesian_backend('stream'))
        self.assertEqual(c.cartesian.backend,CartesianBackend.FIRMWARE)
        for command in self.motions:self.assertFalse(c.send_cmd(command))
        self.assertEqual(self.commands(),[])

    def test_native_mode2_still_admits_plain_motion(self):
        c=self.controller;self.confirm_mode(2)
        for command in self.motions:
            self.assertTrue(c.send_cmd(command))
            generation,queued=c._tx_queue.get_nowait()
            self.assertTrue(c._can_write(generation,queued))
