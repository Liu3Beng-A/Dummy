"""Selection and admission states for the dedicated curve communication entry."""
import unittest
from app.ui.terminal_diagnostics import TerminalDiagnostics
from tests.test_ui import APPLICATION


class AxisCommunicationUiTests(unittest.TestCase):
    def setUp(self):
        self.ui = TerminalDiagnostics()
        self.requests = []
        self.ui.axis_test_requested.connect(self.requests.append)

    def tearDown(self):
        self.ui.close(); self.ui.deleteLater(); APPLICATION.processEvents()

    def state(self, **changes):
        values = dict(connected=True, locked=False, calibrating=False, connection_type='USB', disabled=False, busy=False)
        values.update(changes); self.ui.set_state(**values)

    def test_axis_choice_is_passive_and_click_emits_exact_selected_mask(self):
        self.state()
        self.assertEqual(self.ui.axis_test_selector.count(), 9)
        masks = [1, 2, 4, 8, 16, 32, 64, 65, 127]
        for index, mask in enumerate(masks):
            self.ui.axis_test_selector.setCurrentIndex(index)
            self.assertEqual(self.requests, masks[:index])
            self.assertEqual(self.ui.axis_test_selector.currentData(), mask)
            self.ui.axis_test_button.click()
        self.assertEqual(self.requests, masks)

    def test_entry_rejects_connection_lock_calibration_busy_and_disabled(self):
        for changes in (dict(connected=False), dict(connection_type='UART4'), dict(locked=True),
                        dict(calibrating=True), dict(busy=True), dict(disabled=True)):
            with self.subTest(**changes):
                self.state(**changes)
                self.assertFalse(self.ui.axis_test_button.isEnabled())
                self.assertFalse(self.ui.axis_test_selector.isEnabled())
                self.ui.axis_test_button.click()
                self.assertEqual(self.requests, [])

    def test_curve_axes_selection_is_independent_of_readonly_diagnostic_node(self):
        self.state(); self.ui.node.setCurrentIndex(7)
        self.assertFalse(self.ui.open_test_button.isEnabled())
        self.assertTrue(self.ui.axis_test_button.isEnabled())
        self.ui.axis_test_selector.setCurrentIndex(6); self.ui.axis_test_button.click()
        self.assertEqual(self.requests, [64])

    def test_tooltip_states_actual_behavior_and_result_is_readonly(self):
        text = self.ui.axis_test_button.toolTip()
        self.assertIn('保持电机已有目标', text)
        self.assertIn('不自动使能', text)
        self.assertIn('全机软停', text)
        self.assertTrue(self.ui.open_test_output.isReadOnly())


if __name__ == '__main__':
    unittest.main()
