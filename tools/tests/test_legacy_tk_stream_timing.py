"""Real hidden Tk callbacks, fake serial; no hardware connection or motion."""
import unittest
from test_legacy_tk_finite_gate import LegacyFiniteGateTests, module


class LegacyStreamTimingTests(LegacyFiniteGateTests):
    def test_selected_parameter_node_and_field_only_issue_one_read(self):
        self.connect(); self.take()
        self.a._stop_locked = True
        self.a.param_node.set(5)
        self.a.stream_timing_field.current(13)  # RUNNING peak, field58.
        self.a.btn_stream_timing.invoke()
        self.assertEqual(self.commands(), ['#STREAM_DIAG 5 58'])
        for node in (0, 8):
            self.a.param_node.set(node)
            self.a.btn_stream_timing.invoke()
            self.assertEqual(self.commands(), [])

    def test_stream_query_remains_read_only_when_locked(self):
        for field in range(64):
            self.assertTrue(self.a._is_protection_status_query(f'#STREAM_DIAG 3 {field}'))
        for command in ('#STREAM_DIAG 3 64', '#STREAM_DIAG 0 51',
                        '#STREAM_DIAG 8 51', '!STREAM_START 1', '~S0011'):
            self.assertFalse(self.a._is_protection_status_query(command))

    def test_units_scope_and_malformed_replies(self):
        self.assertIn('57.583 us', module.format_stream_timing('STREAM_NODE node=3 n=1 field=9 value=4146'))
        self.assertIn('94.943 ms', module.format_stream_timing('STREAM_NODE node=3 n=1 field=2 value=6835912'))
        self.assertIn('抽样', module.format_stream_timing('STREAM_NODE node=5 n=2 field=52 value=717'))
        self.assertIn('未知签名', module.format_stream_timing('STREAM_NODE node=3 n=3 field=51 value=0'))
        for line in ('STREAM_NODE node=8 n=1 field=9 value=4146',
                     'STREAM_NODE node=3 n=65536 field=9 value=4146',
                     'STREAM_NODE node=3 n=1 field=9 value=4294967296',
                     'STREAM_NODE node=3 n=1 field=9 value=-1',
                     'STREAM_NODE node=3 n=1 field=9 value=4146 extra'):
            self.assertIsNone(module.format_stream_timing(line))


if __name__ == '__main__':
    unittest.main()
