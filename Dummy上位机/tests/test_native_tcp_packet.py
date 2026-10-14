"""World TCP v2 packet/real serial-writer contract; no hardware or PC IK."""
import time
import unittest
from unittest.mock import patch
import numpy as np
from tests.test_ui import APPLICATION
from tests.test_native_execution_contract import NativeMemoryPort
from app.core.controller import RobotController
from app.core.cartesian_backend import tcp_command
from dummy_motion import Tool,IKSolver,StreamPlanner
from dummy_motion.frames import from_xyzabc,to_xyzabc

CAP='KIN v=2 xyz=mm abc=deg joints=deg rail=mm'
class TCPPort(NativeMemoryPort):
    def __init__(self,advertise=False):super().__init__();self.advertise=advertise
    def write(self,data):
        count=super().write(data)
        if data.decode('ascii').strip()=='#KIN_CONFIG':
            self.inject(((CAP if self.advertise else 'error UNKNOWN_COMMAND')+'\n').encode('ascii'))
        return count

class NativeTCPPacketTests(unittest.TestCase):
    def setUp(self):
        self.port=TCPPort();self.c=RobotController(serial_factory=lambda *a,**k:self.port)
        self.assertTrue(self.c.connect_port('MEMORY_TCP','USB'))
        self.wait(lambda:self.c.current_cmdmode==2 and self.c._mode_request is None)
        self.c.set_polling(False);self.c._motor_state_known=self.c._motor_enabled=True
        self.c.cartesian.update_feedback([0,10,80,0,90,0,37.25],time.perf_counter())
    def tearDown(self):self.c.close()
    def wait(self,predicate,timeout=1.5):
        deadline=time.perf_counter()+timeout
        while time.perf_counter()<deadline:
            APPLICATION.processEvents()
            if predicate():return
            time.sleep(.002)
        self.fail('real native-TCP writer timed out')
    def settle(self,seconds=.08):
        deadline=time.perf_counter()+seconds
        while time.perf_counter()<deadline:APPLICATION.processEvents();time.sleep(.002)
    def upgrade(self):
        self.port.advertise=True;self.assertTrue(self.c.send_cmd('#KIN_CONFIG'))
        self.wait(lambda:self.c.cartesian.native_tcp)
    def motion(self):return [s for s in self.port.commands()if s.startswith(('@','>','&','~S','!STREAM_'))]
    def record(self):return dict(name='TCP',type='MoveL',pose=[220,70,210,23,-30,48],speed=31.5,gripper=100,frame='world',target_kind='tcp',tool=[10,-20,114,8,5,-7],tool_version=1)
    def test_connection_queries_capability_once_and_old_error_stays_legacy(self):
        self.wait(lambda:'#KIN_CONFIG' in self.port.commands())
        self.assertEqual(self.port.commands().count('#KIN_CONFIG'),1)
        self.assertFalse(self.c.cartesian.native_tcp)
        self.assertTrue(self.c.send_typed_position(self.record()))
        self.wait(lambda:self.motion());self.assertFalse(self.motion()[0].startswith('@T'))
        self.assertEqual(self.port.commands().count('#KIN_CONFIG'),1)
    def test_new_firmware_connect_auto_discovers_v2_without_manual_query(self):
        self.c.disconnect_port();self.port=TCPPort(advertise=True)
        self.c._serial_factory=lambda *a,**k:self.port
        self.assertTrue(self.c.connect_port('NEW_TCP_CORE','USB'))
        self.wait(lambda:self.c.current_cmdmode==2 and self.c.cartesian.native_tcp)
        self.c.set_polling(False);self.c._motor_state_known=self.c._motor_enabled=True
        self.c.cartesian.update_feedback([0,10,80,0,90,0,37.25],time.perf_counter())
        self.assertEqual(self.port.commands().count('#KIN_CONFIG'),1)
        self.assertTrue(self.c.send_typed_position(self.record()))
        self.wait(lambda:self.motion());self.assertEqual(len(self.motion()),1)
        self.assertTrue(self.motion()[0].startswith('@T'))
    def test_unknown_capability_uses_one_legacy_flange_packet(self):
        self.assertFalse(self.c.cartesian.native_tcp);record=self.record()
        self.assertTrue(self.c.send_typed_position(record));self.wait(lambda:len(self.motion())==1)
        command=self.motion()[0];self.assertTrue(command.startswith('@')and not command.startswith('@T'))
        expected=self.c.cartesian.model.native_target(from_xyzabc(record['pose']),.03725,Tool(from_xyzabc(record['tool'])))
        np.testing.assert_allclose(list(map(float,command[1:].split(',')))[:6],to_xyzabc(expected),atol=.051,rtol=0)
    def test_capability_switch_sends_one_world_tcp_with_tool_snapshot(self):
        self.upgrade();record=self.record();original_tool=list(record['tool'])
        with patch.object(IKSolver,'solve_pose',side_effect=AssertionError('live PC IK')),patch.object(StreamPlanner,'plan_line',side_effect=AssertionError('PC line')),patch.object(StreamPlanner,'plan_joint',side_effect=AssertionError('PC joints')):
            self.assertTrue(self.c.send_typed_position(record))
            record['tool'][:]=[0]*6;self.c.gamepad.robot['tool_offset_z']=222
            self.wait(lambda:len(self.motion())==1);self.settle()
        command=self.motion()[0];self.assertTrue(command.startswith('@T'));self.assertLessEqual(len(command),159)
        values=list(map(float,command[2:].split(',')));self.assertEqual(len(values),14)
        np.testing.assert_allclose(values[:6],[220,70,210,23,-30,48],atol=.0001,rtol=0)
        self.assertEqual(values[6:8],[31.5,1]);np.testing.assert_allclose(values[8:],original_tool,atol=.0001,rtol=0)
        self.assertEqual(len(self.motion()),1)
    def test_base_flange_point_stays_legacy_even_after_v2(self):
        self.upgrade();self.assertTrue(self.c.send_typed_position(dict(name='old',type='MoveL',pose=[220,20,180,180,0,180],speed=35,gripper=100)))
        self.wait(lambda:self.motion());self.assertEqual(self.motion(),['@220,20,180,180,0,180,35'])
    def test_exact_capability_contract_not_auxiliary_or_different_units(self):
        for line in ('KIN v=1 xyz=mm abc=deg joints=deg rail=mm','KIN v=3 xyz=mm abc=deg joints=deg rail=mm','KIN v=2 xyz=m abc=rad joints=rad rail=m','KIN base=207.3 db=0 arm=170 forearm=117','ok KIN v=2 xyz=mm abc=deg joints=deg rail=mm'):
            self.c._process_rx_line(line);self.assertFalse(self.c.cartesian.native_tcp,line)
        self.c._process_rx_line(CAP);self.assertTrue(self.c.cartesian.native_tcp)
    def test_reconnect_discards_capability_and_late_old_connection(self):
        self.upgrade();old_connection=self.c._connection_id;self.c.disconnect_port();self.assertFalse(self.c.cartesian.native_tcp)
        port=TCPPort();self.c._serial_factory=lambda *a,**k:port;self.assertTrue(self.c.connect_port('OLD_CORE','USB'))
        self.wait(lambda:self.c.current_cmdmode==2 and self.c._mode_request is None)
        self.c.set_polling(False);self.assertFalse(self.c.cartesian.native_tcp)
        self.c._on_transport_line(CAP,old_connection,time.perf_counter());self.assertFalse(self.c.cartesian.native_tcp)
    def test_raw_8_and_14_fields_and_auto_flag_zero_one(self):
        for command in ('@T220,20,180,10,20,30,35,0','@T220,20,180,10,20,30,35,1,10,-20,114,8,5,-7'):
            self.assertTrue(self.c.send_cmd(command));self.wait(lambda:command in self.port.commands())
        self.assertEqual(len(self.motion()),2)
    def test_invalid_raw_tcp_fields_never_reach_dispatch(self):
        base='220,20,180,10,20,30,35,1';bad=[]
        for token in ('nan','inf','-inf','1e999','1e100','bad','1junk'):
            bad.append('@T'+token+',20,180,10,20,30,35,1')
        bad+=['@T'+base+'\x00junk','@T'+base+' trailing','@T'+base+',','@T'+base+',0','@T'+base+',0,0,0,0,0','@T'+base+',0,0,0,0,0,0,0','@T220,,20,180,10,20,30,35,1','@T220 20,180,10,20,30,35,1']
        for speed,flag in ((0,1),(101,1),(35,-1),(35,2),(35,.5)):
            bad.append(f'@T220,20,180,10,20,30,{speed},{flag}')
        bad.append('@T'+base+',501,0,114,0,0,0')
        before=self.motion()
        for command in bad:
            with self.subTest(command=command):self.assertFalse(self.c.send_cmd(command))
        self.settle();self.assertEqual(self.motion(),before)
    def test_total_packet_159_byte_limit(self):
        base='0,0,0,0,0,0,10,1';packet='@T'+'0'*(159-2-len(base))+base
        self.assertEqual(len(packet),159);self.assertTrue(self.c.send_cmd(packet));self.wait(lambda:packet in self.port.commands())
        self.assertFalse(self.c.send_cmd('@T0'+packet[2:]));self.settle();self.assertEqual(self.motion(),[packet])
    def test_tcp_helper_keeps_degrees_and_full_tool_pose(self):
        target=from_xyzabc([220,70,210,23,-30,48]);tool=Tool(from_xyzabc([10,-20,114,8,5,-7]))
        values=list(map(float,tcp_command(target,tool,31.5,rail_auto=False)[2:].split(',')))
        self.assertEqual(values[7],0);np.testing.assert_allclose(values[:6],[220,70,210,23,-30,48],atol=.0001,rtol=0)
        np.testing.assert_allclose(values[8:],[10,-20,114,8,5,-7],atol=.0001,rtol=0)
        for flag in (2,-1,.5,'false'):
            with self.subTest(flag=flag),self.assertRaises(ValueError):tcp_command(target,tool,35,rail_auto=flag)
    def test_mode7_and_stop_block_tcp_at_final_writer(self):
        command='@T220,20,180,10,20,30,35,1';self.c.current_cmdmode=7;before=self.motion()
        self.assertFalse(self.c.send_cmd(command));self.c._tx_queue.put((self.c._tx_generation,command));self.settle();self.assertEqual(self.motion(),before)
        self.c.current_cmdmode=2;old=self.c._tx_generation;self.assertTrue(self.c.send_cmd('!STOP'));self.wait(lambda:self.c._stop_confirmed)
        self.c._tx_queue.put((old,command));self.c._tx_queue.put((self.c._tx_generation,command));self.settle()
        self.assertFalse(self.c.send_cmd(command));self.assertEqual(self.motion(),before)

if __name__=='__main__':unittest.main()
