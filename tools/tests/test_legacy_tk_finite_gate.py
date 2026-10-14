"""Hidden real Tk widgets and fake serial only; no device access or motion."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2] if Path(__file__).resolve().parents[2].name=='Dummy-code' else Path(r'E:\Dummy-code')
spec=importlib.util.spec_from_file_location('legacy_serial_under_test',ROOT/'串口助手_old.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
CAP='KIN v=2 xyz=mm abc=deg joints=deg rail=mm'

class FakePort:
    def __init__(self):
        self.is_open=True;self.write_timeout=1;self.writes=[];self.reply=None
        self.rx=bytearray();self.lock=threading.Lock()
    @property
    def in_waiting(self):
        with self.lock:return len(self.rx)
    def read(self,count):
        with self.lock:
            data=bytes(self.rx[:count]);del self.rx[:count];return data
    def write(self,data):
        command=data.decode().strip()
        with self.lock:
            self.writes.append(command)
            if command=='#KIN_CONFIG' and self.reply:
                self.rx.extend((self.reply+'\n').encode())
        return len(data)
    def close(self):self.is_open=False

class LegacyFiniteGateTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk();self.root.withdraw();self.port=FakePort()
        self.patches=[patch.object(module.serial.tools.list_ports,'comports',return_value=[]),
                      patch.object(module.serial,'Serial',side_effect=lambda *args,**kw:self.port),
                      patch.object(module.messagebox,'showwarning'),patch.object(module.messagebox,'showerror')]
        for p in self.patches:p.start()
        self.a=module.RobotSerialAssistantOld(self.root)
        # Queued cases still use real Tk, real callbacks and real serial writer,
        # but start the writer explicitly so ordering is deterministic.
        self.a.serial_receive_loop=lambda *args:None
        self.a._start_tx_thread=lambda:None
    def tearDown(self):
        if self.a.is_connected:self.a.toggle_connection()
        self.pump(.015);self.root.destroy()
        for p in reversed(self.patches):p.stop()
    def pump(self,seconds=.015):
        self.root.after(max(1,int(seconds*1000)),self.root.quit);self.root.mainloop()
    def until(self,predicate,timeout=1):
        deadline=time.monotonic()+timeout
        def check():
            if predicate() or time.monotonic()>=deadline:self.root.quit()
            else:self.root.after(5,check)
        self.root.after(5,check);self.root.mainloop();self.assertTrue(predicate())
    def connect(self):
        self.a.toggle_connection();self.assertTrue(self.a.is_connected)
    def take(self):
        items=[]
        while True:
            try:
                item=self.a._tx_queue.get_nowait()
                if item is not None:items.append(item)
            except queue.Empty:return items
    def commands(self):return [i[1] for i in self.take()]
    def writer(self):
        self.a._tx_queue.put(None);self.a._serial_write_loop()
    def cap(self,line=CAP,connection=None):
        self.a._handle_rx_line(line,self.a._kin_connection if connection is None else connection)
        self.pump()
    def test_connection_queries_once_and_reconnect_clears_capability(self):
        self.connect();self.assertEqual(self.commands(),['#KIN_CONFIG','#RGB_GET'])
        self.cap();self.assertTrue(self.a._native_tcp)
        old=self.a._kin_connection;self.a.toggle_connection();self.assertFalse(self.a._native_tcp)
        self.port=FakePort();self.connect();self.assertEqual(self.commands(),['#KIN_CONFIG','#RGB_GET'])
        self.cap(connection=old);self.assertFalse(self.a._native_tcp)
        stale=FakePort();stale.rx.extend((CAP+'\n').encode())
        module.RobotSerialAssistantOld.serial_receive_loop(self.a,stale,old)
        self.pump();self.assertFalse(self.a._native_tcp)
        self.assertEqual(str(self.a.btn_movel_drag.cget('state')),'normal')
        self.cap();self.assertTrue(self.a._native_tcp)
    def test_only_exact_new_protocol_upgrades_old_firmware_unchanged(self):
        self.connect();self.take();self.a.current_cmdmode=3
        for line in ('ok','error KIN unsupported','KIN v=1 xyz=mm abc=deg joints=deg rail=mm',
                     'KIN v=2 xyz=m abc=deg joints=deg rail=mm',CAP+' extra'):
            self.cap(line);self.assertFalse(self.a._native_tcp)
        self.a.movel_drag_enable.set(True);self.a._schedule_motion_callback(self.a.send_movel)
        self.pump();items=self.take();self.assertEqual(len(items),1);self.assertTrue(items[0][2])
        self.a._tx_queue.put(items[0]);self.writer();self.assertTrue(self.port.writes[0].startswith('@'))
    def test_native_disables_only_movel_drag_and_keeps_preview(self):
        self.connect();self.take();self.a.movej_drag_enable.set(True);self.a.hand_pos_drag_enable.set(True)
        self.cap();self.assertEqual(str(self.a.btn_movel_drag.cget('state')),'disabled')
        self.assertFalse(self.a.movel_drag_enable.get());self.assertTrue(self.a.movej_drag_enable.get())
        self.assertTrue(self.a.hand_pos_drag_enable.get())
        self.a.scl_pose[0].set(180);self.pump()
        self.assertAlmostEqual(float(self.a.ent_pose[0].get()),180)
        self.assertEqual(self.commands(),[])
        self.assertIn('单个笛卡尔目标',self.a._kin_status_var.get())
        self.a.movel_drag_enable.set(True);self.a._warn_drag_mode();self.assertFalse(self.a.movel_drag_enable.get())
    def test_late_cap_cancels_unwritten_callback_without_stop_or_joint_hand_change(self):
        self.connect();self.take();self.a.movel_drag_enable.set(True)
        self.a.movej_drag_enable.set(True);self.a.hand_pos_drag_enable.set(True)
        self.a._schedule_motion_callback(self.a.send_movel)
        self.assertTrue(self.a._movel_drag_callbacks)
        self.a._process_rx_line(CAP,self.a._kin_connection)
        self.assertFalse(self.a._movel_drag_callbacks);self.pump()
        self.assertEqual(self.commands(),[]);self.assertFalse(self.a._stop_locked)
        self.assertTrue(self.a.movej_drag_enable.get());self.assertTrue(self.a.hand_pos_drag_enable.get())
        fired=[];self.a._schedule_motion_callback(lambda:fired.append('joint'))
        self.a._schedule_motion_callback(lambda:fired.append('hand'));self.pump()
        self.assertEqual(fired,['joint','hand'])
    def test_final_writer_discards_drag_tail_but_preserves_manual_same_target_and_movej_hand(self):
        self.connect();self.take();self.a.movel_drag_enable.set(True)
        self.a.send_movel(continuous=True);self.a.send_movel();self.a.send_movej()
        self.a.send_cmd('!HAND_POS 50');self.a._process_rx_line(CAP,self.a._kin_connection)
        self.writer();self.assertEqual(sum(c.startswith('@') for c in self.port.writes),1)
        self.assertTrue(any(c.startswith('>') for c in self.port.writes))
        self.assertIn('!HAND_POS 50',self.port.writes);self.assertNotIn('!STOP',self.port.writes)
        self.assertFalse(self.a.send_movel(continuous=True));self.assertEqual(self.commands(),[])
    def test_late_cap_after_real_drag_write_stops_and_cannot_replay_tail(self):
        self.connect();self.take();self.a.movel_drag_enable.set(True)
        self.a.send_movel(continuous=True);self.writer();self.assertTrue(self.a._movel_drag_written)
        self.a.send_movel(continuous=True)
        self.a._process_rx_line(CAP,self.a._kin_connection)
        items=self.take();self.assertEqual([i[1] for i in items],['!STOP'])
        for item in items:self.a._tx_queue.put(item)
        self.writer();self.assertEqual(sum(c.startswith('@') for c in self.port.writes),1)
        self.assertEqual(self.port.writes[-1],'!STOP');self.assertTrue(self.a._stop_locked)
        self.a._process_rx_line(CAP,self.a._kin_connection);self.assertEqual(self.commands(),[])
    def test_raw_native_tcp_and_readonly_queries_remain_available(self):
        self.connect();self.take();self.cap()
        raw='@T220,0,180,180,0,180,35,1'
        self.assertTrue(self.a.send_cmd(raw));self.assertEqual(self.commands(),[raw])
        self.a.send_movel();single=self.commands();self.assertEqual(len(single),1)
        self.assertTrue(single[0].startswith('@'));self.assertFalse(single[0].startswith('@T'))
        self.a._stop_locked=True
        for command in ('#KIN_CONFIG','#TOOL','#TCPPOSE','#CARTESIAN_STATUS','#IKCHECK 0,10,80,0,90,0'):
            self.assertTrue(self.a.send_cmd(command),command)
        self.assertFalse(self.a.send_cmd('#TOOL_SET 0,0,114,0,0,0'))
        self.assertFalse(self.a.send_cmd(raw));self.assertTrue(self.a.send_cmd('!STOP'))
        self.assertEqual(self.commands(),['!STOP'])
    def test_actual_threads_rx_tk_delivery_and_writer_never_read_tk_off_main(self):
        del self.a.__dict__['serial_receive_loop'];del self.a.__dict__['_start_tx_thread']
        main=threading.get_ident();original=self.a.movel_drag_enable.get
        def checked_get():
            self.assertEqual(threading.get_ident(),main);return original()
        self.a.movel_drag_enable.get=checked_get
        self.port.reply=CAP;self.connect();self.until(lambda:self.a._native_tcp)
        self.assertEqual(self.port.writes.count('#KIN_CONFIG'),1)
        self.assertEqual(str(self.a.btn_movel_drag.cget('state')),'disabled')
        self.a.send_movel();self.until(lambda:any(c.startswith('@') for c in self.port.writes))
        self.assertEqual(sum(c.startswith('@') for c in self.port.writes),1)
        self.a.send_cmd('!STOP');self.until(lambda:'!STOP' in self.port.writes)
        self.assertFalse(self.a._movel_drag_written)

if __name__=='__main__':unittest.main(verbosity=2)

