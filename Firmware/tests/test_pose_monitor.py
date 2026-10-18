"""Persistent display, port handoff, and absence of monitor control commands."""
import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'diagnostics'))
from pose_stream import SerialPoseReader
from serial_monitor_handoff import SerialMonitorHandoff, lease_path, monitor_should_yield


class Link:
    def __init__(self):
        self.is_open = False
        self.buffer = b''
        self.writes = []
        self.failed = False

    @property
    def in_waiting(self):
        return len(self.buffer)

    def open(self):
        if self.failed:
            raise serial.SerialException('Disconnected')
        self.is_open = True

    def close(self):
        self.is_open = False

    def read(self, count):
        if self.failed:
            raise serial.SerialException('Disconnected')
        result, self.buffer = self.buffer[:count], self.buffer[count:]
        return result

    def write(self, data):
        self.writes.append(data)

    def telemetry(self, pitch=23.4, roll=50):
        self.buffer += (f'$TELE,Pit:{pitch},Rol:{roll},Bal:0,EN:000,S1:0,S2:0,S3:0#\n').encode()


class StreamTests(unittest.TestCase):
    def test_dynamic_neutral_target_and_baseline_reset(self):
        from live_pose import PosePublisher
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'live.json'
            pub = PosePublisher(path, [24.7,49.72], 1)
            pub.update({'Pit':'24.7','Rol':'49.72','Bal':'3','NPit':'24.5','NRol':'49.5'})
            self.assertEqual(json.loads(path.read_text())['target'], [24.5,49.5])
            # A caller may continue assigning the original anchor on every frame.
            pub.target = [24.7,49.72]
            pub.last_write = -1
            pub.update({'Pit':'24.5','Rol':'49.5','Bal':'3'})
            self.assertEqual(json.loads(path.read_text())['target'], [24.5,49.5])
            pub.last_write = -1
            pub.update({'App':'1','NOn':'0','NPit':'24.7','NRol':'49.72','Bal':'0'})
            data = json.loads(path.read_text())
            self.assertFalse(data['neutral_enabled'])
            self.assertEqual(data['target'], [24.7,49.72])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.live = self.folder / 'live.json'
        self.leases = self.folder / 'leases'
        self.reader = SerialPoseReader('COM7', self.live, [23.4399, 50], 1, self.leases)
        self.addCleanup(self.reader.close)
        self.link = Link()

    def test_lease_filename_cannot_be_windows_serial_device(self):
        self.assertEqual(lease_path('COM7', self.leases).name, 'serial-COM7.json')
        self.assertEqual(lease_path(r'\\.\COM7', self.leases), lease_path('COM7', self.leases))

    def test_reader_continues_beyond_old_timeout_and_never_writes(self):
        with patch('pose_stream.serial.Serial', return_value=self.link):
            for now in (1, 601, 3601):
                self.link.telemetry(pitch=now/100)
                with patch('time.monotonic', return_value=now):
                    self.reader.step()
                self.assertEqual(json.loads(self.live.read_text())['pitch'], now/100)
        self.assertTrue(self.link.is_open)
        self.assertEqual(self.link.writes, [])

    def test_monitor_yields_then_resumes_after_test(self):
        handoff = SerialMonitorHandoff('COM7', self.leases)
        self.addCleanup(handoff.release)
        with patch('pose_stream.serial.Serial', return_value=self.link):
            self.reader.step()
            self.assertTrue(self.link.is_open)
            handoff.acquire()
            self.reader.step()
            self.assertFalse(self.link.is_open)
            self.assertEqual(self.reader.status, 'handoff')
            handoff.release()
            self.reader.step()
            self.assertTrue(self.link.is_open)
        self.assertEqual(self.link.writes, [])

    def test_usb_read_failure_reconnects_without_command(self):
        replacement = Link()
        with patch('pose_stream.serial.Serial', side_effect=[self.link, replacement]):
            self.reader.step()
            self.link.failed = True
            self.reader.step()
            self.assertFalse(self.link.is_open)
            with patch('time.monotonic', return_value=self.reader.retry_at+1):
                replacement.telemetry(24, 49)
                self.reader.step()
            self.assertTrue(replacement.is_open)
        self.assertEqual(json.loads(self.live.read_text())['pitch'], 24)
        self.assertEqual(replacement.writes, [])

    def test_dead_diagnostic_does_not_block_monitor_forever(self):
        path = lease_path('COM7', self.leases)
        path.parent.mkdir()
        path.write_text(json.dumps({'pid': 12345, 'token': 'dead'}))
        with patch('serial_monitor_handoff.process_alive', return_value=False), \
                patch('pose_stream.serial.Serial', return_value=self.link):
            self.reader.step()
        self.assertTrue(self.link.is_open)

    def test_second_diagnostic_cannot_remove_first_reservation(self):
        first = SerialMonitorHandoff('COM7', self.leases)
        self.addCleanup(first.release)
        second = SerialMonitorHandoff('COM7', self.leases)
        first.acquire()
        with self.assertRaises(RuntimeError):
            second.acquire()
        second.release()
        self.assertTrue(monitor_should_yield('COM7', self.leases))
        self.assertEqual(json.loads(first.path.read_text())['token'], first.token)

    def test_failed_open_releases_reservation(self):
        handoff = SerialMonitorHandoff('COM7', self.leases)
        self.link.failed = True
        with self.assertRaises(serial.SerialException):
            handoff.open(self.link, timeout=0)
        self.assertFalse(handoff.path.exists())

    def test_handoff_waits_for_reader_to_close_port(self):
        handoff = SerialMonitorHandoff('COM7', self.leases)
        self.addCleanup(handoff.release)
        with patch('pose_stream.serial.Serial', return_value=self.link):
            self.reader.step()
        control = Link()
        actual_open = control.open
        def open_after_yield():
            self.reader.step()
            self.assertFalse(self.link.is_open)
            actual_open()
        with patch.object(control, 'open', side_effect=open_after_yield):
            handoff.open(control)
        self.assertTrue(control.is_open)
        control.close()
        handoff.release()
        with patch('pose_stream.serial.Serial', return_value=self.link):
            self.reader.step()
        self.assertTrue(self.link.is_open)


class DisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
        from PyQt5.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('pose_monitor', ROOT/'Software/pose_monitor.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_display_has_no_capture_button_and_resumes_with_fresh_data(self):
        from PyQt5.QtWidgets import QPushButton
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'live.json'
            window = self.module.PoseMonitor(path)
            window.timer.stop()
            try:
                window.refresh()
                self.assertEqual(window.findChildren(QPushButton), [])
                payload = {'timestamp': 100, 'pitch': 23.4, 'roll': 50, 'bal': 0,
                           'target': [23.4399, 50], 'capture_angle': 1}
                path.write_text(json.dumps(payload))
                with patch('time.time', return_value=100.1):
                    window.refresh()
                self.assertIn('+23.40', window.angles.text())
                with patch('time.time', return_value=102):
                    window.refresh()
                self.assertIn('—', window.angles.text())
                payload.update(timestamp=102, pitch=24)
                path.write_text(json.dumps(payload))
                with patch('time.time', return_value=102.1):
                    window.refresh()
                self.assertIn('+24.00', window.angles.text())
                self.assertEqual(window.status.text(), '电机失能')
                payload.update(bal=3, remaining_s=1.2)
                path.write_text(json.dumps(payload))
                with patch('time.time', return_value=102.1):
                    window.refresh()
                self.assertIn('剩余 1.2 秒', window.status.text())
                payload.update(bal=0, remaining_s=None, outcome_text='轮速保护停机')
                path.write_text(json.dumps(payload))
                with patch('time.time', return_value=102.1):
                    window.refresh()
                self.assertIn('轮速保护停机', window.status.text())
            finally:
                window.close()

    def test_nonmodal_reminder_once_per_trial_and_closes_on_takeover(self):
        from PyQt5.QtWidgets import QPushButton
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'live.json'
            window = self.module.PoseMonitor(path)
            window.timer.stop()
            try:
                payload = {'timestamp':100,'pitch':23.4,'roll':50,'bal':6,
                           'target':[24.7,49.72],'trial_id':'first','trial_text':'5秒对照'}
                def refresh():
                    path.write_text(json.dumps(payload))
                    with patch('time.time',return_value=100.1): window.refresh()
                refresh()
                self.assertTrue(window.reminder.isVisible())
                self.assertFalse(window.reminder.isModal())
                self.assertEqual(window.reminder.findChildren(QPushButton), [])
                self.assertIn('+24.70',window.reminder_angles.text())
                first = window.reminder
                first.close()
                refresh()
                self.assertFalse(first.isVisible())  # no repeat popup for the same trial
                payload.update(trial_id='second')
                refresh()
                self.assertTrue(window.reminder.isVisible())
                payload.update(bal=3)
                refresh()
                self.assertIsNone(window.reminder)
                self.assertIn('松手',window.status.text())
            finally:
                window.close()

    def test_connected_reader_without_pose_is_not_reported_as_reconnecting(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as folder:
            reader = SimpleNamespace(status='connected',step=lambda:None,close=lambda:None)
            window = self.module.PoseMonitor(Path(folder)/'missing.json',reader=reader)
            window.timer.stop()
            try:
                window.refresh()
                self.assertIn('串口已连接',window.status.text())
                reader.status='handoff'
                window.refresh()
                self.assertEqual(window.status.text(),'等待本轮实时数据')
            finally:
                window.close()


if __name__ == '__main__':
    unittest.main()
