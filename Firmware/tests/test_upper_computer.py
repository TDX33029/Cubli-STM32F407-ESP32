import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import importlib.util
from pathlib import Path
import socket
import threading
import time
import unittest
from PyQt5.QtCore import QPoint, QPointF, Qt
from PyQt5.QtGui import QWheelEvent
spec = importlib.util.spec_from_file_location('cubli_ui', Path(__file__).resolve().parents[1]/'Software/main.py')
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)
app = ui.QApplication.instance() or ui.QApplication([])
STATE = '$STATE,App:0,Dbg:2,Mode:0,BMode:1,Bal:0,Cal:1,IMU:1,Aligned:111,EN:010,Fault:0,Trip:0,Tilt:0.12,U1:0.00,U2:0.25,U3:0.00#'
AUTO_STATE = STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('Aligned:111','Aligned:100').replace('#',
    ',Auto:0,BReason:0,FullSpeed:250.0,FullMs:0,RetryMs:0,EdgeReady:1,Wheel:0.0,Rate:0.0,Angle:0.1,PoseTilt:0.1,TripDeg:20.0,VLimit:6.0,SLimit:330.0,Ref:1,AxesOK:1,DirOK:1,Axis1:2,D1:-1#')
CORNER_AUTO_STATE = AUTO_STATE.replace('BMode:1','BMode:4').replace('Aligned:100','Aligned:111').replace(
    'Axis1:2,D1:-1#','Axis1:2,Axis2:3,Axis3:1,D1:-1,D2:-1,D3:-1,AxisMask:7,DirMask:7,CornerStart:1,'
    'BTrim:0.0,TrimTune:1,CapDeg:0.50,Trim1:0.000,Trim2:0.000,Trim3:0.000#')
class UpperTests(unittest.TestCase):
    def setUp(self):
        self.win = ui.MainWindow()
        self.win.worker.is_running = True
        # 模拟已连接链路，避免未连接时点击自检按钮弹出模态提示框 (offscreen 下不可用)
        self.win.worker.ser = type('FakeSerial', (), {'is_open': True})()
        self.win.worker._process_received_line(STATE)
    def tearDown(self):
        self.win.worker.is_running = False
        self.win.close()
    def queued(self):
        return [item[0] for item in self.win.worker.tx_queue]
    def test_balance_only_left_panel(self):
        # 左侧面板仅保留平衡模式：单电机卡片/同步/闭环调试部件全部移除
        for name in ('card_m1', 'card_m2', 'card_m3', 'sync_group', 'cl_group', 'combo_app', 'combo_debug',
                     'combo_pid_motor', 'btn_mode_toggle', 'spin_sync'):
            self.assertFalse(hasattr(self.win, name), name)
        self.assertTrue(hasattr(self.win, 'card_self_test'))
        self.assertTrue(hasattr(self.win, 'btn_three_axis_test'))
        self.assertTrue(hasattr(self.win, 'btn_balance_start'))
    def test_all_dropdowns_ignore_wheel_including_when_focused(self):
        combos=self.win.findChildren(ui.QComboBox)
        self.assertGreaterEqual(len(combos),10)
        for combo in combos:
            self.assertIsInstance(combo,ui.NoWheelComboBox)
            if combo.count()<2:combo.addItem('测试候选')
            combo.setCurrentIndex(0)
            combo.setFocus()
            self.win.worker.clear_pending()
            for delta in (120,-120,360):
                event=QWheelEvent(QPointF(5,5),QPointF(5,5),QPoint(0,0),QPoint(0,delta),Qt.NoButton,Qt.NoModifier,Qt.ScrollUpdate,False)
                app.sendEvent(combo,event)
                self.assertEqual(combo.currentIndex(),0)
                self.assertEqual(self.queued(),[])
        self.win.combo_balance.setCurrentIndex(3)
        self.win.combo_balance.activated.emit(3)
        self.assertEqual(self.queued(),['BMODE 4'])
    def test_three_axis_self_test_from_boot_without_mode_switch(self):
        initial=AUTO_STATE.replace('App:1','App:0').replace('Cal:1','Cal:0').replace('Aligned:100','Aligned:000').replace('Ref:1','Ref:0').replace('AxesOK:1','AxesOK:0').replace('DirOK:1','DirOK:0')
        self.win.worker._process_received_line(initial)
        self.assertFalse(self.win.auto_edge_group.isHidden())
        self.assertTrue(self.win.btn_three_axis_test.isEnabled())
        self.assertTrue(self.win.card_self_test.btn_run_test.isEnabled())
        self.assertTrue(self.win.btn_edge_imu_cal.isEnabled())
        self.assertFalse(self.win.btn_edge_profile.isEnabled())
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.assertIn('三轴自检',self.win.lbl_edge_setup.text())
        self.win.btn_three_axis_test.click()
        self.assertEqual(self.queued(),['APP 0','TEST 1'])
        self.assertTrue(self.win.self_test_running)
        self.assertNotIn('TEST 2',self.queued())
    def test_three_axis_self_test_sequence_and_merge(self):
        self.win.worker._process_received_line(AUTO_STATE.replace('Aligned:100','Aligned:000'))
        self.win.card_self_test.btn_run_test.click()
        self.assertEqual(self.queued(),['APP 0','TEST 1'])
        self.win.worker.clear_pending()
        self.win.worker._process_received_line('$TEST_REPORT,M1:PASS:100:432.1,M2:SKIP:0:0.0,M3:SKIP:0:0.0,ALL:PASS#')
        self.assertEqual(self.queued(),['TEST 2'])
        self.assertTrue(self.win.self_test_running)
        self.win.worker.clear_pending()
        self.win.worker._process_received_line('$TEST_REPORT,M1:SKIP:0:0.0,M2:PASS:100:428.9,M3:SKIP:0:0.0,ALL:PASS#')
        self.assertEqual(self.queued(),['TEST 3'])
        self.win.worker.clear_pending()
        self.win.worker._process_received_line('$TEST_REPORT,M1:SKIP:0:0.0,M2:SKIP:0:0.0,M3:PASS:100:435.0,ALL:PASS#')
        self.assertEqual(self.queued(),['STATE?'])
        self.assertFalse(self.win.self_test_running)
        self.assertTrue(self.win.self_test_passed)
        self.assertEqual(self.win.card_self_test.lbl_badge.text(),'自检通过 (已解锁)')
        self.assertEqual(self.win.card_self_test.lbl_m_tags[2].text(),'正常')
    def test_three_axis_self_test_failure_keeps_lock(self):
        self.win.worker._process_received_line(AUTO_STATE.replace('Aligned:100','Aligned:000'))
        self.win.btn_three_axis_test.click()
        for report in ('$TEST_REPORT,M1:PASS:100:432.1,M2:SKIP:0:0.0,M3:SKIP:0:0.0,ALL:PASS#',
                       '$TEST_REPORT,M1:SKIP:0:0.0,M2:FAIL_MAP:10:3.0,M3:SKIP:0:0.0,ALL:FAIL#',
                       '$TEST_REPORT,M1:SKIP:0:0.0,M2:SKIP:0:0.0,M3:PASS:100:435.0,ALL:PASS#'):
            self.win.worker._process_received_line(report)
        self.assertFalse(self.win.self_test_running)
        self.assertFalse(self.win.self_test_passed)
        self.assertEqual(self.win.card_self_test.lbl_m_tags[1].text(),'映射检测失败')
        self.assertIn('锁定',self.win.card_self_test.lbl_badge.text())
    def test_emergency_stop_cancels_running_self_test(self):
        self.win.worker._process_received_line(AUTO_STATE.replace('Aligned:100','Aligned:000'))
        self.win.btn_three_axis_test.click()
        self.assertTrue(self.win.self_test_running)
        self.win.action_emergency_stop()
        self.assertFalse(self.win.self_test_running)
        self.assertEqual(self.win.self_test_queue,[])
        self.assertEqual(self.queued(),['STOP'])
    def test_connected_port_is_reflected_without_selection_commands(self):
        self.win.combo_ports.addItem('测试串口','COM_TEST')
        self.win.on_serial_connected('COM_TEST',115200)
        self.assertEqual(self.win.combo_ports.currentData(),'COM_TEST')
        self.assertEqual(self.queued(),['APP 0','STATE?'])
    def test_calibration_then_profile_unlocks_with_visible_reason(self):
        initial=AUTO_STATE.replace('Cal:1','Cal:0').replace('Ref:1','Ref:0')
        self.win.worker._process_received_line(initial)
        self.assertIn('静止校准',self.win.lbl_edge_setup.text())
        self.win.btn_edge_imu_cal.click()
        self.assertEqual(self.queued(),['CAL_IMU'])
        self.win.worker._process_received_line(initial.replace('Cal:0','Cal:1'))
        self.assertTrue(self.win.btn_edge_profile.isEnabled())
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.assertIn('加载已验证',self.win.lbl_edge_setup.text())
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
    def test_axis_map_label_from_state(self):
        state=STATE.replace('#',',Map1:3,Map2:1,Map3:2,PP1:7,PP2:14,PP3:7,Dir1:1,Dir2:-1,Dir3:1#')
        self.win.worker._process_received_line(state)
        text=self.win.card_self_test.lbl_maps.text()
        self.assertIn('M1→ENC3(PP7)',text)
        self.assertIn('M2→ENC1(PP14)',text)
        self.assertIn('M3→ENC2(PP7)',text)
    def test_reference_required_and_mapping_visible(self):
        state=STATE.replace('App:0','App:1').replace('#', ',Ref:0,Map1:3,Map2:1,Map3:2,PP1:7,PP2:14,PP3:7,Dir1:1,Dir2:-1,Dir3:1#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.assertIn('ENC1', self.win.card_self_test.lbl_maps.text())
        self.assertIn('PP14', self.win.card_self_test.lbl_maps.text())
        self.win.worker._process_received_line(state.replace('Ref:0','Ref:1'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_auto_edge_start_stop_and_protection_status(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 1'])
        waiting=AUTO_STATE.replace('Auto:0','Auto:1').replace('Bal:0','Bal:6').replace('BReason:0','BReason:2').replace('RetryMs:0','RetryMs:500')
        self.win.worker._process_received_line(waiting)
        self.assertIn('满速持续3秒',self.win.lbl_edge_detect.text())
        self.assertIn('0.5秒',self.win.lbl_edge_detect.text())
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 0'])
        self.win.worker._process_received_line(AUTO_STATE.replace('Bal:0','Bal:3').replace('EdgeReady:1','EdgeReady:0'))
        self.assertIn('正在平衡',self.win.lbl_edge_detect.text())
    def test_auto_edge_requires_reference_limit_and_selected_axis(self):
        for bad in (AUTO_STATE.replace('Ref:1','Ref:0'),AUTO_STATE.replace('Aligned:100','Aligned:000'),
                    AUTO_STATE.replace('SLimit:330.0','SLimit:180.0'),AUTO_STATE.replace('BMode:1','BMode:4')):
            self.win.worker._process_received_line(bad)
            self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line(STATE)
        self.assertFalse(self.win.btn_auto_edge.isEnabled()) # Older firmware lacks Auto.
    def test_auto_corner_start_stop_and_detect_label(self):
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.assertIn('单点',self.win.btn_auto_edge.text())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 1 2'])
        running=CORNER_AUTO_STATE.replace('Auto:0','Auto:1').replace('Bal:0','Bal:3').replace('EdgeReady:1','EdgeReady:0')
        self.win.worker._process_received_line(running)
        self.assertIn('单点（三轴）检测',self.win.lbl_edge_detect.text())
        self.assertIn('正在平衡',self.win.lbl_edge_detect.text())
        self.assertIn('停止自动单点',self.win.btn_auto_edge.text())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 0'])

    def test_auto_corner_requires_firmware_capability_three_axes_and_limits(self):
        for bad in (CORNER_AUTO_STATE.replace('CornerStart:1','CornerStart:0'),
                    CORNER_AUTO_STATE.replace('Aligned:111','Aligned:110'),
                    CORNER_AUTO_STATE.replace('Ref:1','Ref:0'),
                    CORNER_AUTO_STATE.replace('AxesOK:1','AxesOK:0'),
                    CORNER_AUTO_STATE.replace('SLimit:330.0','SLimit:250.0')):
            self.win.worker._process_received_line(bad)
            self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())

    def test_corner_profile_loads_only_after_three_axis_self_test(self):
        self.win.worker._process_received_line(CORNER_AUTO_STATE.replace('Aligned:111','Aligned:011'))
        self.assertFalse(self.win.btn_corner_profile.isEnabled())
        self.assertIn('三轴硬件自检',self.win.lbl_edge_setup.text())
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.assertTrue(self.win.btn_corner_profile.isEnabled())

    def test_corner_profile_waits_for_ack_and_fresh_matching_state(self):
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.win.btn_corner_profile.click()
        self.assertTrue(self.win.edge_profile_pending)
        queued=self.queued()
        for command in ('BMODE 4','BAXIS 2 3 1','BDIR -1 -1 -1','U 6','L 330','BYAW 3','BTRIM 0','BTRIP 20','BFULL 250','BCAP 0.5',
                        'LQR1 80 5 -0.030 0','LQR2 80 5 -0.030 0','LQR3 80 5 -0.030 0',
                        'BREF -0.7028287890572913 0.3977869310324257 0.5897433770480731'):
            self.assertIn(command,queued)
        self.assertNotIn('BAUTO 1',queued)
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.assertTrue(self.win.edge_profile_pending)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(CORNER_AUTO_STATE.replace('BTrim:0.0','BTrim:2.0'))
        self.assertTrue(self.win.edge_profile_pending)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line(CORNER_AUTO_STATE.replace('CapDeg:0.50','CapDeg:2.00'))
        self.assertTrue(self.win.edge_profile_pending)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line(CORNER_AUTO_STATE)
        self.assertFalse(self.win.edge_profile_pending)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.assertAlmostEqual(self.win.balance_spins[2].value(),-.030)
        self.assertEqual(self.win.combo_balance.currentIndex(),3)

    def test_edge_profile_waits_for_ack_and_fresh_matching_state(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_edge_profile.click()
        self.assertTrue(self.win.edge_profile_pending)
        self.assertIn('LQR1 80 5 -0.045 0',self.queued())
        self.assertIn('BFULL 250',self.queued())
        self.assertNotIn('BAUTO 1',self.queued())
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertTrue(self.win.edge_profile_pending)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertFalse(self.win.edge_profile_pending)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.assertAlmostEqual(self.win.balance_spins[2].value(),-.045)
    def test_edge_profile_error_cancels_remaining_commands(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_edge_profile.click()
        self.win.worker._process_received_line('ERR BREF')
        self.assertFalse(self.win.edge_profile_pending)
        self.assertEqual(self.queued(),['STOP'])
    def test_auto_fields_validation_and_user_edit_preservation(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.spin_full_speed.setValue(270)
        self.win.spin_trip_angle.setValue(25)
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertEqual(self.win.spin_full_speed.value(),270)
        self.assertEqual(self.win.spin_trip_angle.value(),25)
        before=self.win.device_state.copy()
        for bad in (AUTO_STATE.replace('Auto:0','Auto:2'),AUTO_STATE.replace('FullMs:0','FullMs:3001'),AUTO_STATE.replace('Rate:0.0','Rate:nan')):
            self.win.worker._process_received_line(bad)
            self.assertEqual(self.win.device_state,before)
    def test_emergency_stop_cancels_auto_arm(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_auto_edge.click()
        self.win.action_emergency_stop()
        self.assertEqual(self.queued(),['STOP'])
    def test_startup_auto_load_then_arm_after_confirmed_profile(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.assertTrue(self.win.edge_profile_pending)
        self.assertTrue(self.win.startup_auto_edge)
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.assertEqual(self.queued(),['BAUTO 1'])
    def test_startup_auto_cannot_override_emergency_stop(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.action_emergency_stop()
        app.processEvents()
        self.assertEqual(self.queued(),['STOP'])
        self.assertFalse(self.win.startup_auto_edge)
    def test_startup_auto_requires_alignment_and_calibration(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE.replace('Cal:1','Cal:0'))
        app.processEvents()
        self.assertFalse(self.win.startup_auto_edge)
        self.assertEqual(self.queued(),[])
    def test_uncalibrated_direction_flag_is_valid_boot_state(self):
        initial=AUTO_STATE.replace('DirOK:1','DirOK:0').replace('AxesOK:1','AxesOK:0').replace('Ref:1','Ref:0')
        self.win.worker._process_received_line(initial)
        self.assertEqual(self.win.device_state['DirOK'],0)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.assertTrue(self.win.btn_edge_profile.isEnabled())
    def test_body_axes_need_confirmation(self):
        state=STATE.replace('App:0','App:1').replace('#', ',Ref:1,AxesOK:0,DirOK:0,Axis1:1,Axis2:2,Axis3:3#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.axis_combos[0].setCurrentIndex(1)
        self.win.axis_combos[1].setCurrentIndex(2)
        self.win.axis_combos[2].setCurrentIndex(0)
        self.win.action_send_axes()
        self.assertEqual(self.queued(), ['BAXIS 2 3 1'])
        self.win.worker._process_received_line(state.replace('AxesOK:0','AxesOK:1').replace('DirOK:0','DirOK:1'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_edge_countdown_reference_and_active_lock(self):
        state = STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('#', ',Ref:0,AxesOK:1,DirOK:1,AxisMask:1,DirMask:1,CountMs:0#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.worker._process_received_line(state.replace('Ref:0','Ref:1'))
        self.assertTrue(self.win.btn_balance_countdown.isEnabled())
        self.win.btn_balance_countdown.click()
        self.assertIn('BSTART 8', self.queued())
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:5').replace('CountMs:0','CountMs:8000'))
        self.assertEqual(self.win.device_state['Bal'],5)
        self.assertIn('8.0',self.win.lbl_balance.text())
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:6'))
        self.assertEqual(self.win.device_state['Bal'],6)
        self.assertIn('等待扶到',self.win.lbl_balance.text())
    def test_balance_start_switches_to_work_mode(self):
        self.win.worker._process_received_line(AUTO_STATE.replace('App:1','App:0'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
        self.win.btn_balance_start.click()
        self.assertEqual(self.queued(),['APP 1','BAL 1'])
    def test_configurable_trip_angle(self):
        state=STATE.replace('App:0','App:1').replace('#',',TripDeg:20.0#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.spin_trip_angle.value(),20)
        self.win.btn_trip_angle.click()
        self.assertEqual(self.queued(),['BAL 0','BTRIP 20.0'])
        self.win.worker.clear_pending()
        self.win.worker._process_received_line(state.replace('TripDeg:20.0','TripDeg:60.0'))
        self.assertEqual(self.win.spin_trip_angle.value(),60)
        self.win.btn_trip_angle.click()
        self.assertEqual(self.queued(),['BAL 0','BTRIP 60.0'])
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:5'))
        self.assertFalse(self.win.btn_trip_angle.isEnabled())
    def test_balance_wheel_limit_readback(self):
        self.win.device_state=None
        state=STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('#',',VLimit:6.0,SLimit:330.0#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.spin_vlim.value(),330)
        self.assertGreaterEqual(self.win.spin_vlim.maximum(),360)
    def test_corner_requires_all_axes(self):
        state = STATE.replace('App:0','App:1').replace('BMode:1','BMode:4').replace('Aligned:111','Aligned:110')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.worker._process_received_line(state.replace('Aligned:110','Aligned:111'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_corner_countdown_requires_firmware_capability_and_all_axes(self):
        state=AUTO_STATE.replace('BMode:1','BMode:4').replace('Aligned:100','Aligned:111')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
        self.win.worker._process_received_line(state.replace('#',',CornerStart:1#'))
        self.assertTrue(self.win.btn_balance_countdown.isEnabled())
        self.assertIn('单点',self.win.btn_balance_countdown.text())
        self.assertIn('2秒',self.win.btn_balance_countdown.text())
        self.assertIn('0.5°',self.win.btn_balance_countdown.text())
        self.win.btn_balance_countdown.click()
        self.assertEqual(self.queued(),['BSTART 2'])
        self.win.worker._process_received_line(state.replace('Aligned:111','Aligned:110').replace('#',',CornerStart:1#'))
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
    def test_balance_gain_readback(self):
        self.win.combo_gain_axis.setCurrentIndex(2)
        self.win.worker._process_received_line('OK LQR3: P=29.000, D=2.300, W=-0.0200, I=0.0010')
        self.assertAlmostEqual(self.win.balance_spins[2].value(), -.02)
    def test_old_and_extended_telemetry(self):
        received=[]
        self.win.worker.sig_telemetry.connect(received.append)
        base='$TELE,M1:0.00,M2:1.00,M3:0.00,Vq:5.00,EN:010,A1:0.00,A2:0.12,A3:0.00'
        self.win.worker._process_received_line(base+'#')
        full=base+',Gx:0.0,Gy:0.0,Gz:0.0,Ax:0.00,Ay:0.00,Az:1.00,H1:0.0,H2:1.2,H3:0.0,S1:0.00,S2:1.00,S3:0.00,Pit:0.00,Rol:0.00,Bal:0#'
        self.win.worker._process_received_line(full)
        self.assertEqual(len(received),2)
        self.assertTrue(received[1]['has_speed'])
        self.assertEqual(received[1]['bal'],0)
        self.assertEqual(len(self.win.history_s1),len(self.win.history_time))
    def test_quad_mode_state_and_count_labels(self):
        state = STATE.replace('#', ',EncType:1,C1:4096,C2:4000,C3:2048,Enc:111#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.device_state['EncType'], 1)
        self.assertEqual(self.win.device_state['C2'], 4000)
        self.assertIn('4096/4000/2048', self.win.lbl_balance.text())
    def test_invalid_state_is_ignored(self):
        before = self.win.device_state.copy()
        for bad in (STATE.replace('Dbg:2', 'Dbg:9'), STATE.replace('Tilt:0.12', 'Tilt:nan'), STATE.replace('Cal:1', 'Cal:3')):
            self.win.worker._process_received_line(bad)
        self.assertEqual(self.win.device_state, before)
    def test_installation_direction_does_not_override_user_edit(self):
        self.win.direction_combos[0].setCurrentIndex(1)
        self.win.worker._process_received_line(STATE.replace('#', ',D1:1,D2:1,D3:1#'))
        self.assertEqual(self.win.direction_combos[0].currentIndex(), 1)
        self.win.on_log_rx('OK BDIR')
        self.win.worker._process_received_line(STATE.replace('#', ',D1:-1,D2:1,D3:1#'))
        self.assertEqual(self.win.direction_combos[0].currentIndex(), 1)
    def test_tcp_eof_is_error(self):
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen()
        link=ui.TcpLink('127.0.0.1',server.getsockname()[1])
        peer,_=server.accept();peer.close()
        deadline=time.monotonic()+1
        with self.assertRaises(ConnectionError):
            while time.monotonic()<deadline:
                link.in_waiting
                time.sleep(.01)
        link.close();server.close()
    def test_worker_heartbeat_and_disconnect(self):
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen()
        received=[]
        def device():
            peer,_=server.accept();peer.settimeout(2);peer.sendall((STATE+'\r\n').encode())
            data=b''
            try:
                while b'STOP' not in data:
                    chunk=peer.recv(4096)
                    if not chunk: break
                    data+=chunk
            finally:
                received.append(data);peer.close()
        thread=threading.Thread(target=device);thread.start()
        worker=ui.SerialWorker();worker.connect_wifi('127.0.0.1',server.getsockname()[1])
        time.sleep(.55);worker.disconnect_port();thread.join(2);server.close()
        self.assertFalse(worker.isRunning())
        self.assertGreaterEqual(received[0].count(b'PING'),2)
        self.assertIn(b'STOP',received[0])
        self.assertFalse(worker.tx_queue)
if __name__=='__main__': unittest.main()
