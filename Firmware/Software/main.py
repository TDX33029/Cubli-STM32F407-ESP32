# -*- coding: utf-8 -*-
"""
Cubli 三轴无刷电机联调上位机 (PyQt5 + pyqtgraph + pyserial)
-------------------------------------------------------------------------
适用硬件: STM32F407ZGT6 + 3x DRV8313 (M1/M2/M3) + LSM6DSRTR / MPU-6050
         + 板载 ESP32-WROOM32E 无线链路
通信协议:
  - 有线模式: USART2 (PD5-TX, PD6-RX), 115200 8N1
  - 无线模式: F407 USART1 (PA9/PA10) -> ESP32 (IO25/IO26) -> WiFi TCP :3333
    两种链路字节流协议完全一致, 可在下拉框中随时切换
主要特性:
  1. 无多余 emoji，工控简洁高对比度深色风格
  2. 左侧电机调控与参数面板，右侧 3D 姿态立方体与电机转速实时曲线
  3. 修复转速曲线显示异常问题 (采用示波器式平滑滚动时间窗，修复 autoRange 导致的数据出界)
  4. 纯 QPainter 高性能 3D 立方体引擎，与陀螺仪/加速度计互补滤波姿态实时同步，默认水平朝上
  5. 飞轮转速联动动画、坐标轴指示与鼠标自由旋转视角
  6. 支持 WiFi(TCP) 无线连接 ESP32 (AP 热点直连默认 192.168.4.1:3333)
  7. 修复 QComboBox 下拉弹层在系统深色模式下回落到暗色调色板的显示异常
"""

import sys
import time
import math
import re
import socket
import select
import argparse
import json
from pathlib import Path
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QMutex, QMutexLocker, QPointF
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSlider, QDoubleSpinBox, QCheckBox, QTextEdit, QLineEdit,
    QSplitter, QFrame, QMessageBox, QTabWidget, QStatusBar, QProgressBar,
    QScrollArea
)
from PyQt5.QtGui import (
    QFont, QColor, QPalette, QPainter, QPolygonF, QPen, QBrush,
    QLinearGradient, QRadialGradient, QPainterPath
)

import pyqtgraph as pg

# 配置 pyqtgraph 全局浅色高对比度样式
pg.setConfigOption('background', '#ffffff')
pg.setConfigOption('foreground', '#334155')
pg.setConfigOption('antialias', True)


# =========================================================================
# WiFi(TCP) 链路封装: 与 pyserial.Serial 接口语义对齐
# 用于连接板载 ESP32-WROOM32E 的 TCP 服务器 (默认 192.168.4.1:3333),
# 数据流与有线串口完全一致 ($TELE...# 帧 / ASCII 命令行)
# =========================================================================
class TcpLink:
    """TCP 链路, 提供 is_open / in_waiting / read / write / close 接口"""

    def __init__(self, host, port, timeout=0.1):
        self._sock = socket.create_connection((host, int(port)), timeout=3.0)
        self._sock.settimeout(timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    @property
    def is_open(self):
        return self._sock is not None

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def write(self, data):
        if self._sock is None:
            raise OSError("TCP link closed")
        self._sock.sendall(data)

    @property
    def in_waiting(self):
        if self._sock is None:
            return 0
        try:
            readable, _, _ = select.select([self._sock], [], [], 0)
            if not readable:
                return 0
            data = self._sock.recv(65536, socket.MSG_PEEK)
            if not data:
                raise ConnectionError('TCP 对端已断开')
            return len(data)
        except (OSError, ValueError) as exc:
            raise ConnectionError(str(exc)) from exc

    def read(self, n=1):
        if self._sock is None:
            return b""
        try:
            return self._sock.recv(min(int(n), 65536))
        except socket.timeout:
            return b""
        except OSError as exc:
            raise ConnectionError(str(exc)) from exc


# =========================================================================
# 禁用鼠标滚轮调整的自定义控件 (防止在滚动面板时误触修改速度与参数)
# =========================================================================
class NoWheelSlider(QSlider):
    """水平滑块组件：禁用鼠标滚轮，防止在滚动左侧控制面板时误触更改速度"""
    def wheelEvent(self, event):
        event.ignore()


class NoWheelDoubleSpinBox(QDoubleSpinBox):
    """数值输入框组件：禁用鼠标滚轮，防止滚轮滑动时误改转速与参数"""
    def wheelEvent(self, event):
        event.ignore()


class NoWheelComboBox(QComboBox):
    """Keep dropdown selections unchanged while scrolling the control panel."""
    def wheelEvent(self, event):
        event.ignore()


# =========================================================================
# 串口后台通信工作线程 (QThread)
# =========================================================================
class SerialWorker(QThread):
    """串口数据收发与协议解析线程"""
    sig_connected = pyqtSignal(str, int)
    sig_disconnected = pyqtSignal()
    sig_error = pyqtSignal(str)
    sig_log_rx = pyqtSignal(str)
    sig_mode_changed = pyqtSignal(int)
    sig_pid_received = pyqtSignal(int, float, float, float)
    sig_log_tx = pyqtSignal(str)
    sig_telemetry = pyqtSignal(dict)
    sig_state = pyqtSignal(dict)
    sig_lqr = pyqtSignal(int, float, float, float, float)
    sig_test_step = pyqtSignal(str)     # 自检单步进度: "M1", "M2", "M3"
    sig_test_report = pyqtSignal(dict)  # 自检总结结果字典

    # 自检协议正则:
    # 步骤帧: $TEST_STEP,M1#
    # 总结帧: $TEST_REPORT,M1:PASS:100:432.1,M2:PASS:100:428.9,M3:PASS:100:435.0,ALL:PASS#
    TEST_STEP_PATTERN = re.compile(r'\$TEST_STEP,(M[123])#')
    TEST_REPORT_PATTERN = re.compile(
        r'\$TEST_REPORT,'
        r'M1:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'M2:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'M3:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'ALL:(PASS|FAIL)#'
    )

    # 遥测帧正则:
    # 基础帧: $TELE,M1:%.2f,M2:%.2f,M3:%.2f,Vq:%.2f,EN:%d%d%d,A1:%.2f,A2:%.2f,A3:%.2f#
    # 扩展帧1 (IMU): ...,Gx:%.1f,Gy:%.1f,Gz:%.1f,Ax:%.2f,Ay:%.2f,Az:%.2f#
    # 扩展帧2 (IMU + Hall): ...,H1:%.1f,H2:%.1f,H3:%.1f#
    # 扩展帧3 (Closed-loop Speed): ...,S1:%.2f,S2:%.2f,S3:%.2f#
    TELE_PATTERN = re.compile(
        r'\$TELE,'
        r'M1:([+-]?\d+(?:\.\d+)?),'
        r'M2:([+-]?\d+(?:\.\d+)?),'
        r'M3:([+-]?\d+(?:\.\d+)?),'
        r'Vq:([+-]?\d+(?:\.\d+)?),'
        r'EN:([01])([01])([01]),'
        r'A1:([+-]?\d+(?:\.\d+)?),'
        r'A2:([+-]?\d+(?:\.\d+)?),'
        r'A3:([+-]?\d+(?:\.\d+)?)(?:,'
        r'Gx:([+-]?\d+(?:\.\d+)?),'
        r'Gy:([+-]?\d+(?:\.\d+)?),'
        r'Gz:([+-]?\d+(?:\.\d+)?),'
        r'Ax:([+-]?\d+(?:\.\d+)?),'
        r'Ay:([+-]?\d+(?:\.\d+)?),'
        r'Az:([+-]?\d+(?:\.\d+)?))?'
        r'(?:,H1:([+-]?\d+(?:\.\d+)?),'
        r'H2:([+-]?\d+(?:\.\d+)?),'
        r'H3:([+-]?\d+(?:\.\d+)?))?'
        r'(?:,S1:([+-]?\d+(?:\.\d+)?),'
        r'S2:([+-]?\d+(?:\.\d+)?),'
        r'S3:([+-]?\d+(?:\.\d+)?))?'
        r'(?:,Pit:([+-]?\d+(?:\.\d+)?),'
        r'Rol:([+-]?\d+(?:\.\d+)?),'
        r'Bal:(\d+))?#'
    )
    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_running = False
        self.mutex = QMutex()
        self.tx_queue = deque()

    def connect_port(self, port_name, baud_rate=115200):
        """请求连接串口"""
        try:
            self.disconnect_port()
            self.clear_pending()
            self.ser = serial.Serial(
                port=None,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1, write_timeout=0.3
            )
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.port = port_name
            self.ser.open()
            self.is_running = True
            if not self.isRunning():
                self.start()
            self.sig_connected.emit(port_name, baud_rate)
        except Exception as e:
            self.sig_error.emit(f"无法打开串口 {port_name}: {str(e)}")

    def connect_wifi(self, host, port=3333, baud_rate=115200):
        """请求连接 ESP32 WiFi(TCP) 无线链路 (字节流协议与串口一致)"""
        try:
            self.disconnect_port()
            self.clear_pending()
            self.ser = TcpLink(host, port)
            self.is_running = True
            if not self.isRunning():
                self.start()
            self.sig_connected.emit(f"{host}:{port}(WiFi)", baud_rate)
        except Exception as e:
            self.sig_error.emit(f"无法连接 WiFi 设备 {host}:{port}: {str(e)}")

    def disconnect_port(self):
        self.is_running = False
        if QThread.currentThread() != self:
            self.wait(1500)

    def send_cmd(self, cmd_str):
        cmd_str = cmd_str.strip()
        if not cmd_str or not self.is_running:
            return
        payload = (cmd_str + "\r\n").encode('ascii')
        with QMutexLocker(self.mutex):
            if cmd_str.upper() in ("STOP", "S"):
                self.tx_queue.clear()
                self.tx_queue.appendleft((cmd_str, payload))
            else:
                self.tx_queue.append((cmd_str, payload))

    def clear_pending(self):
        with QMutexLocker(self.mutex):
            self.tx_queue.clear()

    def run(self):
        rx_buffer = bytearray()
        next_ping = next_query = 0.0
        last_rx = time.monotonic()
        try:
            while self.is_running and self.ser and self.ser.is_open:
                now = time.monotonic()
                # Heartbeats originate at the PC; ESP never manufactures them.
                if now >= next_ping:
                    self.ser.write(b"PING\r\n")
                    next_ping = now + 0.25
                if now >= next_query:
                    self.ser.write(b"STATE?\r\n")
                    next_query = now + 1.0
                item = None
                with QMutexLocker(self.mutex):
                    if self.tx_queue:
                        item = self.tx_queue.popleft()
                if item:
                    self.ser.write(item[1])
                    self.sig_log_tx.emit(item[0])
                waiting = self.ser.in_waiting
                if waiting:
                    chunk = self.ser.read(waiting)
                    if chunk:
                        last_rx = now
                        rx_buffer.extend(chunk)
                        if len(rx_buffer) > 65536:
                            raise ConnectionError("接收缓冲溢出")
                        while b'\n' in rx_buffer:
                            raw_line, _, rx_buffer = rx_buffer.partition(b'\n')
                            decoded = raw_line.decode('utf-8', errors='replace').strip()
                            if decoded:
                                self._process_received_line(decoded)
                elif now - last_rx > 6.0:
                    raise ConnectionError("设备超过 6 秒未响应，链路已关闭")
                else:
                    self.msleep(5)
        except Exception as exc:
            if self.is_running:
                self.sig_error.emit(f"通信中断: {exc}")
        finally:
            self.is_running = False
            self.clear_pending()
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.write(b"\nSTOP\r\n")
                except Exception:
                    pass
                self.ser.close()
            self.sig_disconnected.emit()

    def _process_received_line(self, line):
        """解析接收到的单行报文"""
        state_match = re.fullmatch(r'\$STATE,(.+)#', line)
        if state_match:
            try:
                fields = dict(item.split(':', 1) for item in state_match.group(1).split(','))
                for key in ('App', 'Dbg', 'Mode', 'BMode', 'Bal', 'Cal', 'IMU', 'Fault', 'Trip'):
                    fields[key] = int(fields[key])
                for key in ('Tilt', 'U1', 'U2', 'U3'):
                    fields[key] = float(fields[key])
                    if not math.isfinite(fields[key]):
                        raise ValueError('non-finite state')
                if fields['App'] not in (0, 1) or fields['Dbg'] not in (1, 2, 3) or fields['BMode'] not in (1, 2, 3, 4) or fields['Bal'] not in range(8):
                    raise ValueError('invalid state range')
                if any(fields[key] not in (0, 1) for key in ('Mode', 'Cal', 'IMU')):
                    raise ValueError('invalid flags')
                if 'Enc' in fields and not re.fullmatch('[01]{3}', fields['Enc']):
                    raise ValueError('invalid encoder mask')
                for key in ('VLimit', 'SLimit', 'TripDeg', 'PoseTilt', 'Angle', 'FullSpeed', 'Wheel', 'Rate', 'YawD',
                            'Trim1', 'Trim2', 'Trim3', 'BTrim', 'CapDeg', 'AttTau',
                            'NPit', 'NRol', 'NOff', 'NLimit', 'NRate', 'NGain'):
                    if key in fields:
                        fields[key] = float(fields[key])
                        if not math.isfinite(fields[key]):
                            raise ValueError('invalid limits')
                if 'CapDeg' in fields and not .5 <= fields['CapDeg'] <= 10:
                    raise ValueError('invalid capture angle')
                if 'AttTau' in fields and not .25 <= fields['AttTau'] <= 2:
                    raise ValueError('invalid attitude tau')
                if 'NLimit' in fields and not 2 <= fields['NLimit'] <= 5:
                    raise ValueError('invalid neutral limit')
                if 'NRate' in fields and not 0 < fields['NRate'] <= .5:
                    raise ValueError('invalid neutral rate')
                if 'NGain' in fields and not 0 < fields['NGain'] <= .00025:
                    raise ValueError('invalid neutral gain')
                if 'NOff' in fields and fields['NOff'] < 0:
                    raise ValueError('invalid neutral offset')
                for key, upper in (('NTune', 3), ('NOn', 1)):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if not 0 <= fields[key] <= upper:
                            raise ValueError('invalid neutral tuning state')
                if 'EncType' in fields:
                    fields['EncType'] = int(fields['EncType'])
                    if fields['EncType'] not in (0, 1):
                        raise ValueError('invalid encoder type')
                for key in ('C1', 'C2', 'C3'):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if not 1 <= fields[key] <= 16384:
                            raise ValueError('invalid CPR')
                for key in ('Map1', 'Map2', 'Map3', 'PP1', 'PP2', 'PP3', 'Dir1', 'Dir2', 'Dir3', 'Ref', 'Axis1', 'Axis2', 'Axis3', 'AxesOK', 'DirOK'):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if key.startswith('Map') and fields[key] not in (1, 2, 3):
                            raise ValueError('invalid map')
                        if key.startswith('PP') and not 1 <= fields[key] <= 32:
                            raise ValueError('invalid pole pairs')
                        if key in ('Dir1', 'Dir2', 'Dir3') and fields[key] not in (-1, 1):
                            raise ValueError('invalid encoder direction')
                        if key.startswith('Axis') and fields[key] not in (1, 2, 3):
                            raise ValueError('invalid body axis')
                        if key in ('Ref', 'AxesOK', 'DirOK') and fields[key] not in (0, 1):
                            raise ValueError('invalid reference status')
                for key in ('AxisMask', 'DirMask', 'CountMs'):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if not 0 <= fields[key] <= (15000 if key == 'CountMs' else 7):
                            raise ValueError('invalid countdown/configuration mask')
                for key in ('D1', 'D2', 'D3'):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if fields[key] not in (-1, 1):
                            raise ValueError('invalid installation direction')
                for key, upper in (('Auto', 1), ('EdgeReady', 1), ('CornerStart', 1), ('YawTune', 1), ('TrimTune', 1), ('BReason', 5), ('FullMs', 3000), ('RetryMs', 1000)):
                    if key in fields:
                        fields[key] = int(fields[key])
                        if not 0 <= fields[key] <= upper:
                            raise ValueError('invalid automatic edge state')
                if re.fullmatch('[01]{3}', fields['EN']) and re.fullmatch('[01]{3}', fields['Aligned']):
                    self.sig_state.emit(fields)
            except (ValueError, KeyError):
                pass
            return
        lqr = re.fullmatch(r'OK LQR([123]): P=([0-9.-]+), D=([0-9.-]+), W=([0-9.-]+), I=([0-9.-]+)', line)
        if lqr:
            self.sig_lqr.emit(int(lqr[1]), *[float(lqr[i]) for i in range(2, 6)])
        match = self.TELE_PATTERN.search(line)
        if match:
            try:
                m1_spd = float(match.group(1))
                m2_spd = float(match.group(2))
                m3_spd = float(match.group(3))
                vq = float(match.group(4))
                en1 = int(match.group(5)) == 1
                en2 = int(match.group(6)) == 1
                en3 = int(match.group(7)) == 1
                a1 = float(match.group(8))
                a2 = float(match.group(9))
                a3 = float(match.group(10))

                gx = float(match.group(11)) if match.group(11) is not None else 0.0
                gy = float(match.group(12)) if match.group(12) is not None else 0.0
                gz = float(match.group(13)) if match.group(13) is not None else 0.0
                ax = float(match.group(14)) if match.group(14) is not None else 0.0
                ay = float(match.group(15)) if match.group(15) is not None else 0.0
                az = float(match.group(16)) if match.group(16) is not None else 0.0
                has_imu = match.group(11) is not None

                h1 = float(match.group(17)) if match.group(17) is not None else 0.0
                h2 = float(match.group(18)) if match.group(18) is not None else 0.0
                h3 = float(match.group(19)) if match.group(19) is not None else 0.0
                has_hall = match.group(17) is not None

                s1 = float(match.group(20)) if match.group(20) is not None else 0.0
                s2 = float(match.group(21)) if match.group(21) is not None else 0.0
                s3 = float(match.group(22)) if match.group(22) is not None else 0.0
                has_speed = match.group(20) is not None
                pit = float(match.group(23)) if match.group(23) is not None else None
                rol = float(match.group(24)) if match.group(24) is not None else None
                bal = int(match.group(25)) if match.group(25) is not None else None

                tele_data = {
                    'm1_spd': m1_spd, 'm2_spd': m2_spd, 'm3_spd': m3_spd,
                    'vq': vq,
                    'en1': en1, 'en2': en2, 'en3': en3,
                    'a1': a1, 'a2': a2, 'a3': a3,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'ax': ax, 'ay': ay, 'az': az,
                    'h1': h1, 'h2': h2, 'h3': h3,
                    's1': s1, 's2': s2, 's3': s3,
                    'has_imu': has_imu,
                    'has_hall': has_hall,
                    'has_speed': has_speed,
                    'pitch': pit, 'roll': rol, 'bal': bal,
                    'time': time.time()
                }
                self.sig_telemetry.emit(tele_data)
            except Exception:
                pass

        # 解析自检单步步骤帧: $TEST_STEP,M1#
        match_step = self.TEST_STEP_PATTERN.search(line)
        if match_step:
            self.sig_test_step.emit(match_step.group(1))

        # 解析自检汇总报表帧: $TEST_REPORT,M1:PASS:100:432.1,M2:PASS:100:428.9,M3:PASS:100:435.0,ALL:PASS#
        match_rep = self.TEST_REPORT_PATTERN.search(line)
        if match_rep:
            try:
                rep_data = {
                    'm1': {'status': match_rep.group(1), 'comm': int(match_rep.group(2)), 'deg': float(match_rep.group(3))},
                    'm2': {'status': match_rep.group(4), 'comm': int(match_rep.group(5)), 'deg': float(match_rep.group(6))},
                    'm3': {'status': match_rep.group(7), 'comm': int(match_rep.group(8)), 'deg': float(match_rep.group(9))},
                    'all_pass': match_rep.group(10) == 'PASS'
                }
                self.sig_test_report.emit(rep_data)
            except Exception:
                pass
        elif "ALL 3 MOTORS & ENCODERS ARE HEALTHY (PASS)" in line:
            # 兼容终端回退文本
            self.sig_test_report.emit({
                'all_pass': True,
                'm1': {'status': 'PASS', 'comm': 100, 'deg': 360.0},
                'm2': {'status': 'PASS', 'comm': 100, 'deg': 360.0},
                'm3': {'status': 'PASS', 'comm': 100, 'deg': 360.0}
            })
        elif "ENCODER/MOTOR ANOMALY DETECTED (FAIL)" in line:
            self.sig_test_report.emit({
                'all_pass': False,
                'm1': {'status': 'FAIL', 'comm': 0, 'deg': 0.0},
                'm2': {'status': 'FAIL', 'comm': 0, 'deg': 0.0},
                'm3': {'status': 'FAIL', 'comm': 0, 'deg': 0.0}
            })


        if 'OK MODE:1' in line:
            self.sig_mode_changed.emit(1)
        elif 'OK MODE:0' in line:
            self.sig_mode_changed.emit(0)

        m_pid = re.search(r'OK PID([123]): P=([0-9.-]+), I=([0-9.-]+), D=([0-9.-]+)', line)
        if m_pid:
            self.sig_pid_received.emit(int(m_pid.group(1)), float(m_pid.group(2)), float(m_pid.group(3)), float(m_pid.group(4)))
        else:
            m_all_pid = re.search(r'OK ALL PID: P=([0-9.-]+), I=([0-9.-]+), D=([0-9.-]+)', line)
            if m_all_pid:
                self.sig_pid_received.emit(0, float(m_all_pid.group(1)), float(m_all_pid.group(2)), float(m_all_pid.group(3)))
            else:
                m_stat_pid = re.search(r'M([123]) \([^)]+\): P=([0-9.-]+), I=([0-9.-]+), D=([0-9.-]+)', line)
                if m_stat_pid:
                    self.sig_pid_received.emit(int(m_stat_pid.group(1)), float(m_stat_pid.group(2)), float(m_stat_pid.group(3)), float(m_stat_pid.group(4)))

        self.sig_log_rx.emit(line)


# =========================================================================
# 三维姿态立方体组件 (Cubli3DWidget - 纯 QPainter 高性能投影)
# =========================================================================
class Cubli3DWidget(QWidget):
    """
    Cubli 三维立方体渲染组件:
    - 纯基于 PyQt5 原生 QPainter 实现 3D 透视投影与画家算法深度着色
    - 与陀螺仪/加速度计互补滤波姿态实时同步
    - 默认初始状态: 水平朝上 (Roll=0, Pitch=0, Yaw=0, Z轴垂直向上)
    - 动量轮旋转动画跟随三电机目标转速
    - 支持鼠标拖拽旋转观察视角，双击恢复默认等轴测视角
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 220)
        self.setSizePolicy(self.sizePolicy().Expanding, self.sizePolicy().Expanding)

        # 欧拉角 (度, 对应机体姿态)
        self.roll = 0.0    # 绕 X 轴旋转 (横滚)
        self.pitch = 0.0   # 绕 Y 轴旋转 (俯仰)
        self.yaw = 0.0     # 绕 Z 轴旋转 (航向)

        # 姿态零位偏移校准
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

        # 相机观察视角 (度)
        self.cam_elev = 20.0  # 俯仰观察角 (固定 20.0° 低仰视角，符合大致 30° 以下标准)
        self.cam_azim = 35.0  # 方位观察角

        # 鼠标交互
        self._last_mouse_pos = None

        # 动量轮角度 (弧度)
        self.wheel_angles = [0.0, 0.0, 0.0]
        self.encoder_angles = [0.0, 0.0, 0.0]  # MT6701 实时角度 (度)
        self.motor_speeds = [0.0, 0.0, 0.0]

        # 互补滤波相关
        self.last_filter_time = None
        self.alpha = 0.95  # 互补滤波系数 (0.95 陀螺仪积分 + 0.05 加速度计重力)

        # 立方体半边长
        self.cube_size = 75.0

    def update_encoder_angles(self, h1, h2, h3):
        """直接使用 MT6701 实时角度驱动 3D 飞轮旋转"""
        self.encoder_angles = [h1, h2, h3]
        self.wheel_angles[0] = math.radians(h1)
        self.wheel_angles[1] = math.radians(h2)
        self.wheel_angles[2] = math.radians(h3)
        self.update()

    def reset_attitude_zero(self):
        """将当前姿态设为零位"""
        self.roll_offset = self.roll + self.roll_offset
        self.pitch_offset = self.pitch + self.pitch_offset
        self.yaw_offset = self.yaw + self.yaw_offset
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.update()

    def reset_view(self):
        """恢复默认观察视角 (固定 20.0° 低仰俯视视角)"""
        self.cam_elev = 20.0
        self.cam_azim = 35.0
        self.update()

    def update_imu_data(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g):
        """输入 6 轴数据，通过互补滤波解算欧拉角"""
        now = time.time()
        if self.last_filter_time is None:
            dt = 0.05
        else:
            dt = now - self.last_filter_time
            if dt <= 0.0 or dt > 0.2:
                dt = 0.05
        self.last_filter_time = now

        # 加速度计静态姿态解算 (默认水平朝上: Az ≈ +1.0g)
        norm_a = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        if norm_a > 0.1:
            ax = ax_g / norm_a
            ay = ay_g / norm_a
            az = az_g / norm_a

            # 俯仰角与横滚角 (度)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180.0 / math.pi)
            roll_acc  = math.atan2(ay, az) * (180.0 / math.pi)

            # 互补滤波
            self.pitch = self.alpha * (self.pitch + gy_dps * dt) + (1.0 - self.alpha) * pitch_acc
            self.roll  = self.alpha * (self.roll + gx_dps * dt) + (1.0 - self.alpha) * roll_acc
        else:
            self.pitch += gy_dps * dt
            self.roll  += gx_dps * dt

        # 航向角积分 (陀螺仪 Z 轴)
        self.yaw += gz_dps * dt

        # 累加动量轮转角动画
        for i in range(3):
            self.wheel_angles[i] += self.motor_speeds[i] * dt * 2.0
            if self.wheel_angles[i] > 2 * math.pi:
                self.wheel_angles[i] -= 2 * math.pi
            elif self.wheel_angles[i] < -2 * math.pi:
                self.wheel_angles[i] += 2 * math.pi

        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._last_mouse_pos is not None:
            dx = event.x() - self._last_mouse_pos.x()
            self._last_mouse_pos = event.pos()

            # 仅改变水平观察方向 (方位角 cam_azim)，俯仰角保持固定不变，地面始终保持绝对水平静止
            self.cam_azim = (self.cam_azim + dx * 0.5) % 360.0
            self.update()

    def mouseReleaseEvent(self, event):
        self._last_mouse_pos = None

    def mouseDoubleClickEvent(self, event):
        self.reset_view()

    # ---------------------------------------------------------------------
    # 3D 数学变换与渲染
    # ---------------------------------------------------------------------
    @staticmethod
    def _rot_x(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)

    @staticmethod
    def _rot_y(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)

    @staticmethod
    def _rot_z(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])

    def _world_to_cam(self, pt_world):
        """世界坐标系 -> 相机视点空间 (标准右手投影，固定 20.0° 低仰视角，地面基准绝对水平静止)"""
        xw, yw, zw = pt_world
        phi = math.radians(self.cam_azim)
        theta = math.radians(self.cam_elev)

        # 1. 绕世界铅垂 Z 轴旋转水平方位角 phi
        x1 = xw * math.cos(phi) - yw * math.sin(phi)
        y1 = xw * math.sin(phi) + yw * math.cos(phi)
        z1 = zw

        # 2. 俯视视角变换 (相机从水平面上方 theta 度向下俯视原点)
        xc = x1
        yc = z1 * math.cos(theta) + y1 * math.sin(theta)
        zc = -z1 * math.sin(theta) + y1 * math.cos(theta)
        return (xc, yc, zc)

    def _body_to_world(self, pt_body):
        """机体局部坐标系 -> 世界坐标系 (由姿态互补滤波解算欧拉角驱动)"""
        p = self._rot_x(pt_body, self.roll - self.roll_offset)
        p = self._rot_y(p, self.pitch - self.pitch_offset)
        p = self._rot_z(p, self.yaw - self.yaw_offset)
        return p

    def _transform_point(self, pt_body):
        """机体局部顶点 -> 世界坐标系 -> 相机投影空间"""
        pt_world = self._body_to_world(pt_body)
        return self._world_to_cam(pt_world)

    def _project(self, p_cam, cx, cy, d_cam=450.0):
        """透视投影变换"""
        z_eff = p_cam[2] + d_cam
        if z_eff < 10.0:
            z_eff = 10.0
        scale = d_cam / z_eff
        sx = cx + p_cam[0] * scale
        sy = cy - p_cam[1] * scale
        return QPointF(sx, sy), z_eff

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.TextAntialiasing, True)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0

        # 背景渐变 (素雅浅色现代质感)
        bg_grad = QRadialGradient(cx, cy, max(w, h) * 0.7)
        bg_grad.setColorAt(0.0, QColor(255, 255, 255))
        bg_grad.setColorAt(1.0, QColor(241, 245, 249))
        painter.fillRect(0, 0, w, h, bg_grad)

        s = self.cube_size

        # 1. 绘制绝对静止的参考水平地面 (处于世界坐标系，不随机体旋转，仅响应水平观察视角)
        self._draw_ground(painter, cx, cy)

        # 2. 立方体 8 个局部坐标顶点
        # 约定: Z轴向上 (顶面为+Z), Y轴向前 (前面为+Y), X轴向右 (右面为+X)
        verts_local = [
            (-s, -s, -s),  # 0
            ( s, -s, -s),  # 1
            ( s,  s, -s),  # 2
            (-s,  s, -s),  # 3
            (-s, -s,  s),  # 4
            ( s, -s,  s),  # 5
            ( s,  s,  s),  # 6
            (-s,  s,  s),  # 7
        ]

        # 变换顶点 (机体姿态 -> 世界 -> 相机)
        verts_cam = [self._transform_point(v) for v in verts_local]
        verts_2d = []
        for v in verts_cam:
            p2d, _ = self._project(v, cx, cy)
            verts_2d.append(p2d)

        # 3. 定义 6 个面及对应的飞轮
        # 每个面: (顶点索引[4个], 面法向量(局部), 基础色, 标签名称, 动量轮索引[或None])
        faces = [
            # 顶面 (+Z, M3 动量轮)
            ([4, 5, 6, 7], (0, 0, 1), QColor(41, 128, 185), f"M3: {self.encoder_angles[2]:05.1f}°", 2),
            # 底面 (-Z)
            ([3, 2, 1, 0], (0, 0, -1), QColor(100, 116, 139), "BOTTOM", None),
            # 右侧面 (+X, M1 动量轮)
            ([1, 2, 6, 5], (1, 0, 0), QColor(13, 148, 136), f"M1: {self.encoder_angles[0]:05.1f}°", 0),
            # 左侧面 (-X)
            ([0, 4, 7, 3], (-1, 0, 0), QColor(71, 85, 105), "LEFT", None),
            # 前侧面 (+Y, M2 动量轮)
            ([2, 3, 7, 6], (0, 1, 0), QColor(217, 119, 6), f"M2: {self.encoder_angles[1]:05.1f}°", 1),
            # 后侧面 (-Y)
            ([0, 1, 5, 4], (0, -1, 0), QColor(100, 116, 139), "BACK", None),
        ]

        # 4. 计算每个面的平均相机景深 (Z) 并按深度排序 (画家算法: 远 -> 近)
        face_render_list = []
        for face_idx, (v_idx, n_local, base_color, label, wheel_idx) in enumerate(faces):
            avg_z = sum(verts_cam[i][2] for i in v_idx) / 4.0

            # 变换面法线计算光照
            n_cam = self._transform_point(n_local)
            origin_cam = self._transform_point((0, 0, 0))
            norm = (n_cam[0] - origin_cam[0], n_cam[1] - origin_cam[1], n_cam[2] - origin_cam[2])
            length = math.sqrt(norm[0]**2 + norm[1]**2 + norm[2]**2)
            if length > 0.001:
                norm = (norm[0]/length, norm[1]/length, norm[2]/length)

            # 光源方向 (右上斜向朝场景入射)
            light = (0.35, 0.65, -0.65)
            l_len = math.sqrt(light[0]**2 + light[1]**2 + light[2]**2)
            light = (light[0]/l_len, light[1]/l_len, light[2]/l_len)

            dot = norm[0] * light[0] + norm[1] * light[1] + norm[2] * light[2]
            intensity = max(0.35, min(1.0, 0.55 + dot * 0.45))

            face_render_list.append({
                'avg_z': avg_z,
                'v_idx': v_idx,
                'color': base_color,
                'intensity': intensity,
                'label': label,
                'wheel_idx': wheel_idx,
                'norm_z': norm[2]
            })

        # 按相机 Z 降序 (远 -> 近)
        face_render_list.sort(key=lambda item: item['avg_z'], reverse=True)

        # 5. 绘制各个面
        for item in face_render_list:
            v_idx = item['v_idx']
            pts = [verts_2d[i] for i in v_idx]
            poly = QPolygonF(pts)

            # 颜色明暗调制
            c = item['color']
            shaded_color = QColor(
                int(c.red() * item['intensity']),
                int(c.green() * item['intensity']),
                int(c.blue() * item['intensity']),
                230
            )

            # 填充多边形 (浅色背景下描绘利落的灰色边框)
            painter.setBrush(QBrush(shaded_color))
            painter.setPen(QPen(QColor(51, 65, 85, 200), 1.5))
            painter.drawPolygon(poly)

            # 如果朝向观察者 (norm_z < 0.25)，绘制面中心文字与动量轮
            if item['norm_z'] < 0.25:
                fc_x = sum(p.x() for p in pts) / 4.0
                fc_y = sum(p.y() for p in pts) / 4.0

                # 绘制动量轮圆盘 (如果有)
                if item['wheel_idx'] is not None:
                    self._draw_momentum_wheel(painter, pts, fc_x, fc_y, item['wheel_idx'])

                # 绘制面文字标号
                painter.setPen(QColor(255, 255, 255, 230))
                font = QFont("Segoe UI", 8, QFont.Bold)
                painter.setFont(font)
                painter.drawText(int(fc_x - 40), int(fc_y - 8), 80, 16, Qt.AlignCenter, item['label'])

        # 6. 在立方体中心绘制机体姿态 RGB 坐标轴
        self._draw_axes(painter, cx, cy)

        # 7. 绘制左上角 HUD 信息 (欧拉角数值与鼠标提示)
        self._draw_hud(painter, w, h)

    def _draw_ground(self, painter, cx, cy):
        """
        绘制绝对静止的参考水平地面 (World Ground Plane):
        - 位于世界坐标系 Zw = -ground_height 处 (略低于立方体底面)
        - 保持水平静止，不随机体姿态 (Roll/Pitch/Yaw) 发生任何旋转倾斜
        - 鼠标水平拖拽只改变水平环视方位角 (cam_azim)，俯仰观察角固定为 <=30° (20°)
        - 绘制半透明基台、正交参考网格、世界坐标基准指示与机体投影阴影
        """
        s = self.cube_size
        ground_z = -s * 1.35  # 地面固定高度 (约 -101)
        grid_half = s * 2.5   # 地面半边长 (约 188)
        steps = 6
        step_sz = grid_half / steps

        # 1. 绘制地面半透明台面基板 (世界坐标)
        corners_world = [
            (-grid_half, -grid_half, ground_z),
            ( grid_half, -grid_half, ground_z),
            ( grid_half,  grid_half, ground_z),
            (-grid_half,  grid_half, ground_z),
        ]
        corners_2d = []
        for pt_w in corners_world:
            p_cam = self._world_to_cam(pt_w)
            p_2d, _ = self._project(p_cam, cx, cy)
            corners_2d.append(p_2d)

        ground_poly = QPolygonF(corners_2d)
        painter.setBrush(QBrush(QColor(241, 245, 249, 220)))
        painter.setPen(QPen(QColor(203, 213, 225, 200), 1.2))
        painter.drawPolygon(ground_poly)

        # 2. 绘制立方体在静止地面上的柔和投影阴影 (增强空间高度感与着地感)
        center_ground_cam = self._world_to_cam((0, 0, ground_z))
        cg_2d, _ = self._project(center_ground_cam, cx, cy)

        painter.setPen(Qt.NoPen)
        shadow_grad = QRadialGradient(cg_2d.x(), cg_2d.y(), s * 1.2)
        shadow_grad.setColorAt(0.0, QColor(148, 163, 184, 110))
        shadow_grad.setColorAt(0.65, QColor(203, 213, 225, 50))
        shadow_grad.setColorAt(1.0, QColor(241, 245, 249, 0))
        painter.setBrush(QBrush(shadow_grad))
        painter.drawEllipse(cg_2d, s * 0.95, s * 0.50)

        # 3. 绘制地面正交参考网格 (仅由世界坐标投射，绝不随机体姿态倾斜)
        pen_grid = QPen(QColor(203, 213, 225, 180), 1, Qt.SolidLine)
        pen_axis = QPen(QColor(148, 163, 184, 230), 1.4, Qt.SolidLine)

        for i in range(-steps, steps + 1):
            is_center = (i == 0)
            painter.setPen(pen_axis if is_center else pen_grid)

            # 平行于 Y_world 轴的纵向网格线
            p1_cam = self._world_to_cam((i * step_sz, -grid_half, ground_z))
            p2_cam = self._world_to_cam((i * step_sz,  grid_half, ground_z))
            p1_2d, _ = self._project(p1_cam, cx, cy)
            p2_2d, _ = self._project(p2_cam, cx, cy)
            painter.drawLine(p1_2d, p2_2d)

            # 平行于 X_world 轴的横向网格线
            p3_cam = self._world_to_cam((-grid_half, i * step_sz, ground_z))
            p4_cam = self._world_to_cam(( grid_half, i * step_sz, ground_z))
            p3_2d, _ = self._project(p3_cam, cx, cy)
            p4_2d, _ = self._project(p4_cam, cx, cy)
            painter.drawLine(p3_2d, p4_2d)

        # 4. 地面中心同心基准圆环
        painter.setPen(QPen(QColor(148, 163, 184, 160), 1.0, Qt.DashLine))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(cg_2d, s * 0.40, s * 0.20)

        # 5. 静止地面台角基准文字与世界水平标识
        corner_label_cam = self._world_to_cam((grid_half - 12, -grid_half + 12, ground_z))
        cl_2d, _ = self._project(corner_label_cam, cx, cy)
        painter.setPen(QColor(100, 116, 139))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(cl_2d.x() - 110), int(cl_2d.y() - 14), 120, 14, Qt.AlignRight, "水平静止地面基准")

    def _draw_momentum_wheel(self, painter, pts, fc_x, fc_y, wheel_idx):
        """在面上绘制动量轮圆盘、旋转辐条与 MT6701 角度指针"""
        r1 = math.hypot(pts[0].x() - fc_x, pts[0].y() - fc_y)
        r2 = math.hypot(pts[1].x() - fc_x, pts[1].y() - fc_y)
        r = (r1 + r2) * 0.35

        painter.setPen(QPen(QColor(255, 255, 255, 200), 2))
        painter.setBrush(QBrush(QColor(30, 41, 59, 180)))
        painter.drawEllipse(QPointF(fc_x, fc_y), r, r)

        # 旋转辐条
        ang = self.wheel_angles[wheel_idx]
        painter.setPen(QPen(QColor(251, 191, 36, 220), 1.5))
        for k in range(4):
            spoke_a = ang + k * (math.pi / 2.0)
            sx = fc_x + r * 0.85 * math.cos(spoke_a)
            sy = fc_y + r * 0.85 * math.sin(spoke_a)
            painter.drawLine(QPointF(fc_x, fc_y), QPointF(sx, sy))

        # 红色指针精准指示当前 MT6701 实际角度
        px = fc_x + r * 0.90 * math.cos(ang)
        py = fc_y + r * 0.90 * math.sin(ang)
        painter.setPen(QPen(QColor(239, 68, 68), 2.5))
        painter.drawLine(QPointF(fc_x, fc_y), QPointF(px, py))

    def _draw_axes(self, painter, cx, cy):
        """在中心绘制 X(红), Y(绿), Z(蓝) 空间坐标轴"""
        axis_len = self.cube_size * 1.45
        origin_cam = self._transform_point((0, 0, 0))
        o_2d, _ = self._project(origin_cam, cx, cy)

        axes = [
            ((axis_len, 0, 0), QColor(220, 38, 38), "X"),
            ((0, axis_len, 0), QColor(22, 163, 74), "Y"),
            ((0, 0, axis_len), QColor(37, 99, 235), "Z"),
        ]

        for pt_local, color, name in axes:
            p_cam = self._transform_point(pt_local)
            p_2d, _ = self._project(p_cam, cx, cy)

            painter.setPen(QPen(color, 2))
            painter.drawLine(o_2d, p_2d)

            painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
            painter.drawText(int(p_2d.x() + 4), int(p_2d.y() + 4), name)

    def _draw_hud(self, painter, w, h):
        """绘制左上角姿态 HUD 与操作提示"""
        painter.setFont(QFont("Consolas", 9, QFont.Bold))

        # 姿态数值
        r_deg = self.roll - self.roll_offset
        p_deg = self.pitch - self.pitch_offset
        y_deg = self.yaw - self.yaw_offset

        hud_bg = QColor(255, 255, 255, 220)
        painter.fillRect(10, 10, 240, 106, hud_bg)
        painter.setPen(QPen(QColor(203, 213, 225), 1))
        painter.drawRect(10, 10, 240, 106)

        painter.setPen(QColor(220, 38, 38))
        painter.drawText(18, 28, f"Roll (X) : {r_deg:+.1f} deg")
        painter.setPen(QColor(22, 163, 74))
        painter.drawText(18, 48, f"Pitch(Y) : {p_deg:+.1f} deg")
        painter.setPen(QColor(37, 99, 235))
        painter.drawText(18, 68, f"Yaw  (Z) : {y_deg:+.1f} deg")

        painter.setPen(QColor(2, 132, 199))
        painter.drawText(18, 92, f"MT6701: {self.encoder_angles[0]:05.1f}°|{self.encoder_angles[1]:05.1f}°|{self.encoder_angles[2]:05.1f}°")

        # 底部操作提示 (更新说明：地面保持绝对静止，鼠标仅改变水平环视观察方向，视角为低仰视角 20°)
        painter.setFont(QFont("Segoe UI", 8))
        painter.setPen(QColor(100, 116, 139))
        painter.drawText(10, h - 10, "水平拖拽鼠标改变环视方向 | 固定低仰观察视角(20°) | 地面基准水平静止 | 双击重置视角")


# =========================================================================
# 硬件自检与安全互锁卡片组件 (SelfTestCard)
# =========================================================================
class SelfTestCard(QGroupBox):
    """三轴电机与霍尔磁编码器安全自检与互锁控制面板"""
    sig_start_test = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__("硬件安全检测与互锁状态", parent)
        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #ffffff;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #334155;
            }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)

        # 1. 顶部互锁状态标签
        status_row = QHBoxLayout()
        status_row.setSpacing(6)
        lbl_status_title = QLabel("互锁状态:")
        lbl_status_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #94a3b8;")
        status_row.addWidget(lbl_status_title)

        self.lbl_badge = QLabel("未自检 (电机控制锁定)")
        self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #f59e0b;")
        status_row.addWidget(self.lbl_badge)
        status_row.addStretch()
        layout.addLayout(status_row)

        # 2. 执行检测主按钮 (一键依次自检/对齐 M1/M2/M3)
        self.btn_run_test = QPushButton("开始三轴自检/对齐")
        self.btn_run_test.setToolTip("依次对 M1/M2/M3 执行电角度零位对齐；每个电机会短时转动，全部完成后保持失能。")
        self.btn_run_test.setFixedHeight(28)
        self.btn_run_test.setStyleSheet("""
            QPushButton {
                background-color: #f1f5f9;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                color: #0f172a;
            }
            QPushButton:hover {
                background-color: #e2e8f0;
                border-color: #94a3b8;
                color: #0f172a;
            }
            QPushButton:disabled {
                background-color: #f8fafc;
                border-color: #e2e8f0;
                color: #94a3b8;
            }
        """)
        self.btn_run_test.clicked.connect(self.sig_start_test.emit)
        layout.addWidget(self.btn_run_test)

        # 3. 三轴诊断明细网格
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        grid.setContentsMargins(2, 4, 2, 4)

        self.lbl_m_tags = []
        self.lbl_m_details = []

        motors_meta = [
            ("M1", "TIM1 / I2C1"),
            ("M2", "TIM3 / I2C2"),
            ("M3", "TIM4 / I2C3"),
        ]

        for i, (name, bus) in enumerate(motors_meta):
            lbl_name = QLabel(f"{name} ({bus}):")
            lbl_name.setStyleSheet("font-size: 11px; font-weight: bold; color: #475569;")
            grid.addWidget(lbl_name, i, 0)

            lbl_status = QLabel("待检测")
            lbl_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
            lbl_status.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl_status, i, 1)
            self.lbl_m_tags.append(lbl_status)

            lbl_det = QLabel("通信: -- | 转角: --")
            lbl_det.setStyleSheet("font-size: 11px; color: #64748b;")
            lbl_det.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lbl_det, i, 2)
            self.lbl_m_details.append(lbl_det)

        layout.addLayout(grid)

        # 4. 通道映射信息 (由 $STATE 的 Map/PP 字段刷新)
        self.lbl_maps = QLabel("通道映射: --")
        self.lbl_maps.setWordWrap(True)
        self.lbl_maps.setStyleSheet("font-size: 10px; color: #64748b;")
        layout.addWidget(self.lbl_maps)

        # 5. 底部说明提示标签
        self.lbl_hint = QLabel("点击一次依次完成 M1→M2→M3 电角度对齐；每次仅驱动一个电机，完成后保持失能。")
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")
        layout.addWidget(self.lbl_hint)

    def set_testing_state(self, step_str=""):
        self.btn_run_test.setEnabled(False)
        self.btn_run_test.setText("正在执行三轴检测...")
        msg = f"自检中 ({step_str})" if step_str else "自检中 (三轴依次对齐)"
        self.lbl_badge.setText(msg)
        self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #0369a1;")
        self.lbl_hint.setText("正在依次对 M1/M2/M3 扫电角度并验证编码器位移，完成后自动失能。")
        self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")

    def set_step(self, motor_str):
        try:
            idx = int(motor_str[1]) - 1
            if 0 <= idx < 3:
                self.lbl_m_tags[idx].setText("检测中...")
                self.lbl_m_tags[idx].setStyleSheet("font-size: 11px; font-weight: bold; color: #0369a1;")
                self.lbl_badge.setText(f"正在检测 {motor_str} (低速开环与I2C采样)...")
        except Exception:
            pass

    def _apply_axis_status(self, i, info):
        st = info.get('status', 'FAIL')
        comm = info.get('comm', 0)
        deg = info.get('deg', 0.0)

        if st == 'PASS':
            self.lbl_m_tags[i].setText("正常")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #15803d;")
            self.lbl_m_details[i].setText(f"通信: {comm}% | 转角: {deg:.1f}°")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #15803d;")
        elif st in ('SKIP', 'NONE', 'UNINSTALLED'):
            self.lbl_m_tags[i].setText("未检测")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #64748b;")
            self.lbl_m_details[i].setText("本轮未检测")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #64748b;")
        elif st == 'FAIL_MAP':
            self.lbl_m_tags[i].setText("映射检测失败")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #b45309;")
            self.lbl_m_details[i].setText(f"未找到随该驱动运动的编码器 | 最大位移: {deg:.1f}°")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #b45309;")
        elif st == 'FAIL_ALIGN':
            self.lbl_m_tags[i].setText("对齐失败")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #b45309;")
            self.lbl_m_details[i].setText(f"零位吸合未完成 | 映射转角: {deg:.1f}°")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #b45309;")
        elif "I2C" in st or st in ('OFFLINE', 'ENCODER_OFFLINE'):
            self.lbl_m_tags[i].setText("编码器离线")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #b91c1c;")
            self.lbl_m_details[i].setText(f"状态: {st} | 通信: {comm}%")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #b91c1c;")
        elif st in ('STALL', 'STALLED', 'FAIL_STALL'):
            self.lbl_m_tags[i].setText("堵转")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #b91c1c;")
            self.lbl_m_details[i].setText(f"状态: {st} | 通信: {comm}% | 转角: {deg:.1f}°")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #b91c1c;")
        else:
            self.lbl_m_tags[i].setText("自检异常")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #b91c1c;")
            self.lbl_m_details[i].setText(f"状态: {st} | 通信: {comm}% | 转角: {deg:.1f}°")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #b91c1c;")

    def set_axis_result(self, motor_idx, info):
        """单个轴自检完成时立即刷新该轴状态（motor_idx: 1~3）"""
        if 0 <= motor_idx - 1 < 3:
            self._apply_axis_status(motor_idx - 1, info)

    def set_result(self, report):
        self.btn_run_test.setEnabled(True)
        self.btn_run_test.setText("重新三轴自检")

        for i, key in enumerate(['m1', 'm2', 'm3']):
            self._apply_axis_status(i, report.get(key, {}))

        if report.get('all_pass', False):
            self.lbl_badge.setText("自检通过 (已解锁)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #15803d;")
            self.lbl_hint.setText("三轴电机与编码器自检合格，通道已自动校准，控制功能已解锁。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")
        else:
            self.lbl_badge.setText("自检失败 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #b91c1c;")
            if any(report.get(key, {}).get('status') == 'FAIL_MAP' for key in ('m1', 'm2', 'm3')):
                self.lbl_hint.setText("驱动通道与编码器未能匹配；请看串口 MAP 诊断中的三路位移和通信率。")
            elif any(report.get(key, {}).get('status') == 'FAIL_ALIGN' for key in ('m1', 'm2', 'm3')):
                self.lbl_hint.setText("映射已找到，但该轴未完成电角度零位吸合；这不等同于堵转，请查看串口 ERR ALIGN 日志。")
            else:
                self.lbl_hint.setText("存在自检异常轴；请结合各轴状态码、通信结果和串口日志检查。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")

    def set_axis_maps(self, state):
        """由 $STATE 的 Map/PP 字段刷新通道映射信息"""
        parts = []
        for axis in range(1, 4):
            enc = state.get(f'Map{axis}')
            pp = state.get(f'PP{axis}')
            if enc is not None and pp is not None:
                parts.append(f"M{axis}→ENC{enc}(PP{pp})")
        self.lbl_maps.setText("通道映射: " + " | ".join(parts) if parts else "通道映射: --")

    def reset_state(self, connected=False):
        self.btn_run_test.setEnabled(connected)
        self.btn_run_test.setText("开始三轴自检/对齐")
        if connected:
            self.lbl_badge.setText("待自检 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #f59e0b;")
            self.lbl_hint.setText("通信链路已连接。请先完成三轴自检/对齐，再进行平衡操作。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")
        else:
            self.lbl_badge.setText("未连接 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #64748b;")
            self.lbl_hint.setText("提示: 请先连接通信链路，再执行三轴自检/对齐。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")

        for i in range(3):
            self.lbl_m_tags[i].setText("待检测")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
            self.lbl_m_details[i].setText("通信: -- | 转角: --")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #64748b;")
        if not connected:
            self.lbl_maps.setText("通道映射: --")


# =========================================================================
# 主窗口 (MainWindow)
# =========================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cubli 平衡控制上位机")
        self.resize(1440, 920)
        self.setMinimumSize(1260, 760)

        # 串口后台工作线程
        self.worker = SerialWorker()
        self.worker.sig_connected.connect(self.on_serial_connected)
        self.worker.sig_disconnected.connect(self.on_serial_disconnected)
        self.worker.sig_error.connect(self.on_serial_error)
        self.worker.sig_log_rx.connect(self.on_log_rx)
        self.worker.sig_log_tx.connect(self.on_log_tx)
        self.worker.sig_telemetry.connect(self.on_telemetry_received)
        self.worker.sig_state.connect(self.on_state_received)
        self.worker.sig_lqr.connect(self.on_lqr_received)
        self.worker.sig_test_step.connect(self.on_self_test_step)
        self.worker.sig_test_report.connect(self.on_self_test_report)

        # 硬件自检与安全互锁状态 (True: 自检通过，允许操作; False: 锁定拦截)
        self.self_test_passed = False
        self.self_test_running = False
        self.self_test_queue = []
        self.self_test_results = {}
        self.device_state = None
        self.expected_state = {}
        self.edge_profile_pending = False
        self.edge_profile_ack = False
        self.edge_profile_generation = 0
        self.profile_kind = 'edge'
        self.last_balance_reason = 0
        self.startup_auto_edge = False
        self.startup_edge_phase = 0
        self.sync_directions = False
        self.sync_axes = False
        self.active_motors = (False, False, False)
        # 单点自动寻点：记住最近一次由固件学习到的控制目标（用于重载配置后 seed 恢复）
        self.last_learned_target = None
        self.corner_profile_neutral = False
        self.corner_profile_neutral_expect = {}
        # 限时平衡：BSTART 入位接管后计时运行，到时自动 BAL 0
        self.timed_run_armed = False
        self.timed_run_active = False
        self.timed_run_seconds = 15
        self.timed_run_timer = QTimer()
        self.timed_run_timer.setSingleShot(True)
        self.timed_run_timer.timeout.connect(self._timed_run_stop)

        # 波形历史数据队列 (示波器式平滑滚动缓冲)
        self.MAX_POINTS = 800
        self.history_time = deque(maxlen=self.MAX_POINTS)
        self.history_m1 = deque(maxlen=self.MAX_POINTS)
        self.history_m2 = deque(maxlen=self.MAX_POINTS)
        self.history_m3 = deque(maxlen=self.MAX_POINTS)

        # 闭环实测转速队列 (rad/s)
        self.history_s1 = deque(maxlen=self.MAX_POINTS)
        self.history_s2 = deque(maxlen=self.MAX_POINTS)
        self.history_s3 = deque(maxlen=self.MAX_POINTS)

        # MT6701 霍尔编码器角度历史队列 (0 ~ 360°)
        self.history_h1 = deque(maxlen=self.MAX_POINTS)
        self.history_h2 = deque(maxlen=self.MAX_POINTS)
        self.history_h3 = deque(maxlen=self.MAX_POINTS)

        # IMU 6轴数据
        self.history_gx = deque(maxlen=self.MAX_POINTS)
        self.history_gy = deque(maxlen=self.MAX_POINTS)
        self.history_gz = deque(maxlen=self.MAX_POINTS)
        self.history_ax = deque(maxlen=self.MAX_POINTS)
        self.history_ay = deque(maxlen=self.MAX_POINTS)
        self.history_az = deque(maxlen=self.MAX_POINTS)

        self.start_time = None
        self.tele_fps_counter = 0
        self.tele_last_fps_time = time.time()

        # 波形定时刷新 (10 FPS); telemetry and 3D state remain event-driven.
        self.plot_timer = QTimer()
        self.plot_timer.setInterval(100)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start()

        # 串口扫描定时器
        self.port_scan_timer = QTimer()
        self.port_scan_timer.setInterval(2000)
        self.port_scan_timer.timeout.connect(self.refresh_ports)
        self.port_scan_timer.start()

        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        # 全局浅色现代工控风格 (清爽明亮、高对比度专业质感)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f8fafc;
            }
            QWidget {
                color: #0f172a;
                font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
            }
            QFrame#TopBar {
                background-color: #ffffff;
                border-bottom: 1px solid #e2e8f0;
                padding: 4px 8px;
            }
            QLabel {
                font-size: 12px;
                color: #475569;
            }
            QPushButton {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                padding: 4px 8px;
                color: #334155;
                font-size: 12px;
                min-height: 20px;
            }
            QPushButton:hover {
                background-color: #f1f5f9;
                border-color: #94a3b8;
                color: #0f172a;
            }
            QPushButton:pressed {
                background-color: #e2e8f0;
                border-color: #cbd5e1;
            }
            QPushButton:disabled {
                background-color: #f8fafc;
                border-color: #e2e8f0;
                color: #94a3b8;
            }
            QComboBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                padding: 3px 6px;
                color: #0f172a;
                font-size: 12px;
                min-width: 80px;
                min-height: 20px;
            }
            QComboBox:hover {
                border-color: #94a3b8;
            }
            /* 修复下拉弹层(QAbstractItemView)未设置样式时回落到全局暗色 QPalette,
             * 导致系统深色模式下下拉列表背景暗色、文字不可读的问题 */
            QComboBox QAbstractItemView {
                background-color: #ffffff;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                selection-background-color: #dbeafe;
                selection-color: #0f172a;
                outline: none;
            }
            QComboBox QAbstractItemView::item {
                min-height: 22px;
                padding: 2px 4px;
            }
            QDoubleSpinBox {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                padding: 2px 4px;
                color: #0f172a;
                font-size: 12px;
            }
            QDoubleSpinBox:hover {
                border-color: #94a3b8;
            }
            QLineEdit {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 4px;
                padding: 3px 6px;
                color: #0f172a;
                font-size: 12px;
            }
            QLineEdit:hover {
                border-color: #94a3b8;
            }
            QTextEdit {
                background-color: #ffffff;
                border: 1px solid #e2e8f0;
                border-radius: 4px;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 11px;
                color: #1e293b;
            }
            QStatusBar {
                background-color: #ffffff;
                color: #64748b;
                font-size: 11px;
                border-top: 1px solid #e2e8f0;
            }
            QCheckBox {
                font-size: 12px;
                color: #334155;
                spacing: 6px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
                border: 1px solid #cbd5e1;
                border-radius: 3px;
                background-color: #ffffff;
            }
            QCheckBox::indicator:checked {
                background-color: #2563eb;
                border-color: #1d4ed8;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. 顶部操作栏
        top_bar = self._create_top_bar()
        main_layout.addWidget(top_bar)

        # 2. 中部主分割窗格 (左侧调控面板, 右侧 3D 与波形监控)
        splitter_main = QSplitter(Qt.Horizontal)
        splitter_main.setHandleWidth(4)

        # 左侧控制面板 (带自适应滚动区域)
        left_widget = self._create_left_panel()
        splitter_main.addWidget(left_widget)

        # 右侧图表与 3D 姿态监控面板
        right_widget = self._create_right_panel()
        splitter_main.addWidget(right_widget)

        splitter_main.setSizes([510, 930])
        splitter_main.setStretchFactor(0, 0)
        splitter_main.setStretchFactor(1, 1)
        main_layout.addWidget(splitter_main)

        # 3. 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪 - 请选择通信链路并连接")

        # 初始安全互锁：锁定所有电机致动控制，等待串口连接并执行自检
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=False)

    def _create_top_bar(self):
        bar = QFrame()
        bar.setObjectName("TopBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(6)

        # 标题标签 (素雅高亮中性白，无多余彩色)
        lbl_brand = QLabel("Cubli 联调监控")
        lbl_brand.setStyleSheet("font-size: 14px; font-weight: bold; color: #0f172a; margin-right: 6px;")
        layout.addWidget(lbl_brand)

        # 连接链路选择: 有线串口 或 WiFi(TCP) 无线 (ESP32)
        layout.addWidget(QLabel("链路:"))
        self.combo_link = NoWheelComboBox()
        self.combo_link.addItem("串口", "serial")
        self.combo_link.addItem("WiFi(TCP)", "wifi")
        self.combo_link.setMinimumWidth(90)
        self.combo_link.currentIndexChanged.connect(self._on_link_mode_changed)
        layout.addWidget(self.combo_link)

        # WiFi 模式下的 ESP32 地址输入 (AP 热点直连默认 192.168.4.1)
        self.edit_wifi_ip = QLineEdit("192.168.4.1")
        self.edit_wifi_ip.setMaximumWidth(130)
        self.edit_wifi_ip.setToolTip(
            "ESP32 的 IP 地址\nAP 热点模式默认: 192.168.4.1\nSTA 模式: 查看 ESP32 USB 串口启动日志")
        self.edit_wifi_ip.setVisible(False)
        layout.addWidget(self.edit_wifi_ip)

        # 串口选择
        self.lbl_port = QLabel("串口:")
        layout.addWidget(self.lbl_port)
        self.combo_ports = NoWheelComboBox()
        self.combo_ports.setMinimumWidth(130)
        layout.addWidget(self.combo_ports)

        btn_refresh = QPushButton("刷新")
        btn_refresh.setFixedHeight(26)
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)
        self.btn_refresh = btn_refresh   # 保存引用供链路模式切换可见性使用

        # 波特率选择
        self.lbl_baud = QLabel("波特率:")
        layout.addWidget(self.lbl_baud)
        self.combo_baud = NoWheelComboBox()
        for baud in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
            self.combo_baud.addItem(str(baud), baud)
        self.combo_baud.setCurrentText("115200")
        self.combo_baud.setMinimumWidth(80)
        layout.addWidget(self.combo_baud)

        # 打开/关闭按钮 (素雅工控风)
        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.setFixedHeight(26)
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #f1f5f9;
                border: 1px solid #cbd5e1;
                font-weight: bold;
                color: #0f172a;
                padding: 3px 10px;
            }
            QPushButton:hover {
                background-color: #e2e8f0;
                border-color: #94a3b8;
            }
        """)
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)

        # 状态指示标签 (纯文本素雅显示)
        self.lbl_status_led = QLabel("未连接")
        self.lbl_status_led.setStyleSheet("color: #64748b; font-weight: bold; margin-left: 4px;")
        layout.addWidget(self.lbl_status_led)

        layout.addStretch()

        # 全部归零 (平稳停止)
        self.btn_stop_all = QPushButton("停止并失能")
        self.btn_stop_all.setFixedHeight(26)
        self.btn_stop_all.setStyleSheet("background-color: #fef2f2; border: 1px solid #fca5a5; color: #b91c1c; font-weight: bold;")
        self.btn_stop_all.clicked.connect(self.action_stop_all)
        layout.addWidget(self.btn_stop_all)

        # 紧急停止按钮 (素雅工控暗红警示，避免刺眼彩色外框)
        self.btn_estop = QPushButton("紧急停止")
        self.btn_estop.setFixedHeight(26)
        self.btn_estop.setToolTip("立即切断三路驱动器使能")
        self.btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #991b1b;
                border: 1px solid #b91c1c;
                font-weight: bold;
                font-size: 12px;
                padding: 3px 10px;
                border-radius: 4px;
                color: #ffffff;
            }
            QPushButton:hover {
                background-color: #b91c1c;
                border-color: #dc2626;
            }
            QPushButton:pressed {
                background-color: #7f1d1d;
            }
        """)
        self.btn_estop.clicked.connect(self.action_emergency_stop)
        layout.addWidget(self.btn_estop)

        return bar

    def _create_left_panel(self):
        panel = QWidget()
        panel.setObjectName("LeftPanelContent")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 1. 电机三轴自检/对齐与安全互锁卡片 (处于最顶端，三轴对齐后方可进行平衡操作)
        self.card_self_test = SelfTestCard()
        self.card_self_test.sig_start_test.connect(self.action_start_self_test)
        layout.addWidget(self.card_self_test)

        # 2. 平衡控制卡片 (上位机仅保留平衡模式)
        layout.addWidget(self._create_mode_panel())

        # 3. 参数配置与驱动限制 (平衡相关的最小参数集)
        self.param_group = QGroupBox("参数配置与驱动限制")
        self.param_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #ffffff;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #334155;
            }
        """)
        param_layout = QGridLayout(self.param_group)
        param_layout.setVerticalSpacing(8)
        param_layout.setHorizontalSpacing(8)
        param_layout.setContentsMargins(10, 12, 10, 10)

        # 相电压限制 Vq limit (平衡整定常用 6V)
        lbl_vq = QLabel("相电压限制 (Vq):")
        self.spin_vq = QDoubleSpinBox()
        self.spin_vq.setRange(0.5, 6.8)
        self.spin_vq.setSingleStep(0.1)
        self.spin_vq.setValue(5.0)
        self.spin_vq.setSuffix(" V")
        self.spin_vq.setFixedHeight(26)
        self.btn_set_vq = QPushButton("设定Vq")
        self.btn_set_vq.setFixedHeight(26)
        self.btn_set_vq.clicked.connect(self.action_set_voltage_limit)

        param_layout.addWidget(lbl_vq, 0, 0)
        param_layout.addWidget(self.spin_vq, 0, 1)
        param_layout.addWidget(self.btn_set_vq, 0, 2)

        # 速度上限 Limit (必须高于满速保护阈值)
        lbl_vlim = QLabel("极速限制 Limit:")
        self.spin_vlim = QDoubleSpinBox()
        self.spin_vlim.setRange(1.0, 360.0)
        self.spin_vlim.setSingleStep(5.0)
        self.spin_vlim.setValue(60.0)
        self.spin_vlim.setSuffix(" rad/s")
        self.spin_vlim.setFixedHeight(26)
        self.btn_set_vlim = QPushButton("设定Limit")
        self.btn_set_vlim.setFixedHeight(26)
        self.btn_set_vlim.clicked.connect(self.action_set_velocity_limit)

        param_layout.addWidget(lbl_vlim, 1, 0)
        param_layout.addWidget(self.spin_vlim, 1, 1)
        param_layout.addWidget(self.btn_set_vlim, 1, 2)

        # 遥测输出与状态查询
        self.chk_tele = QCheckBox("开启固件遥测流 ($TELE)")
        self.chk_tele.setChecked(True)
        self.chk_tele.toggled.connect(self.action_toggle_tele)
        btn_query = QPushButton("查询状态 (?)")
        btn_query.setFixedHeight(26)
        btn_query.clicked.connect(lambda: self.worker.send_cmd("?"))

        param_layout.addWidget(self.chk_tele, 2, 0, 1, 2)
        param_layout.addWidget(btn_query, 2, 2)

        btn_query_hall = QPushButton("查询编码器(HALL)")
        btn_query_hall.setFixedHeight(26)
        btn_query_hall.clicked.connect(lambda: self.worker.send_cmd("HALL"))
        param_layout.addWidget(btn_query_hall, 3, 0, 1, 3)

        layout.addWidget(self.param_group)
        layout.addStretch()

        # 使用 QScrollArea 容器包裹左侧控制面板，彻底避免小屏下垂直挤压导致的文字变形与截断
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet("""
            QScrollArea {
                background-color: transparent;
                border: none;
            }
            QWidget#LeftPanelContent {
                background-color: #f8fafc;
            }
            QScrollBar:vertical {
                background: #f1f5f9;
                width: 8px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #cbd5e1;
                min-height: 24px;
                border-radius: 4px;
            }
            QScrollBar::handle:vertical:hover {
                background: #94a3b8;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        scroll.setWidget(panel)

        return scroll

    def _create_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 1. 顶部指标栏 (采用 2行x3列 网格布局，彻底解决单行 6 卡片文字挤压截断问题)
        status_card_bar = QGridLayout()
        status_card_bar.setSpacing(6)
        status_card_bar.setContentsMargins(0, 0, 0, 0)

        self.card_stat_m1 = self._create_metric_box("M1 飞轮转速", "0.0 rad/s")
        self.card_stat_m2 = self._create_metric_box("M2 飞轮转速", "0.0 rad/s")
        self.card_stat_m3 = self._create_metric_box("M3 飞轮转速", "0.0 rad/s")
        self.card_stat_hall = self._create_metric_box("MT6701 角度 (H1 / H2 / H3)", "0.0° / 0.0° / 0.0°")
        self.card_stat_gyro = self._create_metric_box("陀螺仪 (Gx / Gy / Gz)", "0.0 / 0.0 / 0.0 dps")
        self.card_stat_fps = self._create_metric_box("通信状态与帧率", "0 FPS")

        status_card_bar.addWidget(self.card_stat_m1['box'], 0, 0)
        status_card_bar.addWidget(self.card_stat_m2['box'], 0, 1)
        status_card_bar.addWidget(self.card_stat_m3['box'], 0, 2)
        status_card_bar.addWidget(self.card_stat_hall['box'], 1, 0)
        status_card_bar.addWidget(self.card_stat_gyro['box'], 1, 1)
        status_card_bar.addWidget(self.card_stat_fps['box'], 1, 2)
        layout.addLayout(status_card_bar)

        # 2. 中部左右/上下布局 (左上 3D 立方体，右上/下方曲线)
        # 上半区: 3D 姿态立方体 (与陀螺仪同步，默认水平朝上)
        cube_group = QGroupBox("姿态与飞轮")
        cube_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dbe3ec;
                border-radius: 6px;
                margin-top: 6px;
                padding-top: 8px;
                background-color: #ffffff;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #475569;
            }
        """)
        cube_layout = QVBoxLayout(cube_group)
        cube_layout.setContentsMargins(8, 8, 8, 8)

        cube_toolbar = QHBoxLayout()
        cube_toolbar.addWidget(QLabel("Roll / Pitch / Yaw"))
        cube_toolbar.addStretch()
        btn_zero_att = QPushButton("姿态归零")
        btn_zero_att.setFixedHeight(25)
        btn_zero_att.clicked.connect(self.action_reset_cube_attitude)
        cube_toolbar.addWidget(btn_zero_att)
        cube_layout.addLayout(cube_toolbar)

        self.cube_widget = Cubli3DWidget()
        cube_layout.addWidget(self.cube_widget)

        # 下半区: 实时动态波形监控选项卡 (默认展示 MT6701 实时角度曲线)
        curve_group = QGroupBox("实时数据")
        curve_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dbe3ec;
                border-radius: 6px;
                margin-top: 4px;
                padding-top: 6px;
                background-color: #ffffff;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #475569;
            }
        """)
        curve_layout = QVBoxLayout(curve_group)
        curve_layout.setContentsMargins(4, 4, 4, 4)

        self.tab_curves = QTabWidget()
        self.tab_curves.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #dbe3ec;
                border-radius: 4px;
                background-color: #ffffff;
                color: #334155;
            }
            QTabBar::tab {
                background-color: #f1f5f9;
                border: 1px solid #dbe3ec;
                padding: 5px 10px;
                margin-right: 2px;
                color: #475569;
                font-weight: bold;
                font-size: 11px;
            }
            QTabBar::tab:selected {
                background-color: #ffffff;
                color: #0f172a;
                border-bottom-color: #ffffff;
            }
        """)

        # Tab 1: MT6701 实时角度波形 (0° ~ 360°)
        self.plot_angle = pg.PlotWidget(title="三轴 MT6701 实时编码器角度 (0° ~ 360°)")
        self.plot_angle.showGrid(x=True, y=True, alpha=0.15)
        self.plot_angle.addLegend(offset=(10, 10))
        self.plot_angle.setLabel('left', 'Angle', units='deg')
        self.plot_angle.setLabel('bottom', 'Time', units='s')
        self.plot_angle.setYRange(0.0, 360.0, padding=0.02)
        self.plot_angle.setXRange(0.0, 10.0, padding=0.0)

        for pos in [90.0, 180.0, 270.0]:
            self.plot_angle.addItem(pg.InfiniteLine(pos=pos, angle=0, pen=pg.mkPen('#cbd5e1', width=1, style=Qt.DashLine)))

        self.curve_h1 = self.plot_angle.plot(pen=pg.mkPen('#00bcd4', width=2.4), name="M1 MT6701 (I2C1)")
        self.curve_h2 = self.plot_angle.plot(pen=pg.mkPen('#ff9800', width=2.4), name="M2 MT6701 (I2C2)")
        self.curve_h3 = self.plot_angle.plot(pen=pg.mkPen('#e91e63', width=2.4), name="M3 MT6701 (I2C3)")
        self.tab_curves.addTab(self.plot_angle, "MT6701 角度波形 (0~360°)")

        # Tab 2: 电机目标与闭环实测转速波形
        self.plot_speed = pg.PlotWidget(title="三轴电机速度闭环监控 (rad/s | 虚线:目标设定, 实线:实测反馈)")
        self.plot_speed.showGrid(x=True, y=True, alpha=0.15)
        self.plot_speed.addLegend(offset=(10, 10))
        self.plot_speed.setLabel('left', 'Speed', units='rad/s')
        self.plot_speed.setLabel('bottom', 'Time', units='s')
        self.plot_speed.setYRange(-65, 65, padding=0.05)
        self.plot_speed.setXRange(0, 10, padding=0.0)
        self.plot_speed.addItem(pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#cbd5e1', width=1.2, style=Qt.DashLine)))

        self.curve_m1_tgt = self.plot_speed.plot(pen=pg.mkPen('#0284c7', width=1.5, style=Qt.DashLine), name="M1 目标")
        self.curve_s1_act = self.plot_speed.plot(pen=pg.mkPen('#0369a1', width=2.4), name="M1 实测")
        self.curve_m2_tgt = self.plot_speed.plot(pen=pg.mkPen('#f59e0b', width=1.5, style=Qt.DashLine), name="M2 目标")
        self.curve_s2_act = self.plot_speed.plot(pen=pg.mkPen('#d97706', width=2.4), name="M2 实测")
        self.curve_m3_tgt = self.plot_speed.plot(pen=pg.mkPen('#e11d48', width=1.5, style=Qt.DashLine), name="M3 目标")
        self.curve_s3_act = self.plot_speed.plot(pen=pg.mkPen('#be123c', width=2.4), name="M3 实测")
        self.tab_curves.addTab(self.plot_speed, "电机转速波形 (rad/s)")

        # Tab 3: 陀螺仪角速度波形
        self.plot_gyro = pg.PlotWidget(title="陀螺仪三轴角速度 (dps = °/s)")
        self.plot_gyro.showGrid(x=True, y=True, alpha=0.15)
        self.plot_gyro.addLegend(offset=(10, 10))
        self.plot_gyro.setLabel('left', 'Angular Velocity', units='dps')
        self.plot_gyro.setLabel('bottom', 'Time', units='s')
        self.plot_gyro.setYRange(-200, 200, padding=0.05)
        self.plot_gyro.setXRange(0, 10, padding=0.0)
        self.curve_gx = self.plot_gyro.plot(pen=pg.mkPen('#00e676', width=2.0), name="Gx (Roll)")
        self.curve_gy = self.plot_gyro.plot(pen=pg.mkPen('#ffeb3b', width=2.0), name="Gy (Pitch)")
        self.curve_gz = self.plot_gyro.plot(pen=pg.mkPen('#38bdf8', width=2.0), name="Gz (Yaw)")
        self.tab_curves.addTab(self.plot_gyro, "陀螺仪角速度 (dps)")

        # Tab 4: 加速度计波形
        self.plot_accel = pg.PlotWidget(title="三轴重力加速度 (g)")
        self.plot_accel.showGrid(x=True, y=True, alpha=0.15)
        self.plot_accel.addLegend(offset=(10, 10))
        self.plot_accel.setLabel('left', 'Acceleration', units='g')
        self.plot_accel.setLabel('bottom', 'Time', units='s')
        self.plot_accel.setYRange(-2.0, 2.0, padding=0.05)
        self.plot_accel.setXRange(0, 10, padding=0.0)
        self.curve_ax = self.plot_accel.plot(pen=pg.mkPen('#ff5252', width=2.0), name="Ax")
        self.curve_ay = self.plot_accel.plot(pen=pg.mkPen('#69f0ae', width=2.0), name="Ay")
        self.curve_az = self.plot_accel.plot(pen=pg.mkPen('#448aff', width=2.0), name="Az")
        self.tab_curves.addTab(self.plot_accel, "三轴加速度 (g)")

        curve_layout.addWidget(self.tab_curves)
        monitor_splitter = QSplitter(Qt.Horizontal)
        monitor_splitter.setHandleWidth(5)
        cube_group.setMinimumWidth(300)
        curve_group.setMinimumWidth(390)
        monitor_splitter.addWidget(cube_group)
        monitor_splitter.addWidget(curve_group)
        monitor_splitter.setSizes([420, 570])
        monitor_splitter.setStretchFactor(0, 0)
        monitor_splitter.setStretchFactor(1, 1)
        layout.addWidget(monitor_splitter, stretch=5)

        # 3. 底部终端原始报文日志
        log_group = QGroupBox("串口通信监控与终端指令")
        log_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                margin-top: 4px;
                padding-top: 6px;
                background-color: #ffffff;
                color: #0f172a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #334155;
            }
        """)
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(6, 6, 6, 6)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.document().setMaximumBlockCount(1000)
        self.txt_log.setMinimumHeight(90)
        log_layout.addWidget(self.txt_log)

        log_ctrl_row = QHBoxLayout()
        log_ctrl_row.setSpacing(8)
        self.chk_autoscroll = QCheckBox("自动滚动")
        self.chk_autoscroll.setChecked(True)
        log_ctrl_row.addWidget(self.chk_autoscroll)

        self.chk_filter_tele = QCheckBox("隐藏遥测帧")
        self.chk_filter_tele.setChecked(True)
        log_ctrl_row.addWidget(self.chk_filter_tele)

        btn_clear_log = QPushButton("清空日志")
        btn_clear_log.setFixedHeight(26)
        btn_clear_log.clicked.connect(self.txt_log.clear)
        log_ctrl_row.addWidget(btn_clear_log)

        log_ctrl_row.addSpacing(10)
        lbl_cmd = QLabel("指令:")
        lbl_cmd.setStyleSheet("font-size: 12px; color: #475569;")
        log_ctrl_row.addWidget(lbl_cmd)
        self.edt_custom_cmd = QLineEdit()
        self.edt_custom_cmd.setFixedHeight(26)
        self.edt_custom_cmd.setPlaceholderText("例如: STATE? / STOP / PID1?")
        self.edt_custom_cmd.returnPressed.connect(self.action_send_custom_cmd)
        log_ctrl_row.addWidget(self.edt_custom_cmd)

        btn_send_cmd = QPushButton("发送")
        btn_send_cmd.setFixedHeight(26)
        btn_send_cmd.clicked.connect(self.action_send_custom_cmd)
        log_ctrl_row.addWidget(btn_send_cmd)

        log_layout.addLayout(log_ctrl_row)
        layout.addWidget(log_group, stretch=1)

        return panel

    def _create_metric_box(self, title, initial_val, color=None):
        """创建紧凑的遥测指标卡片。"""
        box = QFrame()
        box.setStyleSheet("""
            QFrame {
                background-color: #ffffff;
                border: 1px solid #e2e8f0;
                border-radius: 4px;
                padding: 4px 8px;
            }
        """)
        l = QVBoxLayout(box)
        l.setContentsMargins(2, 2, 2, 2)
        l.setSpacing(2)

        lbl_t = QLabel(title)
        lbl_t.setStyleSheet("font-size: 11px; color: #64748b;")

        lbl_v = QLabel(initial_val)
        lbl_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #0f172a;")

        l.addWidget(lbl_t)
        l.addWidget(lbl_v)
        return {'box': box, 'title': lbl_t, 'val': lbl_v}

    # =====================================================================
    # 串口连接控制 (无 emoji)
    # =====================================================================
    def refresh_ports(self):
        current = self.combo_ports.currentText()
        ports = serial.tools.list_ports.comports()
        self.combo_ports.blockSignals(True)
        self.combo_ports.clear()
        found_current = False
        for p in ports:
            display = f"{p.device} ({p.description})"
            self.combo_ports.addItem(display, p.device)
            if p.device == current or display == current:
                found_current = True

        if found_current:
            self.combo_ports.setCurrentText(current)
        elif self.combo_ports.count() > 0:
            self.combo_ports.setCurrentIndex(0)
        self.combo_ports.blockSignals(False)

    def _on_link_mode_changed(self):
        """链路模式切换: 串口 <-> WiFi(TCP), 切换对应输入控件的可见性"""
        is_wifi = (self.combo_link.currentData() == "wifi")
        self.edit_wifi_ip.setVisible(is_wifi)
        self.lbl_port.setVisible(not is_wifi)
        self.combo_ports.setVisible(not is_wifi)
        self.btn_refresh.setVisible(not is_wifi)
        self.lbl_baud.setVisible(not is_wifi)
        self.combo_baud.setVisible(not is_wifi)
        # 未连接时同步更新连接按钮文案
        if not (self.worker.ser and self.worker.ser.is_open):
            self.btn_connect.setText("连接 WiFi" if is_wifi else "打开串口")

    def toggle_connection(self):
        if self.worker.ser and self.worker.ser.is_open:
            self.worker.disconnect_port()
        else:
            if self.combo_link.currentData() == "wifi":
                host = self.edit_wifi_ip.text().strip()
                if not host:
                    QMessageBox.warning(self, "提示", "请输入 ESP32 的 IP 地址！")
                    return
                baud = int(self.combo_baud.currentText())
                self.worker.connect_wifi(host, 3333, baud)
            else:
                port = self.combo_ports.currentData()
                if not port:
                    port = self.combo_ports.currentText().split()[0]
                if not port:
                    QMessageBox.warning(self, "提示", "未检测到可用串口！")
                    return
                baud = int(self.combo_baud.currentText())
                self.worker.connect_port(port, baud)

    def on_serial_connected(self, port, baud):
        self.btn_connect.setText("断开连接")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #fff7ed;
                border: 1px solid #fdba74;
                font-weight: bold;
                padding: 3px 10px;
                color: #c2410c;
            }
            QPushButton:hover {
                background-color: #ffedd5;
                border-color: #fb923c;
            }
        """)
        self.lbl_status_led.setText("已连接")
        connection_detail = port if "(WiFi)" in port else f"{port} · {baud} bps"
        if '(WiFi)' not in port:
            port_index = self.combo_ports.findData(port)
            if port_index >= 0:self.combo_ports.setCurrentIndex(port_index)
            self.combo_baud.setCurrentText(str(baud))
        self.lbl_status_led.setToolTip(connection_detail)
        self.lbl_status_led.setStyleSheet("color: #15803d; font-weight: bold; margin-left: 4px;")
        self.status_bar.showMessage(f"链路已连接: {connection_detail}。控制保持锁定，请先执行硬件自检。")

        # 连接成功后，重置自检状态并锁定电机操作
        self.self_test_passed = False
        self.self_test_running = False
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=True)

        # 重置时间原点，确保曲线从 t=0 开始平稳推进
        self.start_time = time.time()
        self.history_time.clear()
        self.history_m1.clear()
        self.history_m2.clear()
        self.history_m3.clear()
        self.history_s1.clear()
        self.history_s2.clear()
        self.history_s3.clear()
        self.history_h1.clear()
        self.history_h2.clear()
        self.history_h3.clear()
        self.history_gx.clear()
        self.history_gy.clear()
        self.history_gz.clear()
        self.history_ax.clear()
        self.history_ay.clear()
        self.history_az.clear()

        startup_requested = self.startup_auto_edge
        self._cancel_motion()
        self.startup_auto_edge = startup_requested
        self.worker.send_cmd("APP 0")
        self.worker.send_cmd("STATE?")

    def on_serial_disconnected(self):
        is_wifi = self.combo_link.currentData() == "wifi"
        self.btn_connect.setText("连接 WiFi" if is_wifi else "打开串口")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                font-weight: bold;
                padding: 3px 10px;
                color: #334155;
            }
            QPushButton:hover {
                background-color: #f1f5f9;
                border-color: #94a3b8;
            }
        """)
        self.lbl_status_led.setText("未连接")
        self.lbl_status_led.setToolTip("")
        self.lbl_status_led.setStyleSheet("color: #64748b; font-weight: bold; margin-left: 4px;")
        self.status_bar.showMessage("链路已断开")

        self._cancel_motion()
        self.device_state = None
        self.active_motors = (False, False, False)
        self.expected_state = {}
        self.lbl_balance.setText("链路已断开，等待设备心跳超时失能")
        # 断开连接时，恢复互锁锁定状态
        self.self_test_passed = False
        self.self_test_running = False
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=False)

    def on_serial_error(self, err_msg):
        self.status_bar.showMessage(f"错误: {err_msg}")
        self.append_log(f"<span style='color: #e74c3c;'>[ERROR] {err_msg}</span>")

    # =====================================================================
    # 硬件自检与安全互锁机制
    # =====================================================================
    def _cancel_motion(self):
        self.startup_auto_edge = False
        self.edge_profile_pending = False
        self.edge_profile_ack = False
        self.timed_run_timer.stop()
        self.timed_run_armed = False
        self.timed_run_active = False
        if self.self_test_running:
            self.self_test_running = False
            self.self_test_queue = []
            self.card_self_test.reset_state(connected=bool(self.device_state))
        self.worker.clear_pending()

    def _request_state_change(self, command, expected=None):
        self._cancel_motion()
        self.expected_state = expected or {}
        self.set_motor_controls_locked(True)
        # 上位机仅保留平衡操作：自检/对齐自动切到调试模式(APP 0)，平衡启动自动切到工作模式(APP 1)
        first = command.split(' ', 1)[0].upper()
        if first in ('TEST', 'ALIGN', 'CALIB', 'CHECK'):
            need = 0
        elif first in ('BAL', 'BSTART', 'BAUTO'):
            need = 1
        else:
            need = None
        if need is not None and (self.device_state or {}).get('App') != need:
            self.worker.send_cmd(f'APP {need}')
        self.worker.send_cmd(command)

    def set_motor_controls_locked(self, locked: bool, active_motors=None):
        state = self.device_state or {}
        locked = locked or not self.device_state
        if active_motors is not None:
            self.active_motors = active_motors
        self.btn_set_vq.setEnabled(not locked)
        self.btn_set_vlim.setEnabled(not locked)
        self.btn_stop_all.setEnabled(True)
        connected = bool(self.worker.is_running and self.device_state)
        busy = self.self_test_running or bool(self.expected_state) or self.edge_profile_pending
        self.mode_group.setEnabled(connected and not self.self_test_running)
        needed = self.active_motors if state.get('BMode') == 4 else (self.active_motors[state.get('BMode', 1)-1],)
        self.btn_balance_start.setEnabled(connected and not busy and
            state.get('Bal', 0) in (0, 4) and bool(state.get('Cal')) and bool(state.get('IMU')) and bool(state.get('Ref', 1)) and bool(state.get('AxesOK', 1)) and bool(state.get('DirOK', 1)) and all(needed))
        self.btn_balance_countdown.setEnabled(connected and not busy and
            (state.get('BMode', 1) != 4 or state.get('CornerStart') == 1) and state.get('Bal', 0) in (0, 4) and
            bool(state.get('Cal')) and bool(state.get('IMU')) and bool(state.get('Ref', 0)) and bool(state.get('AxesOK', 0)) and bool(state.get('DirOK', 0)) and all(needed))
        self.btn_balance_countdown.setText('等待0.5°内入位，2秒倒计时后启动（单点）' if state.get('BMode') == 4 else '等待入位，8秒倒计时后启动（边沿）')
        self.btn_balance_countdown.setToolTip('单点需三轴对齐、映射与方向确认，并采集参考；PG5倒计时，PG6亮即松手。离位取消倒计时。'
            if state.get('BMode') == 4 and state.get('CornerStart') == 1 else
            ('当前固件不支持单点灯光倒计时，请更新固件' if state.get('BMode') == 4 else
             '先采集目标姿态；入位后PG5倒计时，离位取消；PG6亮即松手。'))
        can_test = connected and not busy and state.get('Bal', 0) in (0, 4)
        self.card_self_test.btn_run_test.setEnabled(can_test)
        self.btn_trip_angle.setEnabled(connected and not busy and state.get('Bal', 0) in (0, 4))
        idle = state.get('Bal', 0) in (0, 4) and state.get('EN') == '000' and not self.edge_profile_pending
        common = []
        if not connected:common.append('先连接上位机并等待设备状态')
        if self.self_test_running:common.append('三轴硬件自检进行中，请等待')
        if self.edge_profile_pending:common.append('配置正在加载，等待设备回读')
        if state.get('Bal') == 1:common.append('IMU正在校准，请保持机体静止')
        elif not idle:common.append('先停止当前控制，保持电机失能')
        if self.expected_state:common.append('等待设备确认上一条操作')
        if state.get('Fault',0):common.append('先排除驱动故障')
        if 'Auto' not in state:common.append('需要支持自动平衡的固件')
        load_blockers = common.copy()
        if not self.active_motors[0]:load_blockers.append('先完成三轴自检/对齐')
        if not state.get('Cal'):load_blockers.append('先点击“静止校准IMU”')
        if not state.get('IMU'):load_blockers.append('等待IMU恢复有效数据')
        corner_load_blockers = common.copy()
        if not all(self.active_motors):corner_load_blockers.append('先完成三轴硬件自检')
        if not state.get('CornerStart'):corner_load_blockers.append('固件需支持顶点倒计时（请更新）')
        if not state.get('Cal'):corner_load_blockers.append('先点击“静止校准IMU”')
        if not state.get('IMU'):corner_load_blockers.append('等待IMU恢复有效数据')
        start_blockers = common.copy()
        if not all(needed):start_blockers.append('先完成三轴自检/对齐')
        if not state.get('Cal'):start_blockers.append('先点击“静止校准IMU”')
        if not state.get('IMU'):start_blockers.append('等待IMU恢复有效数据')
        if state.get('BMode',4)>4 or not all(state.get(k) for k in ('Ref','AxesOK','DirOK')):
            start_blockers.append('先加载已验证单点配置' if state.get('BMode')==4 else '先加载已验证M1配置，恢复边沿、方向和参考')
        if state.get('BMode')==4 and not state.get('CornerStart'):
            start_blockers.append('固件需支持顶点倒计时（请更新）')
        if float(state.get('SLimit',0))<=float(state.get('FullSpeed',250)):
            start_blockers.append('速度上限需高于满速阈值，可加载已验证配置恢复')
        self.btn_auto_edge.setEnabled(connected and not self.self_test_running and
            (bool(state.get('Auto')) or not start_blockers))
        corner = state.get('BMode') == 4
        self.btn_auto_edge.setText(('停止自动单点并失能' if corner else '停止自动单边并失能') if state.get('Auto') else
                                   ('启动自动单点（无固定时长）' if corner else '启动自动单边（无固定时长）'))
        self.btn_edge_profile.setEnabled(not load_blockers)
        self.btn_corner_profile.setEnabled(not corner_load_blockers)
        self.btn_three_axis_test.setEnabled(can_test)
        self.btn_edge_imu_cal.setEnabled(connected and idle and not self.self_test_running and not self.expected_state and bool(state.get('IMU')))
        load_hint = (corner_load_blockers[0] if corner_load_blockers else '可加载已验证单点配置（保持失能）') if corner \
            else (load_blockers[0] if load_blockers else '可加载已验证M1配置（保持失能）')
        start_hint = '自动已启用，按钮用于停止' if state.get('Auto') else (start_blockers[0] if start_blockers else
                     ('准备完成，可启动自动单点检测' if corner else '准备完成，可启动自动单边检测'))
        timed_hint = ('限时运行：已接管，计时结束后自动失能' if self.timed_run_active else
                      '限时运行：等待入位接管后开始计时，PG6亮即松手' if self.timed_run_armed else None)
        self.lbl_edge_setup.setText(f'配置准备：{load_hint}\n自动启动：{start_hint}' +
            (f'\n{timed_hint}' if timed_hint else ''))
        self.btn_edge_profile.setToolTip('；'.join(load_blockers) if load_blockers else '恢复当前安装的49°参考和已验证参数，电机保持失能')
        self.btn_corner_profile.setToolTip('；'.join(corner_load_blockers) if corner_load_blockers else '恢复当前安装的顶点参考（23.44°/50°）和已验证参数，电机保持失能')
        self.btn_auto_edge.setToolTip('；'.join(start_blockers) if start_blockers and not state.get('Auto') else
            ('PG6亮即松手；超过20°或持续满速3秒后失能等待下次入位（单点三轴）' if corner else 'PG6亮即松手；超过20°或持续满速3秒后失能等待下次入位'))
        self.btn_timed_run.setEnabled(connected and not self.self_test_running and not self.timed_run_armed and
            (bool(state.get('Auto')) or not start_blockers))
        self.btn_timed_run.setToolTip('；'.join(start_blockers) if start_blockers and not state.get('Auto') else
            ('接管后计时运行，到时自动停止并失能；寻点学习保留，可连续多次运行（单点三轴）' if corner else '接管后计时运行，到时自动停止并失能'))
        self.btn_full_speed.setEnabled(connected and idle and not self.self_test_running and 'Auto' in state)

    def _create_mode_panel(self):
        self.mode_group = QGroupBox("平衡控制")
        outer = QVBoxLayout(self.mode_group)
        layout = QGridLayout()
        outer.addLayout(layout)
        btn_reset = QPushButton("复位安全停机锁存（保持失能）")
        btn_reset.setToolTip("清除安全停机/急停锁存并保持电机失能；停止或急停后如无法启动，可先点此复位")
        btn_reset.clicked.connect(self.action_reset_latch)
        layout.addWidget(btn_reset, 0, 0, 1, 2)
        self.balance_config = QWidget()
        outer.addWidget(self.balance_config)
        layout = QGridLayout(self.balance_config)
        layout.setContentsMargins(0, 0, 0, 0)
        self.combo_balance = NoWheelComboBox()
        self.combo_balance.addItems(["边沿 M1", "边沿 M2", "边沿 M3", "顶点（三轴）"])
        self.combo_balance.activated.connect(lambda i: self._request_state_change(f"BMODE {i+1}", {'BMode': i+1}))
        layout.addWidget(self.combo_balance, 2, 0, 1, 2)
        btn_cal = QPushButton("静止校准 IMU")
        btn_cal.clicked.connect(lambda: self._request_state_change("CAL_IMU"))
        layout.addWidget(btn_cal, 3, 0)
        btn_ref = QPushButton("采集当前平衡姿态")
        btn_ref.setToolTip("扶住 Cubli 放在实际平衡点，静止后采集；操作会先失能电机")
        btn_ref.clicked.connect(lambda: self._request_state_change("BZERO"))
        layout.addWidget(btn_ref, 3, 1)
        self.btn_balance_start = QPushButton("启动平衡")
        self.btn_balance_start.clicked.connect(lambda: self._request_state_change("BAL 1"))
        layout.addWidget(self.btn_balance_start, 4, 0)
        btn_off = QPushButton("停止平衡")
        btn_off.clicked.connect(lambda: self._request_state_change("BAL 0"))
        layout.addWidget(btn_off, 4, 1)
        self.lbl_balance = QLabel("未连接，电机失能")
        self.lbl_balance.setWordWrap(True)
        layout.addWidget(self.lbl_balance, 5, 0, 1, 2)
        self.combo_gain_axis = NoWheelComboBox()
        self.combo_gain_axis.addItems(["平衡参数 M1", "平衡参数 M2", "平衡参数 M3"])
        layout.addWidget(self.combo_gain_axis, 6, 0, 1, 2)
        self.balance_spins = []
        for row, (label, value, upper) in enumerate((("Kθ", 32.0, 200), ("Kω机体", 2.4, 50), ("Kω飞轮", .08, 5), ("Ki飞轮", .008, 5)), 7):
            spin = NoWheelDoubleSpinBox()
            spin.setRange(-5 if row >= 9 else 0, upper)
            spin.setDecimals(4)
            spin.setValue(value)
            self.balance_spins.append(spin)
            layout.addWidget(QLabel(label), row, 0)
            layout.addWidget(spin, row, 1)
        btn_gain = QPushButton("下发平衡参数（先失能）")
        btn_gain.clicked.connect(self.action_send_lqr)
        layout.addWidget(btn_gain, 11, 0)
        btn_read = QPushButton("读取平衡参数")
        btn_read.clicked.connect(lambda: self.worker.send_cmd(f"LQR{self.combo_gain_axis.currentIndex()+1}?"))
        layout.addWidget(btn_read, 11, 1)
        self.axis_combos = []
        for axis in range(3):
            combo = NoWheelComboBox()
            combo.addItems([f'M{axis+1}轮轴 → IMU X', f'M{axis+1}轮轴 → IMU Y', f'M{axis+1}轮轴 → IMU Z'])
            combo.setCurrentIndex(axis)
            self.axis_combos.append(combo)
            layout.addWidget(combo, 12+axis, 0, 1, 2)
        btn_axes = QPushButton('确认轮轴对应（先失能，需重采姿态）')
        btn_axes.clicked.connect(self.action_send_axes)
        layout.addWidget(btn_axes, 15, 0, 1, 2)
        self.direction_combos = []
        for axis in range(3):
            combo = NoWheelComboBox()
            combo.addItems([f"M{axis+1} 机架方向 +1", f"M{axis+1} 机架方向 -1"])
            self.direction_combos.append(combo)
            layout.addWidget(combo, 16+axis, 0, 1, 2)
        btn_dir = QPushButton("设置机架方向（先失能）")
        btn_dir.clicked.connect(self.action_send_direction)
        layout.addWidget(btn_dir, 19, 0, 1, 2)
        self.btn_balance_countdown = QPushButton("等待入位，8秒倒计时后启动（边沿）")
        self.btn_balance_countdown.setToolTip("先采集目标姿态；±10°内PG5倒计时，离位取消。PG8闪烁表示需靠近并稳住，常亮表示约±3°且低角速度。PG6运行，PG7跌倒停机。")
        self.btn_balance_countdown.clicked.connect(lambda: self._request_state_change("BSTART 2" if (self.device_state or {}).get('BMode') == 4 else "BSTART 8"))
        layout.addWidget(self.btn_balance_countdown, 20, 0, 1, 2)
        btn_axis_one = QPushButton("只确认当前边沿轮轴")
        btn_axis_one.clicked.connect(self.action_send_edge_axis)
        layout.addWidget(btn_axis_one, 21, 0)
        btn_dir_one = QPushButton("只确认当前边沿方向")
        btn_dir_one.clicked.connect(self.action_send_edge_direction)
        layout.addWidget(btn_dir_one, 21, 1)
        self.spin_trip_angle = NoWheelDoubleSpinBox()
        self.spin_trip_angle.setRange(10,60);self.spin_trip_angle.setValue(20);self.spin_trip_angle.setSuffix(' °')
        layout.addWidget(QLabel('姿态停机角度'),22,0);layout.addWidget(self.spin_trip_angle,22,1)
        self.btn_trip_angle = QPushButton('设置停机角度（先失能）')
        self.btn_trip_angle.clicked.connect(self.action_set_trip_angle)
        layout.addWidget(self.btn_trip_angle,23,0,1,2)
        self.auto_edge_group = QGroupBox('单边/单点平衡检测与自动运行')
        outer.insertWidget(1,self.auto_edge_group)
        auto_layout = QGridLayout(self.auto_edge_group)
        self.lbl_edge_detect = QLabel('单边检测：等待设备状态')
        self.lbl_edge_detect.setWordWrap(True)
        auto_layout.addWidget(self.lbl_edge_detect,0,0,1,2)
        self.lbl_edge_setup = QLabel('先连接设备，再完成三轴自检与静止校准')
        self.lbl_edge_setup.setWordWrap(True)
        self.lbl_edge_setup.setStyleSheet('color: #b45309;')
        auto_layout.addWidget(self.lbl_edge_setup,1,0,1,2)
        self.btn_three_axis_test = QPushButton('三轴自检/对齐')
        self.btn_three_axis_test.setToolTip('依次检测 M1/M2/M3 并完成电角度对齐，电机会短时转动；完成后保持失能')
        self.btn_three_axis_test.clicked.connect(self.action_start_self_test)
        self.btn_edge_imu_cal = QPushButton('静止校准IMU')
        self.btn_edge_imu_cal.setToolTip('机体静止后点击，校准期间保持不动；电机保持失能')
        self.btn_edge_imu_cal.clicked.connect(self.action_prepare_edge_imu_cal)
        auto_layout.addWidget(self.btn_three_axis_test,2,0);auto_layout.addWidget(self.btn_edge_imu_cal,2,1)
        self.btn_edge_profile = QPushButton('加载已验证 M1 配置（失能）')
        self.btn_edge_profile.setToolTip('M1已对齐且IMU已静止校准后可加载。49°参考仅适用于当前安装与支撑棱；参数80/5/-0.045/0，6V。')
        self.btn_edge_profile.clicked.connect(self.action_load_edge_profile)
        auto_layout.addWidget(self.btn_edge_profile,3,0)
        self.btn_corner_profile = QPushButton('加载已验证单点配置（失能）')
        self.btn_corner_profile.setToolTip('三轴自检完成且IMU静止校准后可加载。参考姿态仅适用于当前安装的顶点支撑角；参数80/5/-0.04/0，BYAW 6，姿态τ0.25s，BCAP 1°，6V；已含实机验证的自动寻点（默认开启）。')
        self.btn_corner_profile.clicked.connect(self.action_load_corner_profile)
        auto_layout.addWidget(self.btn_corner_profile,3,1)
        self.spin_full_speed = NoWheelDoubleSpinBox()
        self.spin_full_speed.setRange(10,350);self.spin_full_speed.setValue(250);self.spin_full_speed.setSuffix(' rad/s')
        auto_layout.addWidget(QLabel('满速保护（持续3秒）'),4,0);auto_layout.addWidget(self.spin_full_speed,4,1)
        self.btn_full_speed = QPushButton('设置满速阈值（先失能）')
        self.btn_full_speed.clicked.connect(self.action_set_full_speed)
        auto_layout.addWidget(self.btn_full_speed,5,0,1,2)
        # 单点有界自动寻点 (BNEUTRAL / NTune3)：目标沿倾斜平面按轮速缓慢修正，逼近真实重心
        self.chk_neutral = QCheckBox('启用自动寻点（单点）')
        self.chk_neutral.setChecked(True)
        self.chk_neutral.setToolTip('加载单点配置时下发 BNEUTRAL 1：目标在参考姿态附近按飞轮轮速慢速修正（总范围2~5°），缓慢逼近真实重心；要求 Ki=0、BTRIM=0')
        auto_layout.addWidget(self.chk_neutral,6,0)
        self.chk_neutral_seed = QCheckBox('重载配置时恢复学习目标')
        self.chk_neutral_seed.setChecked(True)
        self.chk_neutral_seed.setToolTip('把最近一次学习到的控制目标作为 seed 随 BNEUTRAL 恢复；仅在总范围以内时生效，超出则从参考锚点重新开始')
        auto_layout.addWidget(self.chk_neutral_seed,6,1)
        neutral_row = QHBoxLayout()
        neutral_row.setSpacing(4)
        self.spin_neutral_gain = NoWheelDoubleSpinBox()
        self.spin_neutral_gain.setRange(0.000005, 0.00025)
        self.spin_neutral_gain.setDecimals(6)
        self.spin_neutral_gain.setSingleStep(0.000005)
        self.spin_neutral_gain.setValue(0.000075)
        self.spin_neutral_gain.setToolTip('寻点增益：轮速超出死区后每 rad/s 的目标移动比例')
        self.spin_neutral_rate = NoWheelDoubleSpinBox()
        self.spin_neutral_rate.setRange(0.01, 0.5)
        self.spin_neutral_rate.setDecimals(2)
        self.spin_neutral_rate.setSingleStep(0.05)
        self.spin_neutral_rate.setValue(0.3)
        self.spin_neutral_rate.setSuffix(' °/s')
        self.spin_neutral_rate.setToolTip('目标最大移动速度（度/秒）')
        self.spin_neutral_limit = NoWheelDoubleSpinBox()
        self.spin_neutral_limit.setRange(2.0, 5.0)
        self.spin_neutral_limit.setDecimals(1)
        self.spin_neutral_limit.setSingleStep(0.5)
        self.spin_neutral_limit.setValue(5.0)
        self.spin_neutral_limit.setSuffix(' °')
        self.spin_neutral_limit.setToolTip('相对原始参考锚点的总寻点范围（2~5°），超出立即夹紧')
        neutral_row.addWidget(QLabel('增益'))
        neutral_row.addWidget(self.spin_neutral_gain)
        neutral_row.addWidget(QLabel('速率'))
        neutral_row.addWidget(self.spin_neutral_rate)
        neutral_row.addWidget(QLabel('范围'))
        neutral_row.addWidget(self.spin_neutral_limit)
        auto_layout.addLayout(neutral_row,7,0,1,2)
        self.lbl_neutral = QLabel('固件寻点: 待加载单点配置')
        self.lbl_neutral.setWordWrap(True)
        self.lbl_neutral.setStyleSheet('font-size: 11px; color: #475569;')
        auto_layout.addWidget(self.lbl_neutral,8,0,1,2)
        # 限时平衡：用于复现验证过的定长试验（入位接管后计时，到时自动失能并保留寻点学习）
        self.spin_timed_run = NoWheelDoubleSpinBox()
        self.spin_timed_run.setRange(2, 300)
        self.spin_timed_run.setDecimals(0)
        self.spin_timed_run.setValue(15)
        self.spin_timed_run.setSuffix(' 秒')
        self.spin_timed_run.setToolTip('入位接管后计时运行时长；到时自动停止并失能（保留寻点学习）')
        auto_layout.addWidget(QLabel('限时运行时长'),9,0)
        auto_layout.addWidget(self.spin_timed_run,9,1)
        self.btn_timed_run = QPushButton('启动限时平衡（到时自动停止）')
        self.btn_timed_run.setToolTip('扶到目标窗口内，PG5倒计时、PG6亮即松手；接管后计时运行指定秒数，到时自动停止并失能。寻点学习保留，可直接再次启动。')
        self.btn_timed_run.clicked.connect(self.action_start_timed_run)
        auto_layout.addWidget(self.btn_timed_run,10,0,1,2)
        self.btn_auto_edge = QPushButton('启动自动单边（无固定时长）')
        self.btn_auto_edge.setToolTip('±3°、低角速度、飞轮低于5rad/s持续1秒后倒计时8秒。PG6亮即松手。超过20°或满速持续3秒后失能等待下次入位。急停/断连/硬故障退出自动模式。')
        self.btn_auto_edge.clicked.connect(self.action_toggle_auto_edge)
        auto_layout.addWidget(self.btn_auto_edge,11,0,1,2)
        for button in (self.btn_auto_edge,self.btn_edge_profile,self.btn_corner_profile,self.btn_full_speed,self.btn_three_axis_test,self.btn_edge_imu_cal,self.btn_timed_run):button.setEnabled(False)
        self.mode_group.setEnabled(False)
        return self.mode_group

    def action_send_lqr(self):
        self._request_state_change("BAL 0")
        values = ' '.join(f'{spin.value():.4f}' for spin in self.balance_spins)
        self.worker.send_cmd(f"LQR{self.combo_gain_axis.currentIndex()+1} {values}")

    def action_toggle_auto_edge(self):
        state = self.device_state or {}
        if state.get('Auto'):
            self._request_state_change('BAUTO 0', {'Auto': 0})
        elif self.btn_auto_edge.isEnabled():
            self._request_state_change('BAUTO 1 2' if state.get('BMode') == 4 else 'BAUTO 1', {'Auto': 1})

    def action_prepare_edge_imu_cal(self):
        if self.btn_edge_imu_cal.isEnabled():
            self._request_state_change('CAL_IMU',{'Cal':1})

    def action_set_full_speed(self):
        if self.spin_full_speed.value() >= float((self.device_state or {}).get('SLimit', 0)):
            self.status_bar.showMessage('速度上限必须大于满速保护阈值；可先加载已验证配置')
            return
        self._request_state_change('BAL 0')
        self.worker.send_cmd(f'BFULL {self.spin_full_speed.value():.1f}')

    def action_load_edge_profile(self):
        if not self.btn_edge_profile.isEnabled():return
        self._request_state_change('BAL 0')
        self.profile_kind = 'edge'
        self.edge_profile_pending = True;self.edge_profile_ack = False
        self.edge_profile_generation += 1;generation = self.edge_profile_generation
        for command in ('APP 1','BMODE 1','BAXIS1 2','BDIR1 -1','U 6','L 330','BU 6',
                        'LQR1 80 5 -0.045 0','BTRIP 20','BFULL 250','BCAP 10',
                        'BREF -0.754515325 -0.022687334 0.655890166','STATE?'):
            self.worker.send_cmd(command)
        self.set_motor_controls_locked(True)
        QTimer.singleShot(5000, lambda: self._edge_profile_timeout(generation))

    def action_load_corner_profile(self):
        if not self.btn_corner_profile.isEnabled():return
        self._request_state_change('BAL 0')
        self.profile_kind = 'corner'
        self.edge_profile_pending = True;self.edge_profile_ack = False
        self.edge_profile_generation += 1;generation = self.edge_profile_generation
        for command in self._corner_profile_commands():
            self.worker.send_cmd(command)
        self.set_motor_controls_locked(True)
        QTimer.singleShot(5000, lambda: self._edge_profile_timeout(generation))

    def _corner_profile_commands(self):
        """单点已验证配置：80/5/-0.04/0、BYAW6、τ0.25、BCAP1、U6/L330/BU6，含可选有界自动寻点。"""
        commands = ['APP 1','BMODE 4','BAXIS 2 3 1','BDIR -1 -1 -1','U 6','L 330','BU 6',
                    'LQR1 80 5 -0.04 0','LQR2 80 5 -0.04 0','LQR3 80 5 -0.04 0','BYAW 6','BATTAU .25','BTRIM 0',
                    'BTRIP 20','BFULL 250','BCAP 1',
                    'BREF -0.7028287890572913 0.3977869310324257 0.5897433770480731']
        neutral = bool(self.chk_neutral.isChecked())
        self.corner_profile_neutral = neutral
        if neutral:
            commands.append(self._neutral_learning_command())
            self.corner_profile_neutral_expect = {'NOn':1,'NTune':3,
                'NLimit':self.spin_neutral_limit.value(),
                'NGain':self.spin_neutral_gain.value(),
                'NRate':self.spin_neutral_rate.value()}
        else:
            self.corner_profile_neutral_expect = {'NOn':0}
        commands.append('STATE?')
        return commands

    def _neutral_learning_command(self):
        gain = self.spin_neutral_gain.value()
        rate = self.spin_neutral_rate.value()
        limit = self.spin_neutral_limit.value()
        return f'BNEUTRAL 1 {gain:.6f} {rate:g} {limit:g}' + self._neutral_seed_args()

    def _neutral_seed_args(self):
        """把最近一次学习到的目标角作为 seed（须在原锚点总范围锥内），否则从锚点重新开始。"""
        if not self.chk_neutral_seed.isChecked() or self.last_learned_target is None:
            return ''
        pitch_deg, roll_deg = self.last_learned_target
        p = math.radians(pitch_deg); r = math.radians(roll_deg)
        seed = (-math.sin(r)*math.cos(p), math.sin(p), math.cos(r)*math.cos(p))
        base = (-0.7028287890572913, 0.3977869310324257, 0.5897433770480731)
        dot = max(-1.0, min(1.0, sum(a*b for a, b in zip(seed, base))))
        if math.degrees(math.acos(dot)) > self.spin_neutral_limit.value() + 1e-6:
            return ''
        if any(abs(v) < .25 for v in seed):
            return ''
        return f' {pitch_deg:.6f} {roll_deg:.6f}'

    def action_start_timed_run(self):
        """限时平衡：入位倒计时启动，接管后计时，到时自动失能（保留寻点学习）。"""
        if not self.btn_timed_run.isEnabled():return
        state = self.device_state or {}
        self._cancel_motion()
        self.expected_state = {}
        self.timed_run_seconds = int(self.spin_timed_run.value())
        self.timed_run_armed = True
        if state.get('App') != 1:
            self.worker.send_cmd('APP 1')
        if state.get('Auto'):
            self.worker.send_cmd('BAUTO 0')
        self.worker.send_cmd('BSTART 2' if state.get('BMode') == 4 else 'BSTART 8')
        self.set_motor_controls_locked(True)
        self.status_bar.showMessage(f'限时平衡已就绪：入位倒计时后自动计时 {self.timed_run_seconds} 秒，到时自动停止')

    def _timed_run_stop(self):
        if not (self.timed_run_armed or self.timed_run_active):return
        self.timed_run_armed = False
        self.timed_run_active = False
        self.worker.send_cmd('BAL 0')
        self.status_bar.showMessage(f'限时 {self.timed_run_seconds} 秒结束，已停止并失能；寻点学习保留')
        self.append_log(f"<span style='color: #38bdf8;'>[限时] 运行 {self.timed_run_seconds} 秒结束，已停止并失能（保留寻点学习）。</span>")

    def _edge_profile_timeout(self, generation):
        if self.edge_profile_pending and self.edge_profile_generation == generation:
            self._cancel_motion();self.worker.send_cmd('STOP')
            self.status_bar.showMessage('已验证配置未确认完成，已停止；请检查设备返回的错误')

    def _startup_load_edge(self):
        if not self.startup_auto_edge:return
        self.action_load_edge_profile()
        self.startup_auto_edge = self.edge_profile_pending

    def _startup_arm_edge(self):
        if self.startup_auto_edge and self.btn_auto_edge.isEnabled():
            self.action_toggle_auto_edge()

    def action_send_axes(self):
        axes = ' '.join(str(c.currentIndex()+1) for c in self.axis_combos)
        self._request_state_change(f'BAXIS {axes}')

    def action_send_edge_axis(self):
        axis = self.combo_balance.currentIndex()
        if axis < 3:
            self._request_state_change(f'BAXIS{axis+1} {self.axis_combos[axis].currentIndex()+1}')

    def action_set_trip_angle(self):
        self._request_state_change('BAL 0')
        self.worker.send_cmd(f'BTRIP {self.spin_trip_angle.value():.1f}')

    def action_send_edge_direction(self):
        axis = self.combo_balance.currentIndex()
        if axis < 3:
            sign = 1 if self.direction_combos[axis].currentIndex() == 0 else -1
            self._request_state_change(f'BDIR{axis+1} {sign}')

    def action_send_direction(self):
        signs = ' '.join('1' if c.currentIndex() == 0 else '-1' for c in self.direction_combos)
        self._request_state_change(f"BDIR {signs}")

    def on_lqr_received(self, axis, kp, kd, kw, ki):
        if axis == self.combo_gain_axis.currentIndex()+1:
            for spin, value in zip(self.balance_spins, (kp, kd, kw, ki)):
                spin.setValue(value)

    def on_state_received(self, state):
        first_state = self.device_state is None
        previous_state = self.device_state or {}
        self.device_state = state
        if self.edge_profile_pending and self.edge_profile_ack:
            expected = ({'App':1,'BMode':1,'Axis1':2,'D1':-1,'Ref':1,'EN':'000','TripDeg':20.0,'FullSpeed':250.0,'VLimit':6.0,'SLimit':330.0}
                        if self.profile_kind == 'edge' else
                        {'App':1,'BMode':4,'Axis1':2,'Axis2':3,'Axis3':1,'D1':-1,'D2':-1,'D3':-1,
                         'AxisMask':7,'DirMask':7,'Ref':1,'EN':'000','TripDeg':20.0,'FullSpeed':250.0,'VLimit':6.0,'SLimit':330.0,
                         'BTrim':0.0,'TrimTune':1,'CapDeg':1.0,'YawD':6.0,'AttTau':0.25,
                         **self.corner_profile_neutral_expect})
            if all(state.get(k) == v for k,v in expected.items()):
                self.edge_profile_pending = False
                self.spin_vq.setValue(6);self.spin_vlim.setValue(330)
                self.combo_gain_axis.setCurrentIndex(0)
                gains = (80,5,-.045,0) if self.profile_kind == 'edge' else (80,5,-.04,0)
                for spin,value in zip(self.balance_spins,gains):spin.setValue(value)
                neutral_note = '（自动寻点已启用）' if self.corner_profile_neutral and self.profile_kind == 'corner' else ''
                self.status_bar.showMessage('M1单边配置已回读确认，电机失能；可启动自动单边检测'
                    if self.profile_kind == 'edge' else f'单点配置已回读确认{neutral_note}，电机失能；可启动自动单点或限时运行')
                self.profile_kind = 'edge'
        if first_state and "VLimit" in state:
            self.spin_vq.setValue(float(state["VLimit"]))
            self.spin_vlim.setValue(float(state["SLimit"]))
        if 'TripDeg' in state and previous_state.get('TripDeg') != state['TripDeg']:
            self.spin_trip_angle.setValue(float(state['TripDeg']))
        if 'FullSpeed' in state and previous_state.get('FullSpeed') != state['FullSpeed']:
            self.spin_full_speed.setValue(float(state['FullSpeed']))
        if first_state or self.sync_axes:
            for axis, combo in enumerate(self.axis_combos, 1):
                if f'Axis{axis}' in state:
                    combo.setCurrentIndex(state[f'Axis{axis}']-1)
            self.sync_axes = False
        if first_state or self.sync_directions:
            for axis, combo in enumerate(self.direction_combos, 1):
                if f"D{axis}" in state:
                    combo.setCurrentIndex(0 if state[f"D{axis}"] == 1 else 1)
            self.sync_directions = False
        self.card_self_test.set_axis_maps(state)
        self.active_motors = tuple(c == '1' for c in state['Aligned'])
        self.self_test_passed = any(self.active_motors)
        if all(state.get(k) == v for k, v in self.expected_state.items()):
            self.expected_state = {}
        combo = self.combo_balance
        combo.blockSignals(True)
        combo.setCurrentIndex(state['BMode']-1)
        combo.blockSignals(False)
        capture_angle = float(state.get('CapDeg', .5 if state.get('BMode') == 4 else 10))
        entry_angle = min(capture_angle, 3)
        names = {0: '已失能', 1: 'IMU静止校准中', 2: f'等待{entry_angle:g}°接管窗口', 3: '平衡运行', 4: '跌倒/传感器保护，需手动重启', 5: '倒计时：靠近目标并扶稳，PG8常亮才可接管', 6: f'等待扶到目标{capture_angle:g}°内，电机失能', 7: '定时短力矩探测，结束自动失能'}
        if state.get('Auto') and state['Bal'] == 6:names[6] = f'自动等待入位：{entry_angle:g}°内稳定1秒，飞轮需低于5rad/s'
        self.lbl_balance.setText(f"{names.get(state['Bal'], '未知状态')} | 倾斜 {state['Tilt']:.2f}°\n"
            f"Uq {state['U1']:+.2f} / {state['U2']:+.2f} / {state['U3']:+.2f} V | 故障 {state['Fault']} | 停机码 {state['Trip']}")
        if state['Bal'] == 5:
            self.lbl_balance.setText(self.lbl_balance.text() + f"\n剩余 {float(state.get('CountMs', 0))/1000:.1f} 秒；PG5闪烁")
        if state.get('BMode') != 4 and state.get('Bal') != 7 and 'Angle' in state:
            self.lbl_balance.setText(self.lbl_balance.text() + f"\n有符号倾角 {state['Angle']:+.2f}° | 姿态停机阈值 {float(state.get('TripDeg',20)):.1f}°")
        if state.get('AxesOK') == 0 or state.get('DirOK') == 0:
            self.lbl_balance.setText(self.lbl_balance.text() + '\n需确认轮轴映射 BAXIS 与机架方向 BDIR')
        if state.get('Ref') == 0:
            self.lbl_balance.setText(self.lbl_balance.text() + '\n当前模式尚未采集平衡姿态（BZERO）')
        reasons = {0:'无',1:'倾角超过阈值',2:'满速持续3秒',3:'立即超速',4:'机体角速度过大',5:'传感器/安全故障'}
        if 1 <= state.get('BMode',4) <= 4 and 'Auto' in state:
            reason = state.get('BReason',0)
            detection = ('正在平衡（持续运行）' if state['Bal']==3 else
                         '入位倒计时中' if state['Bal']==5 else
                         '入位条件符合' if state.get('EdgeReady') else '等待入位/静止/飞轮停转')
            corner = state.get('BMode') == 4
            mode_name = '单点（三轴）' if corner else f"M{state['BMode']} 单边"
            self.lbl_edge_detect.setText(
                f"{mode_name}检测 | {'自动已启用' if state['Auto'] else '自动未启用'}\n"
                f"{detection} | 偏角 {float(state.get('Angle',0)):+.2f}°\n"
                f"机体 {float(state.get('Rate',0)):+.3f} rad/s | 飞轮 {float(state.get('Wheel',0)):+.1f} rad/s\n"
                f"满速 {float(state.get('FullSpeed',250)):.0f} rad/s，持续 {state.get('FullMs',0)/1000:.1f}/3.0秒\n"
                f"保护：{reasons.get(reason,'未知')} | 重试冷却 {state.get('RetryMs',0)/1000:.1f}秒")
            if reason and reason != self.last_balance_reason:
                self.append_log(f"[{'单点' if corner else '单边'}保护] {reasons.get(reason,'未知')}；{'失能等待下次入位' if state['Auto'] else '自动已退出，需重新启动'}")
            self.last_balance_reason = reason
        else:self.lbl_edge_detect.setText('单边/单点自动检测需要支持BAUTO的固件，并需先选择对应模式')
        if 'NOn' in state:
            if state.get('NOn') and 'NPit' in state:
                npit = float(state['NPit']); nrol = float(state['NRol'])
                self.last_learned_target = (npit, nrol)
                self.lbl_neutral.setText(
                    f"固件寻点: 已开启 | 当前目标 {npit:.2f}°/{nrol:.2f}° | 偏移 {float(state.get('NOff',0)):.2f}°"
                    f"（范围 {float(state.get('NLimit',0)):g}°，速率 {float(state.get('NRate',0)):g}°/s）")
            elif state.get('NOn'):
                self.lbl_neutral.setText('固件寻点: 已开启')
            else:
                memo = (f" | 记忆目标 {self.last_learned_target[0]:.2f}°/{self.last_learned_target[1]:.2f}°（重载配置时恢复）"
                        if self.last_learned_target else '')
                self.lbl_neutral.setText(f"固件寻点: 已关闭{memo}")
        elif state.get('BMode') == 4 and state.get('Ref'):
            self.lbl_neutral.setText('固件寻点: 当前固件未报告寻点状态（需NTune3固件）')
        else:
            self.lbl_neutral.setText('固件寻点: 加载单点配置并启用后可用（2~5°有界）')
        if self.timed_run_armed and state.get('Bal') == 3 and not self.timed_run_active:
            self.timed_run_active = True
            self.timed_run_timer.start(self.timed_run_seconds * 1000)
            self.append_log(f"<span style='color: #38bdf8;'>[限时] 已接管，计时 {self.timed_run_seconds} 秒，到时自动停止。</span>")
        elif self.timed_run_armed and state.get('Bal') == 4:
            self.timed_run_timer.stop()
            self.timed_run_armed = False
            self.timed_run_active = False
            self.append_log("<span style='color: #f59e0b;'>[限时] 保护停机，计时取消。</span>")
        if self.timed_run_active:
            remaining = max(0, self.timed_run_timer.remainingTime()) / 1000.0
            self.lbl_edge_detect.setText(self.lbl_edge_detect.text() + f"\n限时运行中：剩余 {remaining:.0f} 秒后自动停止并失能")
        if state.get('EncType') == 1:
            self.lbl_balance.setText(self.lbl_balance.text() + f"\nF4正交 CPR={state.get('C1', '?')}/{state.get('C2', '?')}/{state.get('C3', '?')}，计数器状态不能判断模块在线")
        self.set_motor_controls_locked(bool(self.expected_state) or self.self_test_running or bool(state['Trip']))
        if self.startup_auto_edge:
            if self.startup_edge_phase == 0:
                if self.btn_edge_profile.isEnabled():
                    self.startup_edge_phase = 1
                    QTimer.singleShot(0,self._startup_load_edge)
                else:
                    self.startup_auto_edge = False
                    self.status_bar.showMessage('自动启动未执行：先完成三轴自检、IMU校准并确认固件支持自动单边')
            elif self.startup_edge_phase == 1 and not self.edge_profile_pending and self.edge_profile_ack:
                self.startup_edge_phase = 2
                QTimer.singleShot(0,self._startup_arm_edge)

    def action_start_self_test(self):
        """一键依次对三轴执行硬件自检与电角度对齐"""
        if not (self.worker.ser and self.worker.ser.is_open):
            QMessageBox.warning(self, "未连接设备", "请先连接上位机通信链路，再执行硬件自检。")
            return
        if self.self_test_running:
            return

        self._cancel_motion()
        self.self_test_passed = False
        self.self_test_running = True
        self.self_test_queue = [1, 2, 3]
        self.self_test_results = {}
        self.set_motor_controls_locked(True)
        self.card_self_test.set_testing_state()
        # 自检/对齐必须在调试模式(APP 0)下执行；启动平衡时上位机会自动切回工作模式
        self.worker.send_cmd("APP 0")
        self._start_next_self_test_axis()
        self.status_bar.showMessage("正在依次对 M1/M2/M3 执行自检与电角度对齐...")
        self.append_log("<span style='color: #38bdf8;'>[自检] 开始三轴电机与编码器检测...</span>")

    def _start_next_self_test_axis(self):
        if not self.self_test_queue:
            return
        axis = self.self_test_queue.pop(0)
        self.card_self_test.set_step(f"M{axis}")
        self.worker.send_cmd(f"TEST {axis}")

    def on_self_test_step(self, step_str):
        """自检步骤更新: M1 / M2 / M3"""
        self.card_self_test.set_step(step_str)
        self.status_bar.showMessage(f"正在自检 {step_str} 电机及对应霍尔编码器...")

    def on_self_test_report(self, report_data):
        """自检完成汇总数据接收；三轴依次执行，全部结束后统一判定"""
        if not self.self_test_running:
            return
        for idx, key in enumerate(('m1', 'm2', 'm3')):
            info = report_data.get(key, {})
            if info.get('status') not in (None, 'SKIP', 'NONE', 'UNINSTALLED'):
                self.self_test_results[key] = info
                self.card_self_test.set_axis_result(idx + 1, info)
        if self.self_test_queue:
            self._start_next_self_test_axis()
            return

        self.self_test_running = False
        merged = {
            key: self.self_test_results.get(key, {'status': 'SKIP', 'comm': 0, 'deg': 0.0})
            for key in ('m1', 'm2', 'm3')
        }
        merged['all_pass'] = all(merged[key].get('status') == 'PASS' for key in ('m1', 'm2', 'm3'))
        self.self_test_passed = merged['all_pass']
        self.card_self_test.set_result(merged)
        if merged['all_pass']:
            self.status_bar.showMessage("三轴自检通过：电机与编码器正常，可以进行平衡操作。")
            self.append_log("<span style='color: #34d399; font-weight: bold;'>[自检] 三轴自检通过：三个电机与编码器均正常。</span>")
        else:
            self.status_bar.showMessage("三轴自检未通过：检测到异常轴，保持电机控制安全锁定。")
            self.append_log("<span style='color: #f87171; font-weight: bold;'>[自检] 三轴自检失败：请检查异常轴硬件连接后重试。</span>")
        self.worker.send_cmd("STATE?")

    # =====================================================================
    # 平衡控制动作
    # =====================================================================
    def action_reset_latch(self):
        """清除安全停机/急停锁存：以当前 App 模式重发 APP 命令（保持失能）"""
        self._request_state_change(f"APP {(self.device_state or {}).get('App', 0)}")

    def action_stop_all(self):
        self._cancel_motion()
        self.worker.send_cmd("STOP")

    def action_emergency_stop(self):
        self._cancel_motion()
        self.worker.send_cmd("STOP")
        self.status_bar.showMessage("紧急停止已触发: 电机转速归零并切断驱动使能！")

    def action_set_voltage_limit(self):
        val = self.spin_vq.value()
        self.worker.send_cmd(f"U {val:.2f}")

    def action_set_velocity_limit(self):
        val = self.spin_vlim.value()
        self.worker.send_cmd(f"L {val:.2f}")

    def action_toggle_tele(self, checked):
        val = 1 if checked else 0
        self.worker.send_cmd(f"TELE {val}")

    def action_reset_cube_attitude(self):
        """重置 3D 立方体姿态零位"""
        self.cube_widget.reset_attitude_zero()
        self.status_bar.showMessage("3D 立方体姿态已重置归零")

    def action_send_custom_cmd(self):
        cmd = self.edt_custom_cmd.text().strip()
        if not cmd:
            return

        # 互锁拦截：如果尚未通过硬件自检，拦截致动类电机控制指令，保护设备安全
        if not self.self_test_passed:
            cmd_upper = cmd.upper()
            allowed_prefixes = ("TEST", "CHECK", "?", "H", "HELP", "IMU", "HALL", "ENC", "TELE", "SENSOR", "STOP", "STATE?", "PP", "FREQ", "APP", "CAL_IMU", "BZERO", "BAXIS", "BDIR", "ACC", "FAST")
            if not any(cmd_upper.startswith(p) for p in allowed_prefixes):
                self.status_bar.showMessage("操作已被互锁拦截：必须先执行并通过硬件自检后方可控制电机。")
                self.append_log(f"<span style='color: #f59e0b;'>[拦截] 未通过硬件自检，拒绝执行电机控制指令: {cmd}</span>")
                self.edt_custom_cmd.clear()
                return

        self.worker.send_cmd(cmd)
        self.edt_custom_cmd.clear()

    # =====================================================================
    # 遥测与波形数据处理 (解决曲线显示异常与滑动视口)
    # =====================================================================
    def on_telemetry_received(self, data):
        """接收并存储遥测数据点"""
        if self.start_time is None:
            self.start_time = data['time']

        t_rel = data['time'] - self.start_time

        self.history_time.append(t_rel)
        self.history_m1.append(data['m1_spd'])
        self.history_m2.append(data['m2_spd'])
        self.history_m3.append(data['m3_spd'])

        # 记录 6 轴 IMU 数据
        self.history_gx.append(data['gx'])
        self.history_gy.append(data['gy'])
        self.history_gz.append(data['gz'])
        self.history_ax.append(data['ax'])
        self.history_ay.append(data['ay'])
        self.history_az.append(data['az'])

        # 同步更新 3D 立方体姿态
        if data['has_imu']:
            self.cube_widget.update_imu_data(
                data['gx'], data['gy'], data['gz'],
                data['ax'], data['ay'], data['az']
            )

        # 同步更新三轴编码角度到 3D立方体与顶部仪表栏
        if not data.get('has_hall', False):
            for hist in (self.history_h1, self.history_h2, self.history_h3):
                hist.append(float('nan'))
        if not data.get('has_speed', False):
            for hist in (self.history_s1, self.history_s2, self.history_s3):
                hist.append(float('nan'))
        if data.get('has_hall', False):
            self.history_h1.append(data['h1'])
            self.history_h2.append(data['h2'])
            self.history_h3.append(data['h3'])
            self.card_stat_hall['val'].setText(f"{data['h1']:05.1f}° / {data['h2']:05.1f}° / {data['h3']:05.1f}°")
            # 实时同步更新 3D 立方体动量轮与表面角度标签
            self.cube_widget.update_encoder_angles(data['h1'], data['h2'], data['h3'])

        # 同步动量轮速度动画
        self.cube_widget.motor_speeds = [data['m1_spd'], data['m2_spd'], data['m3_spd']]

        # 同步更新三轴实测飞轮转速
        if data.get('has_speed', False):
            s1 = data['s1']
            s2 = data['s2']
            s3 = data['s3']
            self.history_s1.append(s1)
            self.history_s2.append(s2)
            self.history_s3.append(s3)
            self.card_stat_m1['val'].setText(f"{s1:+.1f} rad/s")
            self.card_stat_m2['val'].setText(f"{s2:+.1f} rad/s")
            self.card_stat_m3['val'].setText(f"{s3:+.1f} rad/s")
        else:
            self.card_stat_m1['val'].setText(f"{data['m1_spd']:+.2f} rad/s")
            self.card_stat_m2['val'].setText(f"{data['m2_spd']:+.2f} rad/s")
            self.card_stat_m3['val'].setText(f"{data['m3_spd']:+.2f} rad/s")

        if data['has_imu']:
            self.card_stat_gyro['val'].setText(f"{data['gx']:+.1f}/{data['gy']:+.1f}/{data['gz']:+.1f} dps")

        # 统计通信帧率
        self.tele_fps_counter += 1
        elapsed = time.time() - self.tele_last_fps_time
        if elapsed >= 1.0:
            fps = self.tele_fps_counter / elapsed
            self.card_stat_fps['val'].setText(f"{fps:.1f} FPS")
            self.tele_fps_counter = 0
            self.tele_last_fps_time = time.time()

    def update_plots(self):
        """Refresh only the visible plot with a bounded recent time window."""
        if not self.tab_curves.isVisible() or not self.history_time or len(self.history_time) < 2:
            return

        t_data = list(self.history_time)
        t_cur = t_data[-1]
        t_min = max(0.0, t_cur - 10.0)
        t_max = max(10.0, t_cur)
        first = next((i for i, t in enumerate(t_data) if t >= t_min), max(0, len(t_data) - 1))
        t_data = t_data[first:]
        n = len(t_data)
        if n < 2:
            return

        index = self.tab_curves.currentIndex()
        if index == 0 and len(self.history_h1) >= first + n:
            h1_data = list(self.history_h1)[first:first+n]
            h2_data = list(self.history_h2)[first:first+n]
            h3_data = list(self.history_h3)[first:first+n]
            self.curve_h1.setData(t_data, h1_data)
            self.curve_h2.setData(t_data, h2_data)
            self.curve_h3.setData(t_data, h3_data)
            self.plot_angle.setXRange(t_min, t_max, padding=0.01)

        elif index == 1 and len(self.history_m1) >= first + n:
            m1_data = list(self.history_m1)[first:first+n]
            m2_data = list(self.history_m2)[first:first+n]
            m3_data = list(self.history_m3)[first:first+n]
            self.curve_m1_tgt.setData(t_data, m1_data)
            self.curve_m2_tgt.setData(t_data, m2_data)
            self.curve_m3_tgt.setData(t_data, m3_data)

            if len(self.history_s1) >= first + n:
                s1_data = list(self.history_s1)[first:first+n]
                s2_data = list(self.history_s2)[first:first+n]
                s3_data = list(self.history_s3)[first:first+n]
                self.curve_s1_act.setData(t_data, s1_data)
                self.curve_s2_act.setData(t_data, s2_data)
                self.curve_s3_act.setData(t_data, s3_data)
                all_speeds = m1_data + m2_data + m3_data + s1_data + s2_data + s3_data
            else:
                all_speeds = m1_data + m2_data + m3_data

            self.plot_speed.setXRange(t_min, t_max, padding=0.01)
            min_v = min(all_speeds)
            max_v = max(all_speeds)
            bound = max(15.0, abs(min_v) + 5.0, abs(max_v) + 5.0)
            self.plot_speed.setYRange(-bound, bound, padding=0.05)

        elif index == 2 and len(self.history_gx) >= first + n:
            self.curve_gx.setData(t_data, list(self.history_gx)[first:first+n])
            self.curve_gy.setData(t_data, list(self.history_gy)[first:first+n])
            self.curve_gz.setData(t_data, list(self.history_gz)[first:first+n])
            self.plot_gyro.setXRange(t_min, t_max, padding=0.01)

        elif index == 3 and len(self.history_ax) >= first + n:
            self.curve_ax.setData(t_data, list(self.history_ax)[first:first+n])
            self.curve_ay.setData(t_data, list(self.history_ay)[first:first+n])
            self.curve_az.setData(t_data, list(self.history_az)[first:first+n])
            self.plot_accel.setXRange(t_min, t_max, padding=0.01)

    # =====================================================================
    # 日志输出管理
    # =====================================================================
    def on_log_rx(self, line):
        if self.edge_profile_pending:
            if line == 'OK BREF':self.edge_profile_ack = True
            elif line.startswith('ERR') and line != 'ERR SAFETY_TRIP:1':
                self._cancel_motion();self.worker.send_cmd('STOP')
        if line.startswith("OK BAXIS"):
            self.sync_axes = True
        if line == "OK BDIR":
            self.sync_directions = True
        if line.startswith(("OK ALIGN", "ERR ALIGN", "ERR SAFETY_TRIP")):
            # 三轴自检进行中由 on_self_test_report 驱动后续步骤，这里只刷新状态
            if not self.self_test_running:
                self.worker.send_cmd("STATE?")
        if line.startswith("ERR"):
            self.expected_state = {}
            self.status_bar.showMessage(line)
        if self.chk_filter_tele.isChecked() and line.startswith("$TELE"):
            return
        self.append_log(f"<span style='color: #27ae60;'>[RX]</span> {line}")

    def on_log_tx(self, line):
        self.append_log(f"<span style='color: #2980b9;'>[TX]</span> {line}")

    def append_log(self, html_text):
        ts = time.strftime("%H:%M:%S")
        entry = f"<span style='color: #6c7a89;'>[{ts}]</span> {html_text}"
        self.txt_log.append(entry)
        if self.chk_autoscroll.isChecked():
            self.txt_log.moveCursor(self.txt_log.textCursor().End)

    def closeEvent(self, event):
        """平稳释放资源"""
        self._cancel_motion()
        self.plot_timer.stop()
        self.port_scan_timer.stop()
        if self.worker.isRunning():
            self.worker.disconnect_port()
            self.worker.wait(500)
        event.accept()


# =========================================================================
# 程序入口
# =========================================================================
def main():
    parser = argparse.ArgumentParser(description='Cubli平衡控制上位机')
    parser.add_argument('--port',help='启动后连接的串口，如COM9')
    parser.add_argument('--auto-m1',action='store_true',help='连接后加载已验证M1单边配置并启用自动平衡，要求已完成三轴对齐和IMU校准')
    parser.add_argument('--state-json',type=Path,help='可选实时状态记录文件')
    args,qt_args = parser.parse_known_args()
    if args.auto_m1 and not args.port:parser.error('--auto-m1 requires --port')
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication([sys.argv[0]]+qt_args)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(20, 22, 26))
    palette.setColor(QPalette.WindowText, QColor(230, 230, 230))
    palette.setColor(QPalette.Base, QColor(24, 27, 32))
    palette.setColor(QPalette.AlternateBase, QColor(30, 34, 41))
    palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    palette.setColor(QPalette.Text, QColor(230, 230, 230))
    palette.setColor(QPalette.Button, QColor(34, 39, 48))
    palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Highlight, QColor(52, 152, 219))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = MainWindow()
    if args.state_json:
        def record_state(state):
            tmp = args.state_json.with_suffix('.tmp')
            try:
                tmp.write_text(json.dumps({'recorded_at':time.time(),'state':state},ensure_ascii=False),encoding='utf-8')
                tmp.replace(args.state_json)
            except OSError:pass
        win.worker.sig_state.connect(record_state)
    win.startup_auto_edge = args.auto_m1
    win.show()
    if args.port:QTimer.singleShot(0,lambda:win.worker.connect_port(args.port))
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
