"""Small continuous pose display; it cannot enable motors or capture references."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout, QWidget


def gravity(pitch, roll):
    p, r = math.radians(pitch), math.radians(roll)
    return [-math.sin(r)*math.cos(p), math.sin(p), math.cos(r)*math.cos(p)]


class PoseMonitor(QWidget):
    def __init__(self, live_file, capture_file=None, reader=None):
        super().__init__()
        self.live_file, self.reader = Path(live_file), reader
        self.data = {}
        self.last_reminded_trial = None
        self.reminder = None
        font_path = Path('C:/Windows/Fonts/msyh.ttc')
        if font_path.exists():
            QFontDatabase.addApplicationFont(str(font_path))
            self.setFont(QFont('Microsoft YaHei', 12))
        self.setWindowTitle('Cubli 实时角度')
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self.setStyleSheet('QWidget {background:#111827;color:#e5e7eb;} QLabel {padding:8px;}')
        layout = QVBoxLayout(self)
        self.status = QLabel('等待串口数据')
        self.angles = QLabel('Pitch —°     Roll —°')
        self.angles.setStyleSheet('font-size:28px;font-weight:bold;')
        self.target = QLabel('目标 —')
        self.error = QLabel('偏差 —')
        for widget in (self.status, self.angles, self.target, self.error):
            widget.setAlignment(Qt.AlignCenter)
            layout.addWidget(widget)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def refresh(self):
        # Preserve the last test target when the passive reader resumes.
        try:
            data = json.loads(self.live_file.read_text(encoding='utf-8'))
            if data.get('target') and self.reader:
                self.reader.publisher.target = data['target']
                self.reader.publisher.capture_angle = data.get('capture_angle', 1)
                self.reader.publisher.outcome_text = data.get('outcome_text')
                self.reader.publisher.trial_id = data.get('trial_id')
                self.reader.publisher.trial_text = data.get('trial_text')
            self.data = data
        except (OSError, ValueError, TypeError):
            pass
        if self.reader:
            self.reader.step()
        try:
            self.data = json.loads(self.live_file.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            pass
        data = self.data
        if not data or time.time() - data.get('timestamp', 0) > 1:
            self.close_reminder()
            self.angles.setText('Pitch —°     Roll —°')
            self.error.setText('偏差 —')
            status = getattr(self.reader, 'status', None)
            self.status.setText('串口已连接 · 等待姿态数据' if status == 'connected' else
                                '等待本轮实时数据' if status == 'handoff' else
                                '串口未连接 · 自动重连中' if self.reader else '等待实时数据')
            return
        trial = data.get('trial_id')
        if data.get('bal') in (5, 6) and trial and trial != self.last_reminded_trial:
            self.last_reminded_trial = trial
            self.show_trial_reminder(data)
        elif data.get('bal') not in (5, 6):
            self.close_reminder()
        if self.reminder is not None:
            target = data.get('target')
            self.reminder_angles.setText(f"当前 {data['pitch']:+.2f}° / {data['roll']:+.2f}°" +
                                         (f"\n目标 {target[0]:+.2f}° / {target[1]:+.2f}°" if target else ''))
        self.status.setText({0: '电机失能', 1: '静止校准中', 2: '等待接管',
                             3: '正在运行 · 松手', 4: '保护停机',
                             5: '倒计时 · 扶稳', 6: '等待入位 · 扶稳'}.get(data.get('bal'), '读取中'))
        if data.get('outcome_text') == '停机未确认':
            self.status.setText('停机未确认')
        elif data.get('bal') == 3 and data.get('remaining_s') is not None:
            self.status.setText(f"运行 · 松手 · 剩余 {max(0,data['remaining_s']):.1f} 秒")
        elif data.get('bal') == 0 and data.get('outcome_text'):
            self.status.setText(f"上次{data['outcome_text']} · 已失能")
        self.angles.setText(f"Pitch {data['pitch']:+.2f}°     Roll {data['roll']:+.2f}°")
        target = data.get('target')
        if target:
            label = '自动寻点目标' if data.get('neutral_enabled') else '目标'
            self.target.setText(f'{label} {target[0]:+.2f}° / {target[1]:+.2f}°')
            g, ref = gravity(data['pitch'], data['roll']), gravity(*target)
            angle = math.degrees(math.acos(max(-1, min(1, sum(a*b for a, b in zip(g, ref))))))
            self.error.setText(f"偏差 {angle:.2f}°    入位范围 {data.get('capture_angle',1):g}°")

    def show_trial_reminder(self, data):
        self.close_reminder()
        dialog = QDialog(self, Qt.Tool | Qt.WindowStaysOnTopHint)
        dialog.setAttribute(Qt.WA_ShowWithoutActivating, True)
        dialog.setWindowTitle('Cubli · 下一轮测试提醒')
        dialog.setFont(self.font())
        layout = QVBoxLayout(dialog)
        text = QLabel((data.get('trial_text') or '下一轮测试') +
                      '\n请扶回同一支撑角，靠近目标并扶稳。\nPG5倒计时，PG6亮即松手。\n程序自动开始，关闭提示不影响测试。')
        text.setWordWrap(True)
        layout.addWidget(text)
        self.reminder_angles = QLabel()
        layout.addWidget(self.reminder_angles)
        dialog.resize(460, 230)
        screen = QApplication.primaryScreen().availableGeometry()
        dialog.move(screen.right()-dialog.width()-20, screen.top()+20)
        self.reminder = dialog
        dialog.show()
        dialog.raise_()

    def close_reminder(self):
        if self.reminder is not None:
            self.reminder.close()
            self.reminder.deleteLater()
            self.reminder = None

    def closeEvent(self, event):
        self.close_reminder()
        self.timer.stop()
        if self.reader:
            self.reader.close()
        super().closeEvent(event)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-file', type=Path, required=True)
    parser.add_argument('--port', help='Continuous read-only serial source, e.g. COM7')
    parser.add_argument('--target-pitch', type=float, default=23.4399)
    parser.add_argument('--target-roll', type=float, default=50)
    parser.add_argument('--capture-angle', type=float, default=1)
    parser.add_argument('--capture-file', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    reader = None
    if args.port:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'diagnostics'))
        from pose_stream import SerialPoseReader
        reader = SerialPoseReader(args.port, args.live_file,
                                  [args.target_pitch, args.target_roll], args.capture_angle)
    app = QApplication([])
    window = PoseMonitor(args.live_file, reader=reader)
    window.resize(540, 250)
    screen = app.primaryScreen().availableGeometry()
    window.move(screen.center()-window.rect().center())
    window.show()
    window.raise_()
    window.activateWindow()
    app.exec_()
