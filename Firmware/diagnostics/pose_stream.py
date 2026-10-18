"""Continuous telemetry reader. Never sends commands or a safety heartbeat."""
import math
import time

import serial

from live_pose import PosePublisher
from serial_monitor_handoff import monitor_should_yield


class SerialPoseReader:
    def __init__(self, port, live_file, target, capture_angle=1, lease_directory=None):
        self.port = port
        self.publisher = PosePublisher(live_file, target, capture_angle)
        self.lease_directory = lease_directory
        self.link = None
        self.buffer = b''
        self.retry_at = 0
        self.status = 'waiting'

    def close(self):
        if self.link is not None:
            link = self.link
            self.link = None
            try:
                link.close()
            except (serial.SerialException, OSError):
                pass
        self.buffer = b''

    def step(self):
        if monitor_should_yield(self.port, self.lease_directory):
            self.close()
            self.status = 'handoff'
            self.retry_at = 0
            return
        if self.link is None:
            if time.monotonic() < self.retry_at:
                return
            try:
                link = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=.3)
                link.dtr = False
                link.rts = False
                link.port = self.port
                link.open()
                self.link = link
            except (serial.SerialException, OSError):
                self.status = 'disconnected'
                self.retry_at = time.monotonic() + 1
                return
        try:
            self.buffer += self.link.read(min(self.link.in_waiting, 65536))
        except (serial.SerialException, OSError):
            self.close()
            self.status = 'disconnected'
            self.retry_at = time.monotonic() + 1
            return
        self.status = 'connected'
        while b'\n' in self.buffer:
            raw, self.buffer = self.buffer.split(b'\n', 1)
            line = raw.decode('ascii', 'replace').strip()
            if not line.startswith(('$TELE,', '$FAST,', '$STATE,')) or not line.endswith('#'):
                continue
            try:
                data = dict(v.split(':', 1) for v in line.split(',', 1)[1][:-1].split(','))
                if 'Pit' in data and not all(math.isfinite(float(data[k])) for k in ('Pit', 'Rol')):
                    continue
                self.publisher.update(data)
            except (ValueError, KeyError, TypeError):
                continue
        if len(self.buffer) > 65536:
            self.buffer = b''
