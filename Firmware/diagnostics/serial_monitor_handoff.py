"""Let a read-only pose monitor yield its port to a diagnostic process."""
import atexit
import ctypes
import json
import os
import re
import time
import uuid
from pathlib import Path

import serial


MONITOR_DIRECTORY = Path(__file__).resolve().parents[2] / '.codex-debug' / 'serial-monitor'


def lease_path(port, directory=None):
    name = re.sub(r'[^A-Z0-9_.-]', '_', str(port).upper().replace('\\\\.\\', ''))
    # Windows treats COM7.json as the COM7 device, even inside a directory.
    return Path(directory or MONITOR_DIRECTORY) / ('serial-' + name + '.json')


def process_alive(pid):
    if not isinstance(pid, int) or not 0 < pid <= 0xffffffff:
        return False
    if os.name != 'nt':
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
    # os.kill(pid, 0) is not a safe existence check on Windows.
    from ctypes import wintypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    kernel.GetExitCodeProcess.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    handle = kernel.OpenProcess(0x1000, False, pid)
    if not handle:
        return ctypes.get_last_error() != 87
    try:
        code = wintypes.DWORD()
        return not kernel.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value == 259
    finally:
        kernel.CloseHandle(handle)


def monitor_should_yield(port, directory=None):
    path = lease_path(port, directory)
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return False
    except (OSError, ValueError):
        try:
            return time.time() - path.stat().st_mtime < 5
        except OSError:
            return False
    return process_alive(data.get('pid'))


class SerialMonitorHandoff:
    def __init__(self, port, directory=None):
        self.port = port
        self.directory = directory
        self.path = lease_path(port, directory)
        self.token = uuid.uuid4().hex
        self.owned = False

    def acquire(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(2):
            try:
                fd = os.open(str(self.path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            except FileExistsError:
                if monitor_should_yield(self.port, self.directory):
                    raise RuntimeError('Another diagnostic owns serial port ' + str(self.port))
                self.path.unlink(missing_ok=True)
                continue
            self.owned = True
            with os.fdopen(fd, 'w', encoding='utf-8') as output:
                json.dump({'pid': os.getpid(), 'token': self.token}, output)
            atexit.register(self.release)
            return
        raise RuntimeError('Could not reserve serial port ' + str(self.port))

    def open(self, link, timeout=3):
        self.acquire()
        deadline = time.perf_counter() + timeout
        try:
            while True:
                try:
                    link.open()
                    return
                except serial.SerialException:
                    if time.perf_counter() >= deadline:
                        raise
                    time.sleep(.05)
        except BaseException:
            self.release()
            raise

    def release(self):
        if not self.owned:
            return
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('token') == self.token and data.get('pid') == os.getpid():
                self.path.unlink(missing_ok=True)
        except (OSError, ValueError):
            pass
        self.owned = False
