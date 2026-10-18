"""Bounded supervised preparation after a flash: latch reset, three-axis self-test, static IMU calibration.

Keeps the heartbeat alive, prints each firmware step, verifies Aligned=111 and
Cal=1 from a fresh STATE frame, and leaves the device disabled in debug mode.
Never enables balancing; STOP on every exit.
"""
import argparse
import json
import time
import sys
from pathlib import Path

import serial
from serial_monitor_handoff import SerialMonitorHandoff

# A partial boot/UART frame may contain replacement characters. Logging it
# must not abort preparation (or its STOP cleanup) on a Windows GBK console.
for stream in (sys.stdout, sys.stderr):
    if hasattr(stream, 'reconfigure'):
        stream.reconfigure(encoding='utf-8', errors='backslashreplace')

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM7')
parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent / '2026-10-15')
parser.add_argument('--tag', default='corner_auto_prep')
args = parser.parse_args()

folder = args.output_dir
folder.mkdir(parents=True, exist_ok=True)
log = (folder / (args.tag + '.jsonl')).open('x', encoding='utf-8')
link = serial.Serial(port=None, baudrate=115200, timeout=.05, write_timeout=.3)
link.dtr = False
link.rts = False
link.port = args.port
start = time.monotonic()
buffer = bytearray()
next_ping = 0
state = {}
messages = []
fatal = None


def record(kind, data):
    log.write(json.dumps({'t': time.monotonic() - start, 'kind': kind, 'data': data}, ensure_ascii=False) + '\n')
    log.flush()


def send(command):
    link.write((command + '\r\n').encode('ascii'))
    record('tx', command)


def pump(duration):
    global buffer, next_ping, state
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_ping:
            send('PING')
            next_ping = now + .25
        buffer.extend(link.read(link.in_waiting or 1))
        while b'\n' in buffer:
            line, _, buffer = buffer.partition(b'\n')
            line = line.decode('ascii', 'replace').strip()
            if not line:
                continue
            record('rx', line)
            messages.append(line)
            if line.startswith('$STATE,'):
                state.update(dict(item.split(':', 1) for item in line.split(',', 1)[1][:-1].split(',')))
                continue
            if line.startswith(('$TELE,', '$FAST,', '$FREQ,')):
                continue
            print(('  RX ' if not line.startswith(('OK', 'ERR', '$TEST')) else '>> ') + line, flush=True)
        if len(buffer) > 65536:
            raise RuntimeError('Serial overflow')


def wait_line(predicate, label, timeout):
    deadline = time.monotonic() + timeout
    count = len(messages)
    while time.monotonic() < deadline:
        pump(.05)
        for line in messages[count:]:
            if predicate(line):
                return line
        count = len(messages)
    return None


def confirm(predicate, label, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        send('STATE?')
        got = wait_line(lambda l: l.startswith('$STATE,'), label, 1)
        if got and predicate(state):
            return
    raise RuntimeError(label + ': fresh STATE confirmation failed')


handoff = SerialMonitorHandoff(args.port)
try:
    handoff.open(link)
    print('已连接', args.port, '；下发 PING/STOP/APP 0（复位停机锁存）', flush=True)
    send('PING')
    pump(.3)
    send('STOP')
    pump(.3)
    send('APP 0')
    pump(.5)
    print('心跳正常，开始三轴自检（对齐时飞轮会短转，请扶稳不动）', flush=True)
    for motor in (1, 2, 3):
        send(f'DBG {motor}')
        pump(.2)
        print(f'开始 M{motor} 自检对齐...', flush=True)
        send(f'TEST {motor}')
        line = wait_line(lambda l: l.startswith(('OK ALIGN', 'ERR ALIGN', 'ERR ')), f'M{motor} align', 40)
        if line is None:
            raise RuntimeError(f'M{motor} 自检超时未返回结果')
        if not line.startswith('OK ALIGN'):
            raise RuntimeError(f'M{motor} 自检失败：{line}')
        print(f'M{motor} 自检通过', flush=True)
    confirm(lambda d: d.get('Aligned') == '111', '三轴对齐')
    print('三轴 Aligned=111。开始 IMU 静止校准：请保持完全静止约1秒', flush=True)
    send('CAL_IMU')
    line = wait_line(lambda l: l.startswith(('OK CAL_IMU:DONE', 'ERR ')), 'CAL_IMU', 15)
    if line is None or not line.startswith('OK CAL_IMU:DONE'):
        raise RuntimeError('IMU 校准未完成：' + str(line))
    confirm(lambda d: d.get('Cal') == '1' and d.get('Aligned') == '111' and d.get('EN') == '000', '校准后状态')
    print(f"校准完成：Aligned={state.get('Aligned')} Cal={state.get('Cal')} IMU={state.get('IMU')} "
          f"EN={state.get('EN')} Fault={state.get('Fault')} Enc={state.get('Enc')}", flush=True)
    print('准备完成：设备已对齐并校准、三轴失能。可以开始加载单点配置并启动自动平衡。', flush=True)
except KeyboardInterrupt:
    fatal = '用户中断'
except BaseException as error:
    fatal = repr(error)
    print('失败：' + fatal, flush=True)
finally:
    if link.is_open:
        try:
            send('STOP')
            pump(.3)
            confirm(lambda d: d.get('EN') == '000', '最终失能', 2)
        except BaseException as error:
            fatal = fatal or repr(error)
            print('退出前失能确认失败：' + repr(error), flush=True)
        finally:
            link.close()
    handoff.release()
    log.close()
if fatal is not None:
    raise SystemExit(2)
