#!/usr/bin/env python3
"""等待入位 + 捕获平衡段控制分量 (图 4)

与 capture_control.py 的区别: 不重新对齐/校准, 复用当前固件状态;
发送 BAUTO 后持续等待 Bal=3, 检测到平衡后再记录 30 s。
适合设备已校准、只需重新扶姿采集的场合。

用法:
    py capture_control_wait.py COM10 ctrl_run1.csv [--balance=30] [--wait=150]

现场配合:
    * 运行后立即把设备扶成单点姿态 Pit~23.4 deg / Rol~50.0 deg 并保持;
    * PG8 常亮表示进入 0.5 deg 窗口, PG5 倒计时, PG6 亮起时松手;
    * 若 150 s 内未入位, 脚本保存已有数据并退出。
"""
import csv, re, sys, threading, time
import serial

REF = (-0.7028287890572913, 0.3977869310324257, 0.5897433770480731)


def main():
    pos = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(pos) < 2:
        print(__doc__)
        return 1
    port, out_csv = pos[0], pos[1]
    bal_seconds, wait_seconds = 30.0, 150.0
    for a in sys.argv:
        if a.startswith("--balance="):
            bal_seconds = float(a.split("=", 1)[1])
        if a.startswith("--wait="):
            wait_seconds = float(a.split("=", 1)[1])

    ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    rows, lock, stop = [], threading.Lock(), threading.Event()

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                data = ser.read(256)
            except serial.SerialException:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.strip(b"\r").decode("utf-8", "replace")
                if s:
                    with lock:
                        rows.append((time.perf_counter() * 1000.0, s))

    def pinger():
        while not stop.is_set():
            stop.wait(0.2)
            if not stop.is_set():
                try:
                    ser.write(b"PING\r\n")
                except serial.SerialException:
                    break

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=pinger, daemon=True).start()

    def send(cmd, timeout_s=3.0, expect=("OK", "ERR")):
        t_send = time.perf_counter() * 1000.0
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with lock:
                snap = list(rows)
            for ts, s in snap:
                if ts >= t_send - 1.0 and any(s.startswith(e) for e in expect):
                    return s
            time.sleep(0.02)
        return None

    def latest():
        with lock:
            snap = list(rows)
        return snap

    ser.write(b"PING\r\n"); ser.flush(); time.sleep(0.6)
    send("STOP", 1.0, ("OK STOP", "ERR"))
    time.sleep(0.3)
    send("DBG 1", 2.0, ("OK DEBUG", "ERR"))
    send("APP 1", 2.0, ("OK APP", "ERR APP"))
    r = send("BREF %.9f %.9f %.9f" % REF, 4.0, ("OK BREF", "ERR BREF"))
    if not r or r.startswith("ERR"):
        print("BREF 失败:", r); stop.set(); ser.close(); return 1
    with lock:
        rows.clear()
    t0 = time.perf_counter() * 1000.0

    r = send("BAUTO 1 2", 4.0, ("OK BAUTO", "ERR BAUTO"))
    print("BAUTO:", r, flush=True)
    if not r or r.startswith("ERR"):
        stop.set(); ser.close(); return 1

    print("请现在把设备扶成单点姿态 (Pit~23.4  Rol~50.0) 并保持不动; PG6 亮起时松手", flush=True)

    def field(s, k):
        m = re.search(r"(?:^|[, ])" + k + r":(-?[0-9.]+)", s)
        return float(m.group(1)) if m else None

    def find(prefix):
        snap = latest()
        for _, s in reversed(snap):
            if s.startswith(prefix):
                return s
        return None

    start_ms = None
    deadline = time.time() + wait_seconds
    last_print = 0.0
    while time.time() < deadline:
        tele = find("$TELE")
        if tele and field(tele, "Bal") == 3:
            start_ms = time.perf_counter() * 1000.0
            print("检测到 Bal=3, 开始记录 %.0f s" % bal_seconds, flush=True)
            break
        if time.time() - last_print > 2.0:
            last_print = time.time()
            st = find("$STATE")
            if st:
                print("  等待入位: Tilt=%s PoseTilt=%s Angle=%s CapDeg=%s Ready=%s Bal=%s" % (
                    field(st, "Tilt"), field(st, "PoseTilt"), field(st, "Angle"),
                    field(st, "CapDeg"), field(st, "EdgeReady"), field(st, "Bal")), flush=True)
        time.sleep(0.1)

    if start_ms is None:
        print("超时: 未检测到平衡 (Bal=3), 保存等待数据", flush=True)
        result = 1
    else:
        bal_end = start_ms + bal_seconds * 1000.0
        while time.perf_counter() * 1000.0 < bal_end:
            time.sleep(0.1)
        result = 0

    try:
        ser.write(b"STOP\r\n"); ser.flush()
    except serial.SerialException:
        pass
    time.sleep(0.3)
    stop.set(); time.sleep(0.2)

    all_rows = latest()
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_local_ms", "t_rel_ms", "raw"])
        for ts, s in all_rows:
            w.writerow([f"{ts:.1f}", f"{ts - t0:.1f}", s])
    tele = sum(1 for _, s in all_rows if s.startswith("$TELE"))
    print(f"Saved {len(all_rows)} lines ({tele} $TELE) -> {out_csv}", flush=True)
    ser.close()
    return result


if __name__ == "__main__":
    sys.exit(main())
