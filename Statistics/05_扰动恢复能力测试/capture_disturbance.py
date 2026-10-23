#!/usr/bin/env python3
"""等待入位 + 捕获带人工冲击的平衡段 (图 5: 扰动恢复能力)

复用当前固件状态 (不重新对齐/校准), 发送 BAUTO 后持续等待 Bal=3,
检测到平衡后记录 30 s。冲击时刻由 MATLAB 从陀螺/姿态数据中自动检测,
因此脚本不需要触发线, 只要连续记录即可。

用法:
    py capture_disturbance.py COM10 kick_run1.csv [--prepare] [--balance=30] [--wait=600]

    --prepare  完整执行三轴对齐 + IMU 校准 (换电池/断电复位后必须使用);
               不加时复用当前校准, 只重新下发配置并等待入位。

现场配合:
    * 使用 --prepare 时先把设备平放静止, 脚本 5 s 后自动对齐和校准;
    * 之后把设备扶到 PG8 常亮 (中性参考 Rol 约 48.6, 不是旧的 50.0);
    * PG5 倒计时, PG6 亮起瞬间松手;
    * 松手后 15 s 内轻敲设备 3 次 (建议约 4 s / 9 s / 14 s), 每次轻触同一侧,
      力度尽量一致; 不要持续推, 也不要碰到飞轮;
    * 脚本记录 30 s 后自动 STOP。
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
    prepare = "--prepare" in sys.argv
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

    def wait_line(substr, timeout_s, since_ms):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            for ts, s in latest():
                if ts >= since_ms - 1.0 and substr in s:
                    return s
            time.sleep(0.03)
        return None

    def must(cmd, expect, timeout_s=4.0):
        r = send(cmd, timeout_s, expect)
        print("      %-28s -> %s" % (cmd, r), flush=True)
        return r

    ser.write(b"PING\r\n"); ser.flush(); time.sleep(0.6)
    send("STOP", 1.0, ("OK STOP", "ERR"))
    time.sleep(0.3)
    must("DBG 1", ("OK DEBUG", "ERR"))

    if prepare:
        print("[prep] 设备必须平放静止; 5 s 后开始三轴对齐", flush=True)
        time.sleep(5.0)
        for m in (1, 2, 3):
            print(f"[prep] 对齐 M{m} (~7 s)", flush=True)
            t_send = time.perf_counter() * 1000.0
            ser.write(f"TEST {m}\r\n".encode()); ser.flush()
            r = wait_line("OK ALIGN", 22, t_send) or wait_line("ERR ALIGN", 2, t_send)
            print("      " + str(r), flush=True)
            if not r or r.startswith("ERR"):
                print("对齐失败, 中止", flush=True)
                stop.set(); ser.close(); return 1
        print("[prep] IMU 静止校准", flush=True)
        t_send = time.perf_counter() * 1000.0
        ser.write(b"CAL_IMU\r\n"); ser.flush()
        r = wait_line("OK CAL_IMU:DONE", 12, t_send)
        print("      " + str(r), flush=True)
        if not r:
            print("校准失败, 中止", flush=True)
            stop.set(); ser.close(); return 1

    print("[cfg] corner 配置", flush=True)
    for cmd, exp in [
        ("APP 1", ("OK APP", "ERR APP")),
        ("BMODE 4", ("OK BMODE", "ERR BMODE")),
        ("BAXIS 2 3 1", ("OK BAXIS", "ERR BAXIS")),
        ("BDIR -1 -1 -1", ("OK BDIR", "ERR BDIR")),
        ("U 6", ("OK LIMIT", "ERR")),
        ("L 330", ("OK LIMIT", "ERR")),
        ("BU 6", ("OK LIMIT", "ERR")),
        ("BTRIP 20", ("OK BTRIP", "ERR")),
        ("BCAP 0.5", ("OK BCAP", "ERR")),
        ("LQR1 80 5 -0.030 0", ("OK LQR1", "ERR")),
        ("LQR2 80 5 -0.030 0", ("OK LQR2", "ERR")),
        ("LQR3 80 5 -0.030 0", ("OK LQR3", "ERR")),
        ("BATTAU .25", ("OK BATTAU", "ERR")),
        ("BYAW 3", ("OK BYAW", "ERR")),
        ("BTRIM 0", ("OK BTRIM", "ERR")),
        ("BFULL 250", ("OK BFULL", "ERR")),
    ]:
        must(cmd, exp)
    r = must("BREF %.9f %.9f %.9f" % REF, ("OK BREF", "ERR BREF"), 4.0)
    if not r or r.startswith("ERR"):
        print("BREF 失败:", r, flush=True); stop.set(); ser.close(); return 1
    # BREF resets neutral learning; seed it afterwards so the fixed 1-2 degree
    # COM offset does not saturate the wheels. PG8 is the reliable pose guide.
    r = must("BNEUTRAL 1 0.00025 0.5 5 23.40 48.60",
             ("OK BNEUTRAL", "ERR BNEUTRAL"), 3.0)
    if not r or r.startswith("ERR"):
        print("BNEUTRAL 种子失败:", r, flush=True)
        stop.set(); ser.close(); return 1
    with lock:
        rows.clear()
    t0 = time.perf_counter() * 1000.0

    r = send("BAUTO 1 2", 4.0, ("OK BAUTO", "ERR BAUTO"))
    print("BAUTO:", r, flush=True)
    if not r or r.startswith("ERR"):
        stop.set(); ser.close(); return 1

    print("请把设备扶到 PG8 常亮 (中性参考 Rol 约 48.6, 不要按旧的 50.0 找); PG6 亮起瞬间松手", flush=True)
    print("松手后 15 s 内轻敲 3 次 (约 4/9/14 s), 同一侧, 力度一致", flush=True)

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
