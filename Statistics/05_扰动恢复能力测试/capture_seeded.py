#!/usr/bin/env python3
"""使用已种子的中性参考直接捕获平衡段 (图 5 恢复尝试)

前提: 设备已校准/对齐, 已执行
    BNEUTRAL 1 0.00025 0.5 5 <seed_pitch> <seed_roll>
本脚本不再发送 BREF/BMAXIS/BMODE, 避免 LQR_ResetNeutralLearning() 清掉种子;
只发 APP 1 -> BAUTO -> 等待 Bal=3 -> 记录 30 s -> STOP。

用法:
    py capture_seeded.py COM10 kick_run1.csv [--balance=30] [--wait=600]

现场配合:
    * 把设备扶到 PG8 常亮 (新参考比原来的 Rol 50.0 约小 1.4 度);
    * PG6 亮起瞬间松手, 不要继续扶着;
    * 松手后 15 s 内轻敲 3 次 (约 4/9/14 s), 同一侧, 力度一致。
"""
import csv, re, sys, threading, time
import serial


def main():
    pos = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(pos) < 2:
        print(__doc__)
        return 1
    port, out_csv = pos[0], pos[1]
    bal_seconds, wait_seconds = 30.0, 600.0
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
            return list(rows)

    def find(prefix):
        for _, s in reversed(latest()):
            if s.startswith(prefix):
                return s
        return None

    def field(s, k):
        m = re.search(r"(?:^|[, ])" + k + r":(-?[0-9.]+)", s)
        return float(m.group(1)) if m else None

    ser.write(b"PING\r\n"); ser.flush(); time.sleep(0.6)
    send("STOP", 1.0, ("OK STOP", "ERR"))
    time.sleep(0.2)
    r = send("APP 1", 2.0, ("OK APP", "ERR APP"))
    print("APP:", r, flush=True)
    time.sleep(0.4)

    st = find("$STATE")
    if not st:
        print("没有收到 $STATE, 中止", flush=True); stop.set(); ser.close(); return 1
    print("当前状态: NOn=%s NPit=%s NRol=%s BMode=%s Cal=%s Aligned=%s" % (
        field(st, "NOn"), field(st, "NPit"), field(st, "NRol"),
        field(st, "BMode"), field(st, "Cal"), st.split(",Aligned:")[1][:3]
        if ",Aligned:" in st else "?"), flush=True)
    if field(st, "NOn") != 1:
        print("中性搜索未启用, 中止 (先发 BNEUTRAL 种子)", flush=True)
        stop.set(); ser.close(); return 1

    with lock:
        rows.clear()
    t0 = time.perf_counter() * 1000.0

    r = send("BAUTO 1 2", 4.0, ("OK BAUTO", "ERR BAUTO"))
    print("BAUTO:", r, flush=True)
    if not r or r.startswith("ERR"):
        stop.set(); ser.close(); return 1
    print("请扶到 PG8 常亮 (新参考 Rol 约 48.6, 比原来小约 1.4 度); PG6 亮起瞬间松手", flush=True)
    print("松手后 15 s 内轻敲 3 次 (约 4/9/14 s), 同一侧, 力度一致", flush=True)

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
            s2 = find("$STATE")
            if s2:
                print("  等待入位: Tilt=%s Ready=%s Bal=%s NPit=%s NRol=%s" % (
                    field(s2, "Tilt"), field(s2, "EdgeReady"), field(s2, "Bal"),
                    field(s2, "NPit"), field(s2, "NRol")), flush=True)
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
