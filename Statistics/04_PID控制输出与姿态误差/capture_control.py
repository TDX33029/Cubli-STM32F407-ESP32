#!/usr/bin/env python3
"""单点(corner)平衡控制分量采集 (图 4: 控制输出与姿态误差)

需要固件已包含 E1-E3 / P1-P3 / D1-D3 / W1-W3 遥测字段。

按项目已验证流程 (Firmware/Diagnostics/serial_corner_auto.py):
    APP 1 -> BMODE 4 -> BAXIS 2 3 1 -> BDIR -1 -1 -1 -> U 6 L 330 BU 6
    -> BTRIP 20 -> BCAP 0.5 -> LQR1/2/3 80 5 -0.030 0 -> BATTAU .25
    -> BYAW 3 -> BTRIM 0 -> BFULL 250 -> BREF <verified reference>
    -> BAUTO 1 2 (自动等待入位, 2 s 倒计时自动接管)

用法:
    py capture_control.py COM10 ctrl_run1.csv [--seconds 60] [--skip-prepare]

现场配合:
    * 开始前设备必须平放稳定 (三轴对齐 + IMU 校准会自动执行);
    * 听到提示后把设备扶成单点姿态 Pit~23.4 deg / Rol~50.0 deg 并保持;
    * PG8 常亮表示进入 0.5 deg 窗口, PG5 倒计时, PG6 亮起时松手;
    * 采集结束脚本发 STOP, 设备退出平衡。
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
    seconds = 60.0
    skip_prep = "--skip-prepare" in sys.argv
    for a in sys.argv:
        if a.startswith("--seconds="):
            seconds = float(a.split("=", 1)[1])

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

    def send(cmd, timeout_s=4.0, expect=("OK", "ERR")):
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

    def wait_line(substr, timeout_s, since_ms):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with lock:
                snap = list(rows)
            for ts, s in snap:
                if ts >= since_ms - 1.0 and substr in s:
                    return s
            time.sleep(0.03)
        return None

    def must(cmd, expect, timeout_s=4.0):
        r = send(cmd, timeout_s, expect)
        print("      %-28s -> %s" % (cmd, r), flush=True)
        return r

    ser.write(b"PING\r\n"); ser.flush(); time.sleep(0.8)
    send("STOP", timeout_s=1.0, expect=("OK STOP", "ERR"))
    time.sleep(0.4)
    t0 = time.perf_counter() * 1000.0
    with lock:
        rows.clear()

    print("[1] 清锁存/工作模式", flush=True)
    must("DBG 1", ("OK DEBUG", "ERR"))

    if not skip_prep:
        for m in (1, 2, 3):
            print(f"[2] 对齐 M{m} (约 7 s)", flush=True)
            t_send = time.perf_counter() * 1000.0
            ser.write(f"TEST {m}\r\n".encode()); ser.flush()
            r = wait_line("OK ALIGN", 22, t_send) or wait_line("ERR ALIGN", 2, t_send)
            print("      " + str(r), flush=True)
            if not r or r.startswith("ERR"):
                print("对齐失败, 中止"); stop.set(); ser.close(); return 1
        print("[3] IMU 静止校准", flush=True)
        t_send = time.perf_counter() * 1000.0
        ser.write(b"CAL_IMU\r\n"); ser.flush()
        r = wait_line("OK CAL_IMU:DONE", 12, t_send)
        print("      " + str(r), flush=True)
        if not r:
            print("校准失败, 中止"); stop.set(); ser.close(); return 1

    print("[4] corner 配置 (BMODE 4 / BAXIS 2 3 1 / BDIR -1 -1 -1)", flush=True)
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
    ]:
        must(cmd, exp)

    print("[5] LQR/滤波/保护参数 (验证值)", flush=True)
    for cmd, exp in [
        ("LQR1 80 5 -0.030 0", ("OK LQR1", "ERR")),
        ("LQR2 80 5 -0.030 0", ("OK LQR2", "ERR")),
        ("LQR3 80 5 -0.030 0", ("OK LQR3", "ERR")),
        ("BATTAU .25", ("OK BATTAU", "ERR")),
        ("BYAW 3", ("OK BYAW", "ERR")),
        ("BTRIM 0", ("OK BTRIM", "ERR")),
        ("BFULL 250", ("OK BFULL", "ERR")),
    ]:
        must(cmd, exp)

    print("[6] BREF 固定单点参考", flush=True)
    r = must("BREF %.9f %.9f %.9f" % REF, ("OK BREF", "ERR BREF"), 4.0)
    if not r or r.startswith("ERR"):
        print("BREF 失败, 中止"); stop.set(); ser.close(); return 1

    print("[7] 请现在把设备扶成单点姿态 (Pit~23.4  Rol~50.0), 保持稳定", flush=True)
    time.sleep(12.0)

    print("[8] BAUTO 1 2: 等待入位 -> 2 s 倒计时 -> 自动接管", flush=True)
    r = must("BAUTO 1 2", ("OK BAUTO", "ERR BAUTO"), 5.0)
    if not r or r.startswith("ERR"):
        print("BAUTO 失败, 中止"); stop.set(); ser.close(); return 1

    print(f"[9] 采集 {seconds:.0f} s (PG6 亮起时松手, 跌倒后会自动等待下次入位)", flush=True)
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        time.sleep(0.1)

    try:
        ser.write(b"STOP\r\n"); ser.flush()
    except serial.SerialException:
        pass
    time.sleep(0.3)
    stop.set(); time.sleep(0.2)

    with lock:
        all_rows = rows[:]
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_local_ms", "t_rel_ms", "raw"])
        for ts, s in all_rows:
            w.writerow([f"{ts:.1f}", f"{ts - t0:.1f}", s])
    tele = sum(1 for _, s in all_rows if s.startswith("$TELE"))
    print(f"Saved {len(all_rows)} lines ({tele} $TELE) -> {out_csv}")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
