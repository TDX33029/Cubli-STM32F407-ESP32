#!/usr/bin/env python3
"""三组 LQR 参数的平衡性能对比采集 (图 6)

依次运行三组参数, 每组:
    BREF 固定参考 -> BNEUTRAL 种子 -> LQR 增益 -> BAUTO -> 等待入位
    -> 记录 30 s -> STOP
每组之间保持同一中性参考种子, 使三组数据可比。

用法:
    py capture_compare.py COM10 [--prepare] [--balance=30] [--wait=180]

现场配合 (共 3 轮):
    * 第 1 轮: 扶到 PG8 常亮, PG6 亮起瞬间松手, 平衡约 10 s 时轻敲一次;
    * 约 30 s 后设备自动停转, 再扶第 2 轮, 同样 PG8 -> PG6 -> 10 s 轻敲;
    * 第 3 轮同理。敲击方向/力度尽量一致。
"""
import csv, re, sys, threading, time, os
import serial

REF = (-0.7028287890572913, 0.3977869310324257, 0.5897433770480731)
SEED = (23.40, 48.60)
SCHEMES = [
    ("base", 80.0, 5.0, -0.030, 0.0),
    ("low",  60.0, 4.0, -0.030, 0.0),
    ("high", 110.0, 7.0, -0.030, 0.0),
]


def main():
    pos = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not pos:
        print(__doc__)
        return 1
    port = pos[0]
    out_dir = os.path.dirname(os.path.abspath(__file__))
    bal_seconds, wait_seconds = 30.0, 180.0
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

    def latest():
        with lock:
            return list(rows)

    def send(cmd, timeout_s=3.0, expect=("OK", "ERR")):
        t_send = time.perf_counter() * 1000.0
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            for ts, s in latest():
                if ts >= t_send - 1.0 and any(s.startswith(e) for e in expect):
                    return s
            time.sleep(0.02)
        return None

    def wait_line(substr, timeout_s, since_ms):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            for ts, s in latest():
                if ts >= since_ms - 1.0 and substr in s:
                    return s
            time.sleep(0.03)
        return None

    def must(cmd, expect=("OK", "ERR"), timeout_s=4.0):
        r = send(cmd, timeout_s, expect)
        print("      %-32s -> %s" % (cmd, r), flush=True)
        return r

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
    time.sleep(0.3)
    must("DBG 1", ("OK DEBUG", "ERR"))

    if prepare:
        print("[prep] 设备平放静止; 5 s 后对齐/校准", flush=True)
        time.sleep(5.0)
        for m in (1, 2, 3):
            print(f"[prep] 对齐 M{m}", flush=True)
            t_send = time.perf_counter() * 1000.0
            ser.write(f"TEST {m}\r\n".encode()); ser.flush()
            r = wait_line("OK ALIGN", 22, t_send) or wait_line("ERR ALIGN", 2, t_send)
            print("      " + str(r), flush=True)
            if not r or r.startswith("ERR"):
                stop.set(); ser.close(); return 1
        t_send = time.perf_counter() * 1000.0
        ser.write(b"CAL_IMU\r\n"); ser.flush()
        r = wait_line("OK CAL_IMU:DONE", 12, t_send)
        print("      " + str(r), flush=True)
        if not r:
            stop.set(); ser.close(); return 1

    print("[cfg] corner 基础配置", flush=True)
    must("APP 1", ("OK APP", "ERR APP"))
    for cmd, exp in [
        ("BMODE 4", ("OK BMODE", "ERR BMODE")),
        ("BAXIS 2 3 1", ("OK BAXIS", "ERR BAXIS")),
        ("BDIR -1 -1 -1", ("OK BDIR", "ERR BDIR")),
        ("U 6", ("OK LIMIT", "ERR")),
        ("L 330", ("OK LIMIT", "ERR")),
        ("BU 6", ("OK LIMIT", "ERR")),
        ("BTRIP 20", ("OK BTRIP", "ERR")),
        ("BCAP 0.5", ("OK BCAP", "ERR")),
        ("BTRIM 0", ("OK BTRIM", "ERR")),
        ("BFULL 250", ("OK BFULL", "ERR")),
    ]:
        must(cmd, exp)

    meta_path = os.path.join(out_dir, "compare_meta.csv")
    with open(meta_path, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(["scheme", "kp", "kd", "kw", "ki",
                                "balance_s", "frames", "result"])

    for idx, (name, kp, kd, kw, ki) in enumerate(SCHEMES, 1):
        print(f"\n===== 第 {idx}/3 轮: {name} (Kp={kp:g}, Kd={kd:g}, Kw={kw:g}) =====", flush=True)
        send("STOP", 1.0, ("OK STOP", "ERR"))
        time.sleep(0.3)
        must("APP 1", ("OK APP", "ERR APP"))
        r = must("BREF %.9f %.9f %.9f" % REF, ("OK BREF", "ERR BREF"), 4.0)
        if not r or r.startswith("ERR"):
            print("BREF 失败, 中止", flush=True); break
        r = must("BNEUTRAL 1 0.00025 0.5 5 %.2f %.2f" % SEED,
                 ("OK BNEUTRAL", "ERR BNEUTRAL"), 3.0)
        if not r or r.startswith("ERR"):
            print("BNEUTRAL 失败, 中止", flush=True); break
        for i in range(1, 4):
            must(f"LQR{i} {kp:g} {kd:g} {kw:g} {ki:g}", (f"OK LQR{i}", "ERR"))
        st = find("$STATE")
        if st:
            print("      NPit=%s NRol=%s NOn=%s" % (
                field(st, "NPit"), field(st, "NRol"), field(st, "NOn")), flush=True)

        with lock:
            rows.clear()
        t0 = time.perf_counter() * 1000.0
        r = must("BAUTO 1 2", ("OK BAUTO", "ERR BAUTO"), 4.0)
        if not r or r.startswith("ERR"):
            print("BAUTO 失败, 中止", flush=True); break
        print(f"  请扶到 PG8 常亮 -> PG6 亮起瞬间松手; 平衡约 10 s 时轻敲一次", flush=True)

        start_ms = None
        deadline = time.time() + wait_seconds
        last_print = 0.0
        while time.time() < deadline:
            tele = find("$TELE")
            if tele and field(tele, "Bal") == 3:
                start_ms = time.perf_counter() * 1000.0
                print("  检测到 Bal=3, 记录 %.0f s" % bal_seconds, flush=True)
                break
            if time.time() - last_print > 3.0:
                last_print = time.time()
                s2 = find("$STATE")
                if s2:
                    print("    等待入位: Tilt=%s Ready=%s Bal=%s" % (
                        field(s2, "Tilt"), field(s2, "EdgeReady"), field(s2, "Bal")), flush=True)
            time.sleep(0.1)

        result = "ok"
        if start_ms is None:
            print("  超时: 未入位", flush=True)
            result = "timeout"
        else:
            end = start_ms + bal_seconds * 1000.0
            while time.perf_counter() * 1000.0 < end:
                time.sleep(0.1)

        try:
            ser.write(b"STOP\r\n"); ser.flush()
        except serial.SerialException:
            pass
        time.sleep(0.3)

        all_rows = latest()
        out_csv = os.path.join(out_dir, f"compare_{name}.csv")
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_local_ms", "t_rel_ms", "raw"])
            for ts, s in all_rows:
                w.writerow([f"{ts:.1f}", f"{ts - t0:.1f}", s])
        tele = sum(1 for _, s in all_rows if s.startswith("$TELE"))
        print(f"  saved {out_csv} ({tele} $TELE)", flush=True)
        with open(meta_path, "a", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow([name, kp, kd, kw, ki,
                                    bal_seconds if result == "ok" else 0,
                                    tele, result])
        if result == "timeout":
            break

    stop.set(); time.sleep(0.2)
    ser.close()
    print("\n全部完成", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
