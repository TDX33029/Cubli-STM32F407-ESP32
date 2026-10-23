#!/usr/bin/env python3
"""Cubli-F407ZGT6 串口示波器客户端。
功能：向 STM32 (COM10 @ 115200, DEBUG_LINK_USART=USART2) 发送命令，
      在后台保活 PING，并把 $TELE/$FAST 遥测行连同本地时间戳落盘 CSV。
用法:
    py scope_client.py PORT CMDFILE OUT.csv [--baud 115200]
CMDFILE 每行一条命令；以 "WAIT:ms" 开头的行表示休眠；"PING" 行会在整个会话期内
        以 200ms 周期自动发送，无需在文件中反复写。
"""
import csv, sys, threading, time
import serial

STOP = object()

def main():
    if len(sys.argv) < 4:
        print(__doc__); return 1
    port, cmdfile, out_csv = sys.argv[1], sys.argv[2], sys.argv[3]
    baud = int(sys.argv[4]) if len(sys.argv) > 4 else 115200

    ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.5)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # 命令队列
    q = []
    with open(cmdfile, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.upper().startswith("WAIT:"):
                q.append(("WAIT", float(line[5:]) / 1000.0))
            else:
                q.append(("CMD", line))

    log_lock = threading.Lock()
    rows = []  # (t_ms, line)

    def reader():
        buf = b""
        while not stop_evt.is_set():
            try:
                data = ser.read(256)
            except serial.SerialException:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip(b"\r")
                if not line:
                    continue
                t = time.perf_counter() * 1000.0
                s = line.decode("utf-8", "replace")
                with log_lock:
                    rows.append((t, s))
                print(f"[{t:9.1f}ms] {s}")

    def pinger():
        while not stop_evt.is_set():
            stop_evt.wait(0.2)
            if stop_evt.is_set():
                break
            try:
                ser.write(b"PING\r\n")
            except serial.SerialException:
                break

    stop_evt = threading.Event()
    t_reader = threading.Thread(target=reader, daemon=True)
    t_ping = threading.Thread(target=pinger, daemon=True)
    t_reader.start()
    t_ping.start()

    t0 = time.perf_counter()
    try:
        for kind, val in q:
            if kind == "WAIT":
                time.sleep(val)
                continue
            cmd = val
            ser.write((cmd + "\r\n").encode())
            ser.flush()
            time.sleep(0.05)
    finally:
        stop_evt.set()
        t_reader.join(timeout=2)
        t_ping.join(timeout=2)
        try:
            ser.write(b"STOP\r\n"); ser.flush(); time.sleep(0.2)
        except serial.SerialException:
            pass
        ser.close()

    with log_lock:
        all_rows = rows[:]
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_local_ms", "t_rel_ms", "raw"])
        for t, s in all_rows:
            w.writerow([f"{t:.1f}", f"{t - t0 * 1000:.1f}", s])
    print(f"Saved {len(all_rows)} telemetry lines -> {out_csv}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
