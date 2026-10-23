#!/usr/bin/env python3
"""IMU 校准对比采集: 通过 ST-Link 复位 -> (可选 CAL_IMU) -> 采集 30 s $TELE。

用法:
    py imu_capture.py COM10 out.csv [--calibrate] [--seconds 30]

流程:
    1. 打开串口并后台保活 PING;
    2. 调用 STM32_Programmer_CLI -rst 复位设备, 等待启动后的第一条 $TELE;
    3. --calibrate 时先发 CAL_IMU, 等待 OK CAL_IMU:DONE;
    4. 从时间零点(首帧 $TELE 或校准完成)起记录 N 秒;
    5. 保存 t_local_ms, t_rel_ms, raw 到 CSV (t_rel_ms 以时间零点为 0)。
"""
import csv, subprocess, sys, threading, time

import serial

CLI = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
SN = "190A071F7116303030303032"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) < 2:
        print(__doc__)
        return 1
    port, out_csv = args[0], args[1]
    do_cal = "--calibrate" in sys.argv
    seconds = 30.0
    for a in sys.argv:
        if a.startswith("--seconds="):
            seconds = float(a.split("=", 1)[1])

    ser = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    rows = []
    lock = threading.Lock()
    stop = threading.Event()

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

    def wait_for(pred, timeout_s):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with lock:
                snap = list(rows)
            hit = next(((ts, s) for ts, s in snap if pred(s)), None)
            if hit:
                return hit
            time.sleep(0.03)
        return None, None

    # ---- 复位 ----
    print("Resetting target via ST-Link...", flush=True)
    proc = subprocess.run(
        [CLI, "-c", "port=SWD", "sn=" + SN, "-rst"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=40,
    )
    out = proc.stdout.decode("gbk", "ignore")
    if proc.returncode != 0 or "Error" in out:
        print("Reset failed:\n" + out[-800:])
        stop.set(); ser.close()
        return 1

    # 丢弃复位前仍在管道/串口缓冲中的旧遥测, 只保留复位后的数据
    time.sleep(1.0)
    with lock:
        rows.clear()

    ts_first, line_first = wait_for(lambda s: s.startswith("$TELE"), 10)
    if ts_first is None:
        print("ERROR: no $TELE after reset")
        stop.set(); ser.close()
        return 1
    print("Boot OK, first $TELE: " + line_first[:80], flush=True)

    # ---- 可选校准 ----
    if do_cal:
        time.sleep(0.2)
        ser.write(b"CAL_IMU\r\n")
        ser.flush()
        ts_done, line_done = wait_for(lambda s: "OK CAL_IMU:DONE" in s, 15)
        if ts_done is None:
            print("ERROR: CAL_IMU timeout")
            stop.set(); ser.close()
            return 1
        t0 = ts_done
        print("CAL_IMU DONE at first data t=0", flush=True)
    else:
        t0 = ts_first
        print("Uncalibrated capture from first $TELE", flush=True)

    # ---- 采集 N 秒 ----
    deadline = t0 + seconds * 1000.0
    while time.perf_counter() * 1000.0 < deadline:
        time.sleep(0.05)

    stop.set()
    time.sleep(0.2)
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
