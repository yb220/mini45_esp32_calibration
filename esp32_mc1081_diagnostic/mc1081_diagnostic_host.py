from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - depends on local environment
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


BASE_DIR = Path(__file__).resolve().parent
CAPTURE_DIR = BASE_DIR / "captures"
DEFAULT_BAUD = 115200

DIAG_FIELDS = [
    "pc_timestamp",
    "pc_monotonic_s",
    "elapsed_s",
    "esp_ms",
    "seq",
    "valid",
    "error",
    "status",
    "overflow",
    "dref",
    "d0",
    "d1",
    "d2",
    "d3",
    "d4",
    "c0",
    "c1",
    "c2",
    "c3",
    "c4",
    "dt_us",
    "raw_line",
]


@dataclass
class RunConfig:
    port: str
    baud: int
    rate_hz: int
    duration_s: float
    cavg: int
    discard: int
    i2c_hz: int
    label: str
    reset: bool


def timestamp_for_file() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def timestamp_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def safe_label(text: str) -> str:
    invalid = '<>:"/\\|?*'
    cleaned = "".join("_" if char in invalid or ord(char) < 32 else char for char in text.strip())
    cleaned = "_".join(part for part in cleaned.strip(" ._").split() if part)
    return cleaned[:80] or "mc1081_diag"


def list_serial_ports() -> None:
    require_serial()
    ports = list(list_ports.comports())
    if not ports:
        print("未发现串口")
        return
    for port in ports:
        desc = port.description or ""
        hwid = port.hwid or ""
        print(f"{port.device}\t{desc}\t{hwid}")


def parse_diag_line(line: str) -> dict | None:
    parts = line.strip().split(",")
    if len(parts) != 19 or parts[0] != "DIAG":
        return None
    try:
        return {
            "esp_ms": int(parts[1]),
            "seq": int(parts[2]),
            "valid": int(parts[3]),
            "error": int(parts[4]),
            "status": int(parts[5]),
            "overflow": int(parts[6]),
            "dref": int(parts[7]),
            "d0": int(parts[8]),
            "d1": int(parts[9]),
            "d2": int(parts[10]),
            "d3": int(parts[11]),
            "d4": int(parts[12]),
            "c0": float(parts[13]),
            "c1": float(parts[14]),
            "c2": float(parts[15]),
            "c3": float(parts[16]),
            "c4": float(parts[17]),
            "dt_us": int(parts[18]),
        }
    except ValueError:
        return None


def write_command(ser: serial.Serial, command: str, delay_s: float = 0.08) -> None:
    line = command.strip()
    if not line:
        return
    ser.write((line + "\n").encode("ascii"))
    ser.flush()
    time.sleep(delay_s)


def build_output_dir(label: str) -> Path:
    run_dir = CAPTURE_DIR / f"{timestamp_for_file()}_{safe_label(label)}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def configure_device(ser: serial.Serial, config: RunConfig) -> None:
    # ESP32 打开串口后常会复位，先短暂等待启动日志输出。
    time.sleep(1.2)
    write_command(ser, "STOP")
    if config.reset:
        write_command(ser, "RESET", delay_s=0.3)
    write_command(ser, f"I2C,{config.i2c_hz}")
    write_command(ser, f"CAVG,{config.cavg}")
    write_command(ser, f"DISCARD,{config.discard}")
    write_command(ser, "INFO")


def run_capture(config: RunConfig) -> Path:
    require_serial()
    run_dir = build_output_dir(config.label)
    raw_csv = run_dir / "mc1081_diagnostic_raw.csv"
    log_path = run_dir / "serial_log.txt"
    meta_path = run_dir / "metadata.json"

    metadata = {
        "created_at": timestamp_iso(),
        "script": str(Path(__file__).resolve()),
        "output_dir": str(run_dir),
        "config": asdict(config),
        "csv_fields": DIAG_FIELDS,
    }
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"输出目录：{run_dir}")
    print("正在连接串口...")

    diag_count = 0
    invalid_count = 0
    start_s = time.monotonic()
    last_print_s = start_s

    with serial.Serial(config.port, config.baud, timeout=0.2) as ser, \
            raw_csv.open("w", newline="", encoding="utf-8-sig") as csv_file, \
            log_path.open("w", encoding="utf-8") as log_file:
        writer = csv.DictWriter(csv_file, fieldnames=DIAG_FIELDS)
        writer.writeheader()

        configure_device(ser, config)
        write_command(ser, f"START,{config.rate_hz}")
        print("开始采集，按 Ctrl+C 可提前停止。")

        try:
            while True:
                now_s = time.monotonic()
                elapsed_s = now_s - start_s
                if config.duration_s > 0 and elapsed_s >= config.duration_s:
                    break

                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                pc_ts = timestamp_iso()
                log_file.write(f"{pc_ts}\t{line}\n")

                parsed = parse_diag_line(line)
                if parsed is None:
                    continue

                diag_count += 1
                if parsed["valid"] != 1:
                    invalid_count += 1

                row = {
                    "pc_timestamp": pc_ts,
                    "pc_monotonic_s": f"{now_s:.6f}",
                    "elapsed_s": f"{elapsed_s:.6f}",
                    "raw_line": line,
                    **parsed,
                }
                writer.writerow(row)

                if now_s - last_print_s >= 5.0:
                    last_print_s = now_s
                    print(
                        f"{elapsed_s:8.1f}s  rows={diag_count}  invalid={invalid_count}  "
                        f"C0={parsed['c0']:.6f} C1={parsed['c1']:.6f} "
                        f"C2={parsed['c2']:.6f} C3={parsed['c3']:.6f} C4={parsed['c4']:.6f}"
                    )
        except KeyboardInterrupt:
            print("\n用户停止采集")
        finally:
            try:
                write_command(ser, "STOP", delay_s=0.02)
            except Exception:
                pass
            csv_file.flush()
            log_file.flush()

    metadata["finished_at"] = timestamp_iso()
    metadata["diag_count"] = diag_count
    metadata["invalid_count"] = invalid_count
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"采集结束：有效输出 {diag_count} 行，异常行 {invalid_count} 行")
    print(f"CSV：{raw_csv}")
    print(f"日志：{log_path}")
    return run_dir


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MC1081 诊断固件配套上位机")
    parser.add_argument("--list", action="store_true", help="列出可用串口后退出")
    parser.add_argument("--port", help="ESP32 串口，例如 COM6")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="串口波特率，默认 115200")
    parser.add_argument("--rate", type=int, default=10, help="连续采集频率 Hz，建议 5~20，默认 10")
    parser.add_argument("--duration", type=float, default=600.0, help="采集时长 s，默认 600；0 表示一直采到 Ctrl+C")
    parser.add_argument("--cavg", type=int, choices=[1, 4], default=4, help="MC1081 内部平均次数，默认 4")
    parser.add_argument("--discard", type=int, default=1, help="正式读数前丢弃转换次数，默认 1")
    parser.add_argument("--i2c", type=int, default=100000, help="I2C 频率 Hz，默认 100000")
    parser.add_argument("--label", default="mc1081_diag", help="输出文件夹标签，例如 fixed_cap_c3")
    parser.add_argument("--reset", action="store_true", help="开始采集前重新初始化 MC1081")
    return parser.parse_args(argv)


def require_serial() -> None:
    if serial is None or list_ports is None:
        raise SystemExit("缺少 pyserial，请先安装：pip install pyserial")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("请指定 --port，例如：python mc1081_diagnostic_host.py --port COM6 --duration 600")
        print("可先运行：python mc1081_diagnostic_host.py --list")
        return 2

    config = RunConfig(
        port=args.port,
        baud=args.baud,
        rate_hz=max(1, min(int(args.rate), 100)),
        duration_s=float(args.duration),
        cavg=int(args.cavg),
        discard=max(0, min(int(args.discard), 10)),
        i2c_hz=int(args.i2c),
        label=args.label,
        reset=bool(args.reset),
    )
    run_capture(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
