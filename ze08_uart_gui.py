#!/usr/bin/env python3
"""ZE08-CH2O UART monitor/control GUI.

- Read: 9-byte sensor frames, decode concentration/full-range.
- Write: send manual hex frames or predefined ZE08 commands.
"""

import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports


FRAME_LEN = 9


def calc_checksum(frame: bytes) -> int:
    total = sum(frame[1:8]) & 0xFF
    return ((~total + 1) & 0xFF)


def make_cmd(cmd: int, data: int = 0x00) -> bytes:
    payload = bytearray([0xFF, 0x01, cmd & 0xFF, data & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00])
    payload[8] = calc_checksum(payload)
    return bytes(payload)


def parse_frame(frame: bytes):
    if len(frame) != FRAME_LEN:
        raise ValueError("Invalid frame length")
    if frame[0] != 0xFF:
        raise ValueError("Invalid start byte")
    if calc_checksum(frame) != frame[8]:
        raise ValueError("Checksum mismatch")

    gas_id = frame[1]
    unit = frame[2]
    concentration = (frame[4] << 8) | frame[5]
    full_range = (frame[6] << 8) | frame[7]
    return {
        "gas_id": gas_id,
        "unit": unit,
        "concentration_ppb": concentration,
        "full_range_ppb": full_range,
    }


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ZE08-CH2O UART GUI")
        self.geometry("760x520")

        self.ser = None
        self.reader_thread = None
        self.running = False

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="9600")
        self.status_var = tk.StringVar(value="Disconnected")
        self.conc_var = tk.StringVar(value="-")
        self.range_var = tk.StringVar(value="-")

        self._build_ui()
        self.refresh_ports()

    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)

        ttk.Label(top, text="Port").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left", padx=4)

        ttk.Label(top, text="Baud").pack(side="left", padx=(10, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side="left", padx=6)

        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left", padx=4)

        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        info = ttk.LabelFrame(self, text="Decoded data")
        info.pack(fill="x", padx=10, pady=8)
        ttk.Label(info, text="Concentration (ppb):").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        ttk.Label(info, textvariable=self.conc_var).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Label(info, text="Full range (ppb):").grid(row=1, column=0, sticky="w", padx=8, pady=6)
        ttk.Label(info, textvariable=self.range_var).grid(row=1, column=1, sticky="w", padx=8)

        cmd = ttk.LabelFrame(self, text="Command")
        cmd.pack(fill="x", padx=10, pady=8)

        ttk.Button(cmd, text="Set QA mode (0x78 0x40)", command=lambda: self.send_bytes(make_cmd(0x78, 0x40))).pack(side="left", padx=6, pady=6)
        ttk.Button(cmd, text="Set active mode (0x78 0x41)", command=lambda: self.send_bytes(make_cmd(0x78, 0x41))).pack(side="left", padx=6)
        ttk.Button(cmd, text="Read once (0x86)", command=lambda: self.send_bytes(make_cmd(0x86, 0x00))).pack(side="left", padx=6)

        custom = ttk.Frame(self)
        custom.pack(fill="x", padx=10)
        ttk.Label(custom, text="Manual hex (e.g. FF 01 86 00 00 00 00 00 79):").pack(side="left")
        self.hex_entry = ttk.Entry(custom)
        self.hex_entry.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(custom, text="Send", command=self.send_manual_hex).pack(side="left")

        logf = ttk.LabelFrame(self, text="RX/TX log")
        logf.pack(fill="both", expand=True, padx=10, pady=8)
        self.log = tk.Text(logf, wrap="none")
        self.log.pack(fill="both", expand=True)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.2)
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            return

        self.running = True
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"Connected: {self.ser.port}")
        self.append_log("[INFO] connected")

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.status_var.set("Disconnected")
        self.append_log("[INFO] disconnected")

    def send_bytes(self, payload: bytes):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Not connected", "Serial port is not open")
            return
        self.ser.write(payload)
        self.append_log("[TX] " + payload.hex(" ").upper())

    def send_manual_hex(self):
        text = self.hex_entry.get().strip().replace(" ", "")
        try:
            payload = bytes.fromhex(text)
        except ValueError:
            messagebox.showerror("Invalid input", "Hex 문자열 형식이 올바르지 않습니다.")
            return
        self.send_bytes(payload)

    def reader_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(FRAME_LEN)
                if not data:
                    continue
                self.after(0, self.append_log, "[RX] " + data.hex(" ").upper())
                if len(data) == FRAME_LEN:
                    try:
                        parsed = parse_frame(data)
                        self.after(0, self.conc_var.set, str(parsed["concentration_ppb"]))
                        self.after(0, self.range_var.set, str(parsed["full_range_ppb"]))
                    except ValueError as exc:
                        self.after(0, self.append_log, f"[WARN] {exc}")
            except Exception as exc:
                self.after(0, self.append_log, f"[ERR] {exc}")
                time.sleep(0.3)

    def append_log(self, text: str):
        self.log.insert("end", text + "\n")
        self.log.see("end")


if __name__ == "__main__":
    app = App()
    app.mainloop()
