#!/usr/bin/env python3

import socket
import struct
import math
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ================================================================
# CONFIGURATION
# ================================================================
MAX_RANGE = 30.0 # According to datasheet Max_range is 25 meters
STX = "\x02"
ETX = "\x03"
# Front Lidar and Rear Lidar
LIDARS = [
    {
        "ip": "195.37.48.222",
        "port": 2111,
        "name": "Front LiDAR",
        "yaw_deg": 0.0,
        "color": (0, 200, 255)
    },
    {
        "ip": "195.37.48.223",
        "port": 2111,
        "name": "Rear LiDAR",
        "yaw_deg": 180.0,
        "color": (255, 120, 0)
    }
]

# ================================================================
# PARSE LMDscandata
# ================================================================
def parse_lmd_scandata(block):
    parts = block.strip().split()
    if b"LMDscandata" not in parts or b"DIST1" not in parts:
        return None

    i = parts.index(b"DIST1")
    scale = struct.unpack('>f', bytes.fromhex(parts[i + 1].decode()))[0]
    start = int(parts[i + 3], 16) / 10000.0
    step  = int(parts[i + 4], 16) / 10000.0
    count = int(parts[i + 5], 16)

    xs, ys = [], []
    for k, h in enumerate(parts[i + 6:i + 6 + count]):
        raw = int(h, 16)
        if raw > 0:
            r = (raw * scale) / 1000.0
            a = math.radians(start + k * step)
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))
    return xs, ys


def rotate(xs, ys, yaw_deg):
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        [c * x - s * y for x, y in zip(xs, ys)],
        [s * x + c * y for x, y in zip(xs, ys)]
    )

# ================================================================
# STYLE CIRCULAT FAN VIEW (CARTESIAN)
# ================================================================
def add_circular_fan(plot):
    FOV_MIN, FOV_MAX = -45, 225
    R_MIN, R_MAX = 0.5, 25.0

    angles = np.linspace(
        math.radians(FOV_MIN),
        math.radians(FOV_MAX),
        500
    )

    xo = [R_MAX * math.cos(a) for a in angles]
    yo = [R_MAX * math.sin(a) for a in angles]
    xi = [R_MIN * math.cos(a) for a in reversed(angles)]
    yi = [R_MIN * math.sin(a) for a in reversed(angles)]

    plot.addItem(pg.PlotDataItem(
        xo + xi + [xo[0]],
        yo + yi + [yo[0]],
        pen=None,
        brush=pg.mkBrush(200, 200, 200, 40)
    ))

    for r in range(5, int(R_MAX) + 1, 5):
        plot.plot(
            [r * math.cos(a) for a in angles],
            [r * math.sin(a) for a in angles],
            pen=pg.mkPen((160, 160, 160), width=1, style=QtCore.Qt.DotLine)
        )

    for deg in (FOV_MIN, FOV_MAX):
        a = math.radians(deg)
        plot.plot(
            [0, R_MAX * math.cos(a)],
            [0, R_MAX * math.sin(a)],
            pen=pg.mkPen((200, 200, 200), width=2)
        )

    plot.plot([0], [0], pen=None, symbol='o', symbolSize=6, symbolBrush='w')

# ================================================================
# LIDAR CLIENT (ROBUST)
# ================================================================
class LidarClient(QtCore.QObject):
    def __init__(self, cfg, scatter):
        super().__init__()
        self.cfg = cfg
        self.scatter = scatter

        self.sock = None
        self.buffer = b""

        self.connected = False
        self.error_msg = None
        self.frames = 0
        self.start_time = time.time()

        self.init_connection()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(20)

    def init_connection(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.cfg["ip"], self.cfg["port"]))

            for cmd in ("sMN LMCstartmeas", "sMN Run", "sEN LMDscandata 1"):
                self.sock.sendall(f"{STX}{cmd}{ETX}".encode())
                time.sleep(0.1)

            self.sock.settimeout(0.001)
            self.connected = True
            print(f"[CONNECTED] {self.cfg['name']} Active.")

        except Exception as e:
            self.connected = False
            self.error_msg = str(e)
            print(f"[FAILED] {self.cfg['name']} at {self.cfg['ip']}: {e}")

    def poll(self):
        if not self.connected:
            return

        try:
            while True:
                self.buffer += self.sock.recv(65535)
        except:
            pass

        while STX.encode() in self.buffer and ETX.encode() in self.buffer:
            s = self.buffer.find(STX.encode())
            e = self.buffer.find(ETX.encode(), s)
            telegram = self.buffer[s + 1:e]
            self.buffer = self.buffer[e + 1:]

            parsed = parse_lmd_scandata(telegram)
            if parsed:
                self.frames += 1
                x, y = rotate(parsed[0], parsed[1], self.cfg["yaw_deg"])
                self.scatter.setData(x, y)

# ================================================================
# MAIN
# ================================================================
def main():
    app = QtWidgets.QApplication([])

    win = pg.GraphicsLayoutWidget(
        show=True,
        title="SICK TiM781 – Circular Fan View (Front & Rear Separate)"
    )
    win.resize(1200, 600)

    clients = []

    for i, cfg in enumerate(LIDARS):
        plot = win.addPlot(row=0, col=i, title=cfg["name"])
        plot.setAspectLocked(True)
        plot.setXRange(-MAX_RANGE, MAX_RANGE)
        plot.setYRange(-MAX_RANGE, MAX_RANGE)
        plot.setLabel('bottom', "X (m)")
        plot.setLabel('left', "Y (m)")
        plot.showGrid(x=True, y=True, alpha=0.4)
        plot.hideAxis('top')
        plot.hideAxis('right')  

        add_circular_fan(plot)

        scatter = plot.plot(
            pen=None,
            symbol='o',
            symbolSize=2,
            symbolBrush=cfg["color"]
        )

        clients.append(LidarClient(cfg, scatter))

    def cleanup():
        print("\n" + "=" * 48)
        print(f"{'FINAL SESSION SUMMARY':^48}")
        print("=" * 48)

        for c in clients:
            print(f"Device: {c.cfg['name']} [{c.cfg['ip']}]")
            if not c.connected:
                print("Status: DISCONNECTED")
                print(f"Reason: {c.error_msg}")
                print("Frames: 0")
                print("Avg Rate: N/A")
            else:
                duration = time.time() - c.start_time
                hz = c.frames / duration if duration > 0 else 0
                print("Status: CONNECTED")
                print(f"Frames: {c.frames}")
                print(f"Avg Rate: {hz:.2f} Hz ({int(hz * 60)} FPM)")
            print("-" * 48)

            if c.sock and c.connected:
                try:
                    c.sock.sendall(f"{STX}sEN LMDscandata 0{ETX}".encode())
                    c.sock.close()
                except:
                    pass

        print("Process Terminated.")

    app.aboutToQuit.connect(cleanup)
    app.exec_()

if __name__ == "__main__":
    main()