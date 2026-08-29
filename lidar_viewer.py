#!/usr/bin/env python3
"""
Real-time point-cloud field viewer for the LDROBOT D800 (STL-27L) 360 LiDAR.

Dark-themed, dependency-light (pyserial + pygame only). Parses the standard
LDROBOT LD-series UART packet format and renders a live top-down scan field.

Usage:
    python lidar_viewer.py --port /dev/ttyUSB0            # real device
    python lidar_viewer.py --port COM5 --baud 921600
    python lidar_viewer.py --demo                         # no hardware needed

Controls:
    mouse wheel / + -   zoom
    left-drag / arrows  pan
    C                   cycle color mode (distance / intensity / mono)
    G                   toggle range-ring grid
    P                   pause rendering
    R                   reset view
    ESC / Q             quit
"""

import argparse
import math
import struct
import sys
import threading
import time
from collections import deque

# --- LDROBOT LD-series packet layout -----------------------------------------
HEADER = 0x54       # every packet starts with this
VER_LEN = 0x2C      # version/length byte: 12 measurement points per packet
POINTS_PER_PACKET = 12
PACKET_LEN = 47     # 1+1+2+2 + 12*3 + 2+2+1
DEFAULT_BAUD = 921600  # STL-27L / D800 default; LD06/LD19 use 230400


def build_crc_table():
    """LDROBOT CRC-8: MSB-first, polynomial 0x4D, init 0, no reflection."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = build_crc_table()


def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def parse_packet(buf):
    """Decode one 47-byte packet -> (rpm, [(angle_deg, dist_mm, intensity), ...]).

    Returns None if the CRC check fails.
    """
    if crc8(buf[:PACKET_LEN - 1]) != buf[PACKET_LEN - 1]:
        return None

    speed = struct.unpack_from("<H", buf, 2)[0]        # deg / second
    start = struct.unpack_from("<H", buf, 4)[0]        # 0.01 deg
    end = struct.unpack_from("<H", buf, 42)[0]         # 0.01 deg

    diff = (end - start) % 36000
    step = diff / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

    pts = []
    for i in range(POINTS_PER_PACKET):
        off = 6 + i * 3
        dist = struct.unpack_from("<H", buf, off)[0]   # mm
        intensity = buf[off + 2]
        angle = ((start + step * i) % 36000) / 100.0    # degrees
        pts.append((angle, dist, intensity))

    rpm = speed / 6.0  # deg/s -> rpm
    return rpm, pts


# --- Shared scan field -------------------------------------------------------
class ScanField:
    """Latest distance per 0.1-degree bin, updated in place from any source."""

    BINS = 3600

    def __init__(self):
        self._lock = threading.Lock()
        self._dist = [0] * self.BINS       # mm, 0 == no return
        self._inten = [0] * self.BINS
        self.rpm = 0.0
        self.packets = 0

    def update(self, points, rpm=None):
        with self._lock:
            for angle, dist, intensity in points:
                if dist <= 0:
                    continue
                b = int(round(angle * 10)) % self.BINS
                self._dist[b] = dist
                self._inten[b] = intensity
            if rpm is not None:
                self.rpm = rpm
            self.packets += 1

    def snapshot(self):
        """Return list of (angle_deg, dist_mm, intensity) for valid bins."""
        with self._lock:
            out = []
            for b in range(self.BINS):
                d = self._dist[b]
                if d > 0:
                    out.append((b / 10.0, d, self._inten[b]))
            return out, self.rpm


# --- Data sources ------------------------------------------------------------
class SerialSource(threading.Thread):
    """Reads the LiDAR serial stream, syncs on the header, feeds ScanField."""

    def __init__(self, field, port, baud):
        super().__init__(daemon=True)
        self.field = field
        self.port = port
        self.baud = baud
        self.running = True
        self.crc_errors = 0

    def run(self):
        try:
            import serial  # pyserial
        except ImportError:
            print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
            return
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:  # noqa: BLE001
            print(f"Could not open {self.port}: {e}", file=sys.stderr)
            return

        buf = bytearray()
        while self.running:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf.extend(chunk)

            # Scan the buffer for valid header pairs and parse full packets.
            i = 0
            while i + PACKET_LEN <= len(buf):
                if buf[i] == HEADER and buf[i + 1] == VER_LEN:
                    result = parse_packet(buf[i:i + PACKET_LEN])
                    if result is not None:
                        rpm, pts = result
                        self.field.update(pts, rpm)
                        i += PACKET_LEN
                        continue
                    self.crc_errors += 1
                i += 1
            del buf[:i]
        ser.close()

    def stop(self):
        self.running = False


class DemoSource(threading.Thread):
    """Synthetic room so the viewer runs without hardware (--demo)."""

    WALLS = (3000, 2200)  # half-extents in mm (x, y)
    OBSTACLES = ((1200, -600, 350), (-1500, 900, 500), (400, 1400, 250))

    def __init__(self, field):
        super().__init__(daemon=True)
        self.field = field
        self.running = True
        self.cursor = 0.0

    def _ray(self, theta):
        dx, dy = math.cos(theta), math.sin(theta)
        best = 1e9
        # axis-aligned box walls (ray from origin)
        xh, yh = self.WALLS
        if dx > 1e-9:
            best = min(best, xh / dx)
        elif dx < -1e-9:
            best = min(best, -xh / dx)
        if dy > 1e-9:
            best = min(best, yh / dy)
        elif dy < -1e-9:
            best = min(best, -yh / dy)
        # circular obstacles
        for cx, cy, r in self.OBSTACLES:
            b = -(dx * cx + dy * cy)
            c = cx * cx + cy * cy - r * r
            disc = b * b - c
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if 0 < t < best:
                    best = t
        # deterministic jitter (no RNG import needed)
        best += math.sin(theta * 53.0) * 8.0
        return max(0.0, best)

    def run(self):
        while self.running:
            pts = []
            for _ in range(POINTS_PER_PACKET):
                self.cursor = (self.cursor + 0.35) % 360.0
                theta = math.radians(self.cursor)
                dist = self._ray(theta)
                inten = 200 if dist < 2500 else 90
                pts.append((self.cursor, dist, inten))
            self.field.update(pts, rpm=600.0)
            time.sleep(0.0015)

    def stop(self):
        self.running = False


# --- Rendering ---------------------------------------------------------------
BG = (9, 11, 15)
GRID = (30, 36, 46)
GRID_TXT = (70, 82, 100)
HUD = (150, 165, 185)
ACCENT = (0, 230, 170)
SENSOR = (255, 90, 90)

COLOR_MODES = ("distance", "intensity", "mono")


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def color_for(dist, inten, max_range, mode):
    if mode == "mono":
        return ACCENT
    if mode == "intensity":
        t = min(1.0, inten / 255.0)
        return lerp((40, 60, 120), (0, 255, 190), t)
    t = min(1.0, dist / max_range)  # near -> cyan/green, far -> deep blue
    return lerp((0, 255, 180), (40, 80, 210), t)


def run_viewer(field, args, source):
    import pygame

    pygame.init()
    pygame.display.set_caption("D800 LiDAR - point cloud field")
    flags = pygame.RESIZABLE
    screen = pygame.display.set_mode((args.width, args.height), flags)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo,consolas,monospace", 13)
    big = pygame.font.SysFont("menlo,consolas,monospace", 15, bold=True)

    max_range = args.max_range * 1000.0  # m -> mm
    zoom = 1.0
    pan = [0.0, 0.0]
    dragging = False
    last_mouse = (0, 0)
    show_grid = True
    paused = False
    color_mode = 0

    frame_times = deque(maxlen=30)
    frozen = None  # last snapshot kept while paused

    def base_scale(w, h):
        return (min(w, h) / 2 - 40) / max_range

    running = True
    while running:
        w, h = screen.get_size()
        cx = w / 2 + pan[0]
        cy = h / 2 + pan[1]
        scale = base_scale(w, h) * zoom

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif e.key == pygame.K_g:
                    show_grid = not show_grid
                elif e.key == pygame.K_p:
                    paused = not paused
                elif e.key == pygame.K_c:
                    color_mode = (color_mode + 1) % len(COLOR_MODES)
                elif e.key == pygame.K_r:
                    zoom, pan[0], pan[1] = 1.0, 0.0, 0.0
                elif e.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    zoom *= 1.15
                elif e.key == pygame.K_MINUS:
                    zoom /= 1.15
                elif e.key == pygame.K_LEFT:
                    pan[0] += 25
                elif e.key == pygame.K_RIGHT:
                    pan[0] -= 25
                elif e.key == pygame.K_UP:
                    pan[1] += 25
                elif e.key == pygame.K_DOWN:
                    pan[1] -= 25
            elif e.type == pygame.MOUSEWHEEL:
                zoom *= 1.1 ** e.y
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                dragging = True
                last_mouse = e.pos
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                dragging = False
            elif e.type == pygame.MOUSEMOTION and dragging:
                pan[0] += e.pos[0] - last_mouse[0]
                pan[1] += e.pos[1] - last_mouse[1]
                last_mouse = e.pos

        zoom = max(0.1, min(zoom, 40.0))
        screen.fill(BG)

        # range-ring grid
        if show_grid:
            step_m = 1
            r = step_m
            while r * 1000 <= max_range:
                rr = int(r * 1000 * scale)
                if rr > 6:
                    pygame.draw.circle(screen, GRID, (int(cx), int(cy)), rr, 1)
                    label = font.render(f"{r}m", True, GRID_TXT)
                    screen.blit(label, (int(cx) + 3, int(cy) - rr - 2))
                r += step_m
            pygame.draw.line(screen, GRID, (0, int(cy)), (w, int(cy)), 1)
            pygame.draw.line(screen, GRID, (int(cx), 0), (int(cx), h), 1)

        # point field (freeze the last snapshot while paused)
        if paused and frozen is not None:
            pts, rpm = frozen
        else:
            pts, rpm = field.snapshot()
            frozen = (pts, rpm)
        mode = COLOR_MODES[color_mode]
        drawn = 0
        for angle, dist, inten in pts:
            if dist > max_range:
                continue
            th = math.radians(angle)
            x = cx + dist * scale * math.cos(th)
            y = cy - dist * scale * math.sin(th)
            if -5 <= x <= w + 5 and -5 <= y <= h + 5:
                screen.fill(color_for(dist, inten, max_range, mode),
                            (int(x), int(y), 2, 2))
                drawn += 1

        # sensor marker
        pygame.draw.circle(screen, SENSOR, (int(cx), int(cy)), 4)

        # HUD
        frame_times.append(time.time())
        fps = 0.0
        if len(frame_times) > 1:
            fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0] + 1e-9)

        title = "DEMO" if isinstance(source, DemoSource) else args.port
        lines = [
            (big, f"D800 LiDAR  [{title}]", ACCENT),
            (font, f"points   {drawn}", HUD),
            (font, f"rpm      {rpm:6.1f}", HUD),
            (font, f"fps      {fps:5.1f}", HUD),
            (font, f"zoom     {zoom:4.2f}x", HUD),
            (font, f"color    {mode}", HUD),
            (font, f"range    {args.max_range:.0f} m", HUD),
        ]
        if isinstance(source, SerialSource) and source.crc_errors:
            lines.append((font, f"crc err  {source.crc_errors}", (200, 120, 90)))
        if paused:
            lines.append((font, "-- PAUSED --", (240, 200, 90)))

        y = 10
        for fnt, text, col in lines:
            screen.blit(fnt.render(text, True, col), (12, y))
            y += fnt.get_height() + 2

        help_txt = "wheel zoom  drag pan  C color  G grid  P pause  R reset  Q quit"
        screen.blit(font.render(help_txt, True, GRID_TXT), (12, h - 22))

        pygame.display.flip()
        clock.tick(args.fps)

    source.stop()
    pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="LDROBOT D800 real-time point-cloud viewer")
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="baud rate")
    ap.add_argument("--demo", action="store_true", help="run with synthetic data")
    ap.add_argument("--max-range", type=float, default=12.0, help="display range in meters")
    ap.add_argument("--width", type=int, default=900)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--fps", type=int, default=60, help="render frame cap")
    args = ap.parse_args()

    if not args.demo and not args.port:
        ap.error("specify --port <device> or use --demo")

    field = ScanField()
    source = DemoSource(field) if args.demo else SerialSource(field, args.port, args.baud)
    source.start()

    try:
        run_viewer(field, args, source)
    except KeyboardInterrupt:
        source.stop()


if __name__ == "__main__":
    main()
