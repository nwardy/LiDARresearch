# LiDARresearch

Real-time point-cloud field viewer for the **LDROBOT D800 (STL-27L)** 360° dToF LiDAR.

Dark-themed and lightweight — the only dependencies are `pyserial` and `pygame`.
It parses the standard LDROBOT LD-series UART packet format (47-byte packets,
header `0x54 0x2C`, 12 points/packet, CRC-8 poly `0x4D`) and renders a live
top-down scan field.

## Install

```bash
pip install -r requirements.txt
```

## Run

```bash
# real device (D800/STL-27L default baud is 921600)
python lidar_viewer.py --port /dev/ttyUSB0
python lidar_viewer.py --port COM5 --baud 921600

# no hardware — synthetic room to try the UI
python lidar_viewer.py --demo
```

Useful flags: `--max-range <meters>` (display range), `--width/--height`, `--fps`.

## Controls

| key / input        | action                                   |
| ------------------ | ---------------------------------------- |
| mouse wheel / `+` `-` | zoom                                  |
| left-drag / arrows | pan                                      |
| `C`                | cycle color mode (distance / intensity / mono) |
| `G`                | toggle range-ring grid                   |
| `P`                | pause                                    |
| `R`                | reset view                               |
| `ESC` / `Q`        | quit                                     |
