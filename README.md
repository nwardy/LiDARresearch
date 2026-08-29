# LiDARresearch

Live viewer for the LDROBOT D800 (STL-27L) 360 lidar. It reads the sensor's
serial packets and draws the scan top-down. Only needs pyserial and pygame.

## Setup

```
pip install -r requirements.txt
```

## Running

```
python lidar_viewer.py --port /dev/ttyUSB0
python lidar_viewer.py --port COM5 --baud 921600
```

The D800 runs at 921600 baud by default. If you get no points try 230400.

No sensor handy? Run the fake room:

```
python lidar_viewer.py --demo
```

Other flags: `--max-range` (display range in meters), `--width`, `--height`, `--fps`.

## Keys

- wheel or +/- : zoom
- drag or arrows : pan
- C : color mode (distance / intensity / mono)
- G : grid on/off
- P : pause
- R : reset view
- Q or Esc : quit

## Packet format

Standard LDROBOT LD-series frame: 47 bytes, starts with `0x54 0x2C`, 12 points
per packet, crc8 (poly 0x4D) on the last byte.
