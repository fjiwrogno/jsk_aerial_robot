# aerial_robot_rc

Standalone RC input driver. Reads a Futaba S.BUS receiver through a "SBUS to USB 2"
serial converter and publishes the raw channel values into ROS.

```
Futaba T10J ──(T-FHSS Air 2.4GHz)──> R3008SB (port 8/SB, S.BUS out)
    ──> SBUS to USB 2 converter (CH340 USB serial, 115200 8N1, 20 ms frames)
    ──> /dev/ttyUSB0 ──> sbus2usb_rc_node ──> rc/raw
```

This package is intentionally independent from `aerial_robot_base` /
`aerial_robot_control`; it is a pure hardware-facing input layer.

## Converter protocol (35-byte frame)

The converter decodes raw S.BUS in hardware and emits one 35-byte UART frame per
cycle (default 115200 baud 8N1, 20 ms):

| offset | size | content |
|--------|------|---------|
| 0      | 1    | header `0x0F` |
| 1-32   | 32   | 16 channels, `uint16` big endian, range 0-2047 |
| 33     | 1    | flags byte |
| 34     | 1    | XOR checksum over bytes 1..33 (header excluded) |

Flags byte (converter manual): `0x80`=ch17, `0x40`=ch18, `0x20`=frame lost,
`0x10`=failsafe. The example frame in the manual carries `0x0C` during normal
operation, so bit3/bit2 appear to be unused/reserved. **Confirm the bit
assignment on real hardware** (see "Verifying the failsafe bits" below); the
masks are parameters, not hardcoded.

## Run

```bash
roslaunch aerial_robot_rc sbus2usb_rc.launch port:=/dev/ttyUSB0
# under a robot namespace (for later flight integration):
roslaunch aerial_robot_rc sbus2usb_rc.launch robot_name:=gimbalrotor

rostopic hz /rc/raw          # expect ~50 Hz
rostopic echo /rc/raw
rosrun aerial_robot_rc rc_monitor.py              # curses channel monitor
rosrun aerial_robot_rc rc_monitor.py _topic:=/gimbalrotor/rc/raw
```

## Topics

| topic | type | description |
|-------|------|-------------|
| `rc/raw` | `aerial_robot_msgs/RcRaw` | one message per decoded frame (~50 Hz); during signal loss, republished at 20 Hz with `failsafe=true` and the last known channel values frozen |
| `rc/status` | `std_msgs/String` | 1 Hz human-readable counters (bytes, frames, decode errors, resyncs, failsafe state) |

## Parameters (see `config/sbus2usb.yaml`)

| param | default | description |
|-------|---------|-------------|
| `port` | `/dev/ttyUSB0` | serial device — set via the launch arg `port:=...` only (the launch file always overrides the yaml, so the key is intentionally absent there) |
| `baud_rate` | 115200 | converter default; changeable with the vendor SBUStoolkit |
| `poll_rate` | 200.0 | serial poll frequency [Hz] |
| `serial_timeout` | 0.5 | no valid frame within this period [s] -> publish `failsafe=true` at 20 Hz |
| `byte_timeout` | 1.0 | no serial bytes within this period [s] -> close and reopen the device (recovers USB unplug) |
| `failsafe_mask` | `0x10` | flags bit interpreted as receiver failsafe (must be within 0x01-0xFF; invalid values fall back to the default) |
| `frame_lost_mask` | `0x20` | flags bit interpreted as transient frame loss (same range check) |
| `status_rate` | 1.0 | `rc/status` frequency [Hz] |

## Failsafe behavior

`rc/raw.failsafe` becomes `true` when either:

1. the receiver reports failsafe in the flags byte (RF link lost while the
   converter keeps streaming), or
2. no valid frame arrived for `serial_timeout` seconds (converter unplugged,
   port dead, converter silent) — the node then keeps publishing at 20 Hz so
   downstream consumers always hear about the loss.

Unplugging and replugging the USB converter recovers automatically
(`byte_timeout` + 1 s reopen retry).

**Message contract for downstream consumers:**

- decide on the `failsafe` / `frame_lost` **bool** fields, never on `flags` —
  `flags` is the raw converter byte kept for debugging. On driver-generated
  link-loss frames the failsafe/frame_lost bits are forced on in `flags` too,
  so the two stay consistent, but the bools are the authoritative signal.
- during link loss the channel values are **frozen** at the last valid frame;
  until the first valid frame ever arrives they are **all zero**.
- channel values are guaranteed to be within 0-2047: frames carrying
  out-of-range values (misaligned data that passed the XOR by chance) are
  rejected by the parser.

## Verifying the failsafe bits (do once on real hardware)

1. `rostopic echo /rc/raw` with the transmitter ON: note the steady `flags` value.
2. Turn the transmitter OFF: note which bits change (and whether frames keep
   coming at all).
3. If a different bit than `0x10` is set, fix `failsafe_mask`/`frame_lost_mask`
   in `config/sbus2usb.yaml`.
4. If the converter freezes the last frame without any flag change, failsafe
   detection falls back to receiver F/S channel presets: configure the R3008SB
   failsafe positions (e.g. throttle low) on the transmitter and detect those
   values downstream.

Also record per-channel min/center/max and the switch positions with
`rc_monitor.py` (press `r` to reset the observed range) — these numbers feed the
calibration yaml of the dev-goal-2 mapping layer.

## Deployment notes (Khadas VIM4)

- add the user to the `dialout` group for `/dev/ttyUSB*` access
- suggested udev rule for a stable symlink (CH340):

  ```
  # /etc/udev/rules.d/99-rc-usb.rules
  SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="rc_usb"
  ```

  then launch with `port:=/dev/rc_usb`.

## Tests

```bash
catkin build --catkin-make-args run_tests -- --no-deps aerial_robot_rc
```

Unit tests cover frame parsing, checksum rejection, out-of-range channel
rejection, resync after noise, back-to-back frames, `0x0F` bytes inside the
payload, recovery after a corrupted frame and configurable flag masks.
