# Bring-up checklist

Procedure to verify a ZED-X20D board, its antennas and this driver together, e.g. on a new
board, after a firmware update or before a campaign. Each step lists what to do and what
counts as a pass. Record every run in a bag and, where noted, with raw logging enabled
(`enable_raw_observables:=true`) so that `scripts/check_recording.py` can analyse it.

## Setup

- Mount both antennas on a rigid bar under open sky: GPS1 at one end, GPS2 at the other.
  Use the same antenna model and equal cable lengths. Measure the distance between the
  antenna centres with a tape.
- Mark four bar positions 90° apart on the ground (a carpenter's square or a 3-4-5
  triangle).
- Have a block of known height (e.g. 10 cm) to tilt the bar.
- Connect the POWER+GPS USB-C port to a root USB port or a powered hub. Behind
  bus-powered hubs the module can drop off the bus.
- Install the udev rule (see the README) and check that `/dev/ublox_x20d` exists.

## 1. Link

| Step | Pass |
|---|---|
| `lsusb -d 1546:` and `dmesg` while plugging in | One `1546:01ab` device, bound to `cdc_acm` as `/dev/ttyACM*`; symlink `/dev/ublox_x20d` present |
| `udevadm info /dev/ublox_x20d \| grep ID_MM` | `ID_MM_DEVICE_IGNORE=1` |
| Unplug and replug three times | The symlink comes back every time |

## 2. Start-up and configuration

| Step | Pass |
|---|---|
| `roslaunch ublox_x20d_driver ublox_x20d.launch device:=/dev/ublox_x20d` | Log shows `receiver MON-VER: MOD=ZED-X20D FWVER=HDG 2.00 ...` and `user configuration verified on device (N keys)`; no NAK or ERROR lines |
| `rostopic hz` on `nav_pvt`, `nav_hpposllh`, `nav_daheading` | About 1 Hz each; the `link` diagnostic shows at most about 1 NMEA sentence per second (`$GNTHS`) |
| `rate_meas_ms:=500` for 5 minutes, then back to 1000 | Topics at 2 Hz with `rel_pos_heading_valid` held; the heading re-fixes within seconds after going back |
| `receiver_heading_offset_deg:=90` against 0, bar still | `rel_pos_heading` changes by +90° (mod 360); `rel_pos_n`/`rel_pos_e` unchanged |
| `receiver_heading_offset_deg:=200` | The node refuses to start |
| Restart the driver five times in a row | Verified on the first attempt every time |
| Ctrl-C | The node exits within 2 s without an abort or core dump |
| `enable_raw_observables:=true raw_log_content:=rxm_only` | The `.ubx` file contains only `RXM-*` frames |

## 3. Parsing

Record 10 minutes static with raw logging and a bag of `/ublox_x20d/*` and
`/heading_imu/imu`, then run `check_recording.py` on both.

| Check | Pass |
|---|---|
| Field-by-field comparison of every NAV frame with its ROS message | No mismatches, no checksum errors, every NAV frame published |
| Payload lengths and versions | NAV-PVT 92 bytes, NAV-HPPOSLLH version 0, NAV-DAHEADING version 2; no related warning in the driver log |
| Internal consistency | `atan2(relPosE, relPosN)` matches the heading minus the receiver offset (< 0.05°); the norm of the baseline vector matches `rel_pos_length` (±1 mm); Imu yaw = π/2 − heading; `~fix` matches the high-precision position |

## 4. Heading and conventions

| Step | Pass |
|---|---|
| The static recording of section 3 | Share of epochs with `carr_soln = 2` and a valid heading; median heading accuracy about 0.6–0.9° for a 1 m baseline; measured baseline within 3 cm of the tape, standard deviation under 1 cm |
| Turn the bar through the four marks, clockwise seen from above, one minute each | The heading grows by 90° ± 3° per step and the Imu yaw decreases by π/2 per step |
| **Walk test:** carry the bar with GPS2 in front, in a straight line at 1 m/s or more, 30 s in each of two opposite directions | `head_mot` of `nav_pvt` agrees with `rel_pos_heading` within 5°; `~vel` points the same way. This is the absolute check: a swapped antenna pair shows as 180°, a wrong axis as 90° |
| Put the block under GPS2, then under GPS1 | Pitch ≈ −asin(h/L), then +asin(h/L), within 1° (positive pitch is nose down) |
| Cover GPS2 with a metal bowl or foil for a minute, then uncover | `rel_pos_heading_valid` (flags bit 6) drops, the Imu stops within two epochs, the `heading` diagnostic warns; note the time to re-fix |
| Replay the static bag with `expected_baseline_length_m` set to the tape length, then to it plus 0.2 m | No epochs dropped / every epoch dropped with a warning |

## 5. Position and RTK

| Step | Pass |
|---|---|
| The static recording without corrections | `~fix` status FIX or SBAS_FIX, horizontal accuracy under 5 m, covariance consistent with it |
| `ublox_x20d_ntrip.launch` with your caster | The client connects; the `link` diagnostic counts RTCM bytes; `diff_soln` set; `carr_soln` goes 1 → 2; `~fix` GBAS_FIX with horizontal accuracy under 3 cm; note the time to fix |
| 10 minutes static in RTK fixed | Standard deviation under 2 cm horizontal and 4 cm vertical |
| The heading checks of section 4 with corrections | Compare the fixed share and heading accuracy with the run without corrections |
| Interrupt the corrections for a minute | `~fix` degrades to float or FIX without driver errors and recovers when they return |

## 6. Robustness

| Step | Pass |
|---|---|
| Unplug and replug the USB cable while running | The log shows the port lost, reopened and the configuration verified again; topics resume within 5 s of replugging |
| Start the driver with the receiver unplugged, then plug it in | Retries without flooding the log and configures the receiver when it appears |
| 2 hours static with `raw_log_rotate_minutes:=10` | Stable memory use, no checksum errors, no iTOW gaps across rotated files |
| Leave the receiver unpowered for more than 12 hours, then start | Note which step of the configuration ladder (a, b, c or d) ends with the configuration verified |

## 7. Raw data and replay

| Step | Pass |
|---|---|
| `convbin -r ubx` on a recorded `.ubx` | `.obs` and `.nav` files with 1 Hz epochs and all tracked constellations |
| RTKLIB single-point solution against `~fix` of the same period | Agreement within a few metres |
| `replay_ubx.launch` of a recorded `.ubx`, recording a new bag | `check_recording.py` gives the same result as for the live recording |
