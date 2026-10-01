# ublox_x20d_driver

ROS Noetic driver for the u-blox **ZED-X20D** dual-antenna GNSS module, as fitted on the
ArduSimple **simpleRTK4 Dual**. It provides:

- RTK position as `sensor_msgs/NavSatFix` and velocity as `geometry_msgs/TwistWithCovarianceStamped`,
  with RTCM corrections forwarded from a ROS topic (e.g. an NTRIP client);
- the dual-antenna heading computed by the module (`UBX-NAV-DAHEADING`) as raw messages and
  as a `sensor_msgs/Imu` orientation;
- logging of raw observations (`RXM-RAWX`, `RXM-SFRBX`) to `.ubx` files for PPK
  post-processing with RTKLIB;
- replay of recorded `.ubx` files and bags without hardware, and a tool that checks a
  recording against its raw frames.

Message definitions live in the companion package `ublox_x20d_msgs`.

Requires **HDG 2.00 or later** firmware on the module. The driver checks `UBX-MON-VER` at
start-up and does not configure a receiver that reports another module or firmware.

## Hardware

### Connection

Use the **USB-C port labelled POWER+GPS**. It is the module's native USB interface
(CDC-ACM, VID:PID `1546:01ab`) and shows up as `/dev/ttyACM*`. The other USB-C port is an
FTDI bridge to the XBee socket and does not reach the module's USB interface.

### udev rule

Install the rule shipped in `udev/`:

```bash
sudo cp $(rospack find ublox_x20d_driver)/udev/99-ublox-x20d.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

It creates the symlink `/dev/ublox_x20d`, gives the `dialout` group access (add your user
with `sudo usermod -aG dialout $USER`) and tells ModemManager to leave the port alone.
Without `ID_MM_DEVICE_IGNORE`, ModemManager probes every new `/dev/ttyACM*` and the driver
cannot open it.

### Antenna placement

The module measures the vector from antenna **GPS1** (primary) to antenna **GPS2**
(secondary). On the simpleRTK4 Dual, GPS1 is the antenna connected to **RF1** and GPS2 the
one connected to **RF2**. Mount both on the vehicle's centre line:

```
          stern                         bow
   ─────── GPS1 ─────────────────────── GPS2 ───────▶  forward (x)
```

- The heading reported by the receiver is the direction of GPS1 → GPS2. With GPS1 aft and
  GPS2 forward it is the vehicle heading and the receiver offset stays at 0.
- For any other arrangement, set `receiver_heading_offset_deg` to the angle to add to the
  GPS1 → GPS2 direction to obtain the vehicle heading. For example, with GPS1 to port and
  GPS2 to starboard the baseline points 90° clockwise of the bow, so the offset is -90.
- Pitch is derived from the baseline elevation and is only the vehicle pitch when the
  baseline lies along the vehicle's x axis. Otherwise set `publish_pitch: false`.
- Roll cannot be observed: a single baseline carries no information about rotation around
  itself. It is published as unobserved (see [Frames and conventions](#frames-and-conventions)).
  Obtain roll from an IMU, e.g. by fusing both in a state estimator.

### Update rate

On HDG 2.00 firmware the dual-antenna heading is only reliable at 1 Hz, where it stays valid
with fixed ambiguities indefinitely. At 2 Hz (`rate_meas_ms: 500`) position output keeps
the rate, but `NAV-DAHEADING` arrives irregularly (about 0.6 Hz on average) and only about
half of its epochs have fixed ambiguities; at 3 Hz the heading is lost within about 10 s and
at 10 Hz the ambiguities are never fixed. Keep the default measurement period of 1000 ms;
the driver warns if `rate_meas_ms` is set below 500 ms. For higher-rate heading, fuse this
output with a gyroscope.

## Quick start

```bash
cd ~/differential_gps_ws
catkin build
source devel/setup.bash
roslaunch ublox_x20d_driver ublox_x20d.launch device:=/dev/ublox_x20d
```

This starts both nodes with `config/x20d_usb.yaml` and `config/heading_imu.yaml`. To also
record raw observations:

```bash
roslaunch ublox_x20d_driver ublox_x20d.launch device:=/dev/ublox_x20d enable_raw_observables:=true
```

A healthy start logs `receiver MON-VER: MOD=ZED-X20D FWVER=HDG 2.00 ...` followed by
`user configuration verified on device (N keys)`. For RTK, see
[RTK corrections](#rtk-corrections-ntrip). To verify a new setup end to end, follow
[docs/bringup_checklist.md](docs/bringup_checklist.md).

## Nodes

### ublox_x20d_node

Opens the serial port, configures the receiver and publishes its navigation output.

#### Topics

| Topic | Type | Content |
|---|---|---|
| `~fix` | `sensor_msgs/NavSatFix` | High-precision position from `NAV-HPPOSLLH` (ellipsoid height), status from the `NAV-PVT` of the same epoch |
| `~vel` | `geometry_msgs/TwistWithCovarianceStamped` | `NAV-PVT` velocity in ENU; only published with a position fix |
| `~nav_daheading` | `ublox_x20d_msgs/NavDAHeading` | Every parsed `NAV-DAHEADING` frame, valid or not |
| `~nav_pvt` | `ublox_x20d_msgs/NavPVT` | Every parsed `NAV-PVT` frame |
| `~nav_hpposllh` | `ublox_x20d_msgs/NavHPPosLLH` | Every parsed `NAV-HPPOSLLH` frame |
| `~nmea` | `nmea_msgs/Sentence` | With `publish_gga`: a `$GPGGA` sentence per epoch with a valid position, without CRLF |
| `~rtcm` (subscribed) | `rtcm_msgs/Message` | With `rtcm_input`: RTCM corrections, written to the receiver unchanged |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `link` (includes forwarded RTCM bytes), `config`, `fix`, `heading`, `raw_log` |

All messages are stamped with the host time at which the frame was received.

`~fix` status:

| Receiver state | `status.status` |
|---|---|
| no fix, time only, dead reckoning only, invalid position, or `gnssFixOK` not set | `STATUS_NO_FIX` |
| carrier-phase solution (RTK float or fixed) | `STATUS_GBAS_FIX` |
| differential corrections without carrier phase | `STATUS_SBAS_FIX` |
| any other 2D/3D fix | `STATUS_FIX` |

`status.service` is always `SERVICE_GPS`. The position covariance is diagonal
(`COVARIANCE_TYPE_DIAGONAL_KNOWN`): `hAcc²/2` east and north, `vAcc²` up.

#### Parameters

Receiver settings (marked **RAM**) are written to the receiver's RAM layer at every start
and read back; nothing is written to flash or battery-backed RAM.

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/ttyACM0` | Serial device; `/dev/ublox_x20d` with the udev rule installed |
| `baud_rate` | `460800` | Passed to the port; has no effect on the CDC-ACM USB interface |
| `frame_id` | `gps` | `frame_id` of every published message |
| `reopen_period` | `1.0` | Seconds between attempts to (re)open the port |
| `publish_gga` | `false` | Publish `~nmea` GGA sentences for an NTRIP caster |
| `rate_meas_ms` | `1000` | **RAM** `CFG_RATE_MEAS`, measurement period in ms (keep at 1000, see [Update rate](#update-rate)) |
| `rate_nav` | `1` | **RAM** `CFG_RATE_NAV`, measurements per navigation solution |
| `dyn_model` | `5` | **RAM** `CFG_NAVSPG_DYNMODEL`, dynamic platform model (5 = sea) |
| `receiver_heading_offset_deg` | `0.0` | **RAM** `CFG_NAVSPG_DAHEADING_OFFSET`, added by the receiver to the heading; within ±180 |
| `nmea_output` | `false` | **RAM** `CFG_USBOUTPROT_NMEA` |
| `rtcm_input` | `true` | **RAM** `CFG_USBINPROT_RTCM3X`, and subscribe to `~rtcm` to forward corrections to the receiver |
| `allow_receiver_reset` | `true` | Allow a controlled software reset when the receiver keeps ignoring the configuration (see below) |
| `enable_raw_observables` | `false` | **RAM** enable `RXM-RAWX` and `RXM-SFRBX` output and log UBX frames to disk |
| `raw_log_dir` | `~/.ros/ubx` | Directory for `.ubx` files; created if missing |
| `raw_log_prefix` | `x20d` | File name prefix |
| `raw_log_rotate_minutes` | `60` | Start a new file after this many minutes |
| `raw_log_content` | `all` | `all`: every UBX frame; `rxm_only`: only `RXM-*` frames |

`UBX-NAV-DAHEADING`, `NAV-PVT` and `NAV-HPPOSLLH` output on USB and UBX input/output on
USB are always enabled (**RAM**).

#### Configuration and verification

After being unpowered for hours, the ZED-X20D can acknowledge the start-up configuration
without applying it, so an acknowledgement alone proves nothing. The driver therefore:

1. polls `UBX-MON-VER` and refuses to configure anything other than a ZED-X20D with HDG
   2.00 or later (if there is no reply within 5 s it configures anyway, with a warning);
2. sends the configuration as `UBX-CFG-VALSET` to the RAM layer, one frame at a time
   (the firmware rejects a VALSET that arrives while it is still processing the previous one);
3. reads every key back from RAM with `UBX-CFG-VALGET` and compares it with what was sent.

If the readback disagrees it escalates, logging `config NOT applied on device (...)` with
the mismatching keys:

| Step | Action |
|---|---|
| a | Resend, up to 4 attempts, backing off 0.5, 1 and 2 s |
| b | Resend as a configuration transaction, 2 attempts |
| c | Controlled software reset (`UBX-CFG-RST`, hot start, nothing persistent is cleared), then start again from step a; up to 3 resets. Skipped with `allow_receiver_reset: false` |
| d | Retry every 30 s at ERROR level until the receiver accepts the configuration |

A key the receiver rejects on its own is dropped from the configuration and reported in the
`config` diagnostic; it never triggers a reset. Once verified, the configuration is read
back again if the NAV output stops for more than 5 s or if NMEA keeps arriving with NMEA
output disabled. A single `$GNTHS` sentence per second is tolerated: HDG 2.00 emits it
regardless of `CFG_USBOUTPROT_NMEA`.

If the device disappears, e.g. after the reset in step c, the driver reopens it every
`reopen_period` seconds and configures it again.

### heading_imu_node

Turns `nav_daheading` into an orientation. It is separate from the driver so that the
`Imu` can be regenerated from a bag of `nav_daheading` with different parameters.

| Topic | Type | |
|---|---|---|
| `nav_daheading` (subscribed) | `ublox_x20d_msgs/NavDAHeading` | remapped to `ublox_x20d/nav_daheading` by the launch files |
| `~imu` | `sensor_msgs/Imu` | orientation only |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `heading_imu`: published and dropped counts per reason |

An epoch is dropped when the receiver does not flag a valid fix or a valid heading, when
its carrier-phase solution is below `min_carr_soln` (unless `publish_degraded`), or when the
measured baseline length differs from the expected one (if configured).

| Parameter | Default | Description |
|---|---|---|
| `require_gnss_fix_ok` | `true` | Drop epochs without `gnssFixOK` |
| `require_rel_pos_heading_valid` | `true` | Drop epochs without `relPosHeadingValid` |
| `min_carr_soln` | `2` | Minimum carrier-phase solution: 0 any, 1 float, 2 fixed ambiguities |
| `publish_degraded` | `false` | Below `min_carr_soln`, publish with the pitch and yaw variances multiplied by `degraded_covariance_factor` instead of dropping |
| `degraded_covariance_factor` | `100.0` | See above |
| `expected_baseline_length_m` | `0.0` | GPS1–GPS2 distance; 0 disables the check. A mismatch usually means a moved antenna or severe multipath |
| `baseline_length_tolerance_m` | `0.05` | Allowed difference from the expected length |
| `drop_on_baseline_mismatch` | `true` | Drop mismatching epochs (a warning is logged either way) |
| `publish_pitch` | `true` | Publish the pitch derived from the baseline; `false` gives a yaw-only orientation |
| `heading_offset_deg` | `0.0` | Software offset added to the heading. Prefer `receiver_heading_offset_deg`, so that recorded `nav_daheading` already carries the vehicle heading |
| `min_heading_std_dev_rad` | `0.001` | Floor on the yaw standard deviation |
| `min_pitch_std_dev_rad` | `0.001` | Floor on the pitch standard deviation |
| `frame_id` | `""` | Output `frame_id`; empty keeps the one of the input message |

The heading is solved between the two antennas of the module and reaches fixed ambiguities
without RTK corrections. Keep `min_carr_soln` at 2 and `publish_degraded` off unless a
downstream filter can cope with gross errors:

- With float ambiguities the heading can be off by tens of degrees while the receiver
  reports a heading accuracy of a few degrees or less.
- `relPosHeadingValid` alone is not a quality gate. When the signal of one antenna is
  degraded, the receiver keeps that flag set for several epochs while the carrier-phase
  solution falls to float or none and the reported accuracy grows to tens of degrees.
- After one antenna has been obstructed, the return to fixed ambiguities can take minutes
  under a partially obstructed sky. During that time no `Imu` is published and the
  `heading_imu` diagnostic warns.

The `heading` diagnostic of the driver shows the carrier-phase state.

### ubx_file_player_node

Reads a `.ubx` file and publishes its NAV messages exactly as `ublox_x20d_node` does,
pacing them by their GPS time of week. Nothing is sent to any device.

| Parameter | Default | Description |
|---|---|---|
| `file` | — | `.ubx` file to replay |
| `frame_id` | `gps` | As in the driver |
| `speed` | `1.0` | Playback speed factor; 0 plays as fast as possible. Gaps longer than 5 s are shortened to 5 s |
| `start_delay` | `1.0` | Seconds to wait after advertising, so subscribers can connect |
| `loop` | `false` | Replay the file indefinitely |
| `publish_gga` | `false` | As in the driver |

## RTK corrections (NTRIP)

The driver owns the serial port, so corrections reach the receiver through it: anything
published on `/ublox_x20d/rtcm` (`rtcm_msgs/Message`) is written to the receiver as is.
`launch/ublox_x20d_ntrip.launch` adds the NTRIP client of the `ntrip_client` package
(`sudo apt install ros-noetic-ntrip-client ros-noetic-rtcm-msgs`) and connects it:

```bash
roslaunch ublox_x20d_driver ublox_x20d_ntrip.launch device:=/dev/ublox_x20d \
  host:=caster.example.org port:=2101 mountpoint:=MOUNT username:=USER password:=PASS
```

It enables `publish_gga`, so the caster receives the rover position once per second, as
VRS and nearest-station mountpoints require. The GGA sentence is built from `NAV-PVT`
(PDOP goes in the HDOP field, since `NAV-PVT` carries no HDOP) and is only sent with a
valid position. The client runs in the `ntrip` namespace so that its own `fix` input stays
unconnected: `ntrip_client` 1.4.1 would build a GGA from `NavSatFix` that drops leading zeros
of the decimal minutes and uses local time instead of UTC.

Pass credentials on the command line rather than storing them in files. With corrections,
the `link` diagnostic counts the forwarded RTCM bytes, `nav_pvt` shows `diff_soln` and
`carr_soln` 1 (float) then 2 (fixed), and `~fix` reports `STATUS_GBAS_FIX`.

## Frames and conventions

- **Position** (`~fix`) is the phase centre of antenna GPS1. No lever-arm correction is
  applied; publish the static transform from `gps` to your vehicle frame with `tf`.
- **Velocity** (`~vel`) is in ENU: `x = velE`, `y = velN`, `z = -velD`. The linear
  covariance uses the receiver's speed accuracy on each axis. The angular part is not
  observed and carries a variance of 1e6.
- **Heading**: the receiver reports it in NED, clockwise from true north, in [0°, 360°).
  REP-103 uses ENU, with yaw counter-clockwise from east:

  `yaw = normalize(π/2 − heading)`, in (−π, π]

  | Baseline points | heading | yaw |
  |---|---|---|
  | north | 0° | +π/2 |
  | east | 90° | 0 |
  | south | 180° | −π/2 |
  | west | 270° | ±π |

- **Pitch** follows REP-103 for a body frame with x forward, y left, z up: positive pitch is
  **nose down**. It is derived from the baseline as `pitch = asin(relPosD / relPosLength)`,
  so it is positive when GPS2 (forward) is lower than GPS1.
- **Roll** is not observable and is published as 0 with a variance of 1e6.
- The `Imu` quaternion is built from roll, pitch and yaw in the same convention as
  `tf2::Quaternion::setRPY`. `orientation_covariance` is diagonal (roll, pitch, yaw):
  - The yaw variance comes from the receiver's heading accuracy.
  - The pitch variance is `(accD / (relPosLength · cos pitch))²`, ignoring the length
    accuracy and its correlation with `accD`.
  - `angular_velocity_covariance[0]` and `linear_acceleration_covariance[0]` are −1:
    those fields carry no data.

## Raw observations and PPK

With `enable_raw_observables: true` the receiver outputs `RXM-RAWX` and `RXM-SFRBX` at the
measurement rate, and every UBX frame is written unchanged to
`<raw_log_dir>/<raw_log_prefix>_YYYYMMDD_HHMMSS.ubx` (UTC time of file creation). Files are
flushed after every frame and rotated every `raw_log_rotate_minutes`. At 1 Hz this is a few
kB/s.

The files are native UBX streams that RTKLIB converts directly:

```bash
convbin -r ubx -o x20d.obs -n x20d.nav x20d_20260918_120000.ubx
```

Use a recent RTKLIB release. Older versions may not decode every signal of the X20 series
and drop the ones they do not know.

## Replay without hardware

```bash
# a recorded .ubx file, through the driver's publishers and heading_imu_node
roslaunch ublox_x20d_driver replay_ubx.launch file:=/path/to/x20d_20260918_120000.ubx

# regenerate /heading_imu/imu from a bag with /ublox_x20d/nav_daheading
roslaunch ublox_x20d_driver replay_bag.launch bag:=/path/to/recording.bag
```

`replay_ubx.launch` also accepts `speed`, `start_delay` and `loop`; both accept
`heading_config` to try other `heading_imu_node` parameters.

## Checking a recording

`scripts/check_recording.py` compares what the driver published with the raw frames it
logged at the same time. Record the driver topics while raw logging is on
(`enable_raw_observables:=true`, `raw_log_content: all`), then:

```bash
pip install --user pyubx2
rosbag record -O run.bag /ublox_x20d/nav_pvt /ublox_x20d/nav_hpposllh \
  /ublox_x20d/nav_daheading /ublox_x20d/fix /ublox_x20d/vel /heading_imu/imu
rosrun ublox_x20d_driver check_recording.py --bag run.bag --ubx ~/.ros/ubx/x20d_*.ubx
```

It decodes the `.ubx` files with pyubx2, independently of the driver, and compares every
`NAV-PVT`, `NAV-HPPOSLLH` and `NAV-DAHEADING` frame field by field with the published
message. pyubx2 does not know `NAV-DAHEADING`, so that payload is unpacked from its
documented layout; checking the heading against the baseline vector and the length
against its norm covers that layout. It also checks `~fix`, `~vel` and the `Imu` against
the frames, then prints statistics:
- rates, gaps, payload lengths and versions;
- carrier-phase solution shares and heading accuracy;
- baseline length and pitch;
- how often each flag bit is set and how often it changes;
- heading of motion against dual-antenna heading while moving.

It exits with status 1 if any comparison fails. Use `--from-s`/`--to-s` to analyse one
segment of a longer recording. Pass `--receiver-offset-deg` if the receiver offset was not
0 and `--tape-length-m` to compare the baseline length with a measured one.

## Tests

```bash
catkin build && catkin run_tests && catkin_test_results
```

- **Unit tests** (gtest, no ROS) cover:
  - heading conversion and frame conventions;
  - UBX framing and parsing;
  - configuration message encoding and the configuration state machine;
  - the serial transport, on a pseudo-terminal;
  - the heading filter, the raw logger and the GGA sentence.
- **Replay tests** (rostest) replay `test/data/x20d_sample.ubx` and
  `test/data/nav_daheading_sample.bag` and check the published messages. Regenerate that
  data with `rosrun ublox_x20d_driver make_test_ubx.py`; its docstring describes each epoch.
- **`check_recording.py` tests** (nose) run it on the sample data, including cases it must
  reject. They are skipped when pyubx2 is not installed.

## License and acknowledgements

Apache License 2.0, see `LICENSE`.

The configuration engine, the heading conversion and the UBX message layouts are adapted
from [MonKey-Robotics/ublox_zedx20d](https://github.com/MonKey-Robotics/ublox_zedx20d), a
ROS 2 driver developed and verified on the same hardware, itself a fork of
[aussierobots/ublox_dgnss](https://github.com/aussierobots/ublox_dgnss). Files derived from
it keep their original copyright notice and state what was changed.
