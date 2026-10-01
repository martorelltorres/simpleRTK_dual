# simpleRTK_dual

ROS Noetic driver for the ArduSimple **simpleRTK4 Dual** board (u-blox **ZED-X20D**,
dual-antenna GNSS, firmware HDG 2.00 or later).

| Package | Contents |
|---|---|
| [`ublox_x20d_driver`](ublox_x20d_driver/README.md) | Receiver driver (RTK `NavSatFix`, RTCM forwarding, raw `.ubx` logging for PPK), dual-antenna heading as `sensor_msgs/Imu`, `.ubx` replay |
| [`ublox_x20d_msgs`](ublox_x20d_msgs) | Message definitions for `UBX-NAV-DAHEADING`, `UBX-NAV-PVT` and `UBX-NAV-HPPOSLLH` |

## Build

Clone into the `src/` folder of a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/martorelltorres/simpleRTK_dual.git
cd ..
rosdep install --from-paths src --ignore-src -y
catkin build
catkin run_tests && catkin_test_results
```

Wiring, antenna mounting, parameters, topics and frame conventions are documented in the
[driver README](ublox_x20d_driver/README.md).

## License

Apache-2.0. See the `LICENSE` file in each package.
