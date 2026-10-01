#!/usr/bin/env python3
# Copyright 2026 Antoni Martorell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generate the deterministic replay test data in test/data/.

x20d_sample.ubx: ten 1 Hz epochs of NAV-PVT, NAV-DAHEADING and NAV-HPPOSLLH with an
NMEA sentence before each epoch. nav_daheading_sample.bag: the nav_daheading messages
the driver publishes for that file, for replaying against heading_imu_node alone.

Epochs (heading clockwise from north, baseline GPS1 -> GPS2 of about 1 m):
  0-4  valid, fixed ambiguities, 90 deg, level
  5    valid, float ambiguities            (dropped by heading_imu with min_carr_soln 2)
  6    relPosHeadingValid not set          (dropped by heading_imu)
  7    NAV-DAHEADING payload version 0x01  (dropped by the driver)
  8    valid, fixed, 45 deg, forward antenna 100 mm lower (positive pitch)
  9    valid, fixed, 270 deg, level

Requires a sourced workspace (for ublox_x20d_msgs) to write the bag:
  rosrun ublox_x20d_driver make_test_ubx.py
"""

import math
import os
import struct

ITOW0 = 345600000
STAMP0 = 1789732800  # 2026-09-18 12:00:00 UTC

FLAGS_GOOD = 0x01 | 0x04 | (2 << 3) | 0x40  # gnssFixOK, relPosValid, fixed, headingValid
FLAGS_FLOAT = 0x01 | 0x04 | (1 << 3) | 0x40
FLAGS_NO_HEADING = 0x01 | 0x04 | (2 << 3)

# (relPosN, relPosE, relPosD, heading deg, flags, payload version)
EPOCHS = [(0, 1000, 0, 90.0, FLAGS_GOOD, 2)] * 5 + [
    (0, 1000, 0, 90.0, FLAGS_FLOAT, 2),
    (0, 1000, 0, 90.0, FLAGS_NO_HEADING, 2),
    (0, 1000, 0, 90.0, FLAGS_GOOD, 1),
    (700, 700, 100, 45.0, FLAGS_GOOD, 2),
    (0, -1000, 0, 270.0, FLAGS_GOOD, 2),
]


def frame(msg_class, msg_id, payload):
    body = bytes([msg_class, msg_id]) + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes([ck_a, ck_b])


def nav_pvt(itow):
    p = bytearray(92)
    struct.pack_into('<IHBBBBBB', p, 0, itow, 2026, 9, 18, 12, 0, 0, 0x07)
    p[20] = 3                          # 3D fix
    p[21] = 0x01 | 0x02 | (2 << 6)     # gnssFixOK, diffSoln, carrSoln fixed
    p[23] = 30
    struct.pack_into('<iiiiII', p, 24, 26461234, 395712345, 51234, 1234, 14, 21)
    struct.pack_into('<iii', p, 48, 1000, 0, 0)  # 1 m/s north
    struct.pack_into('<I', p, 68, 50)
    return frame(0x01, 0x07, bytes(p))


def nav_hpposllh(itow):
    p = bytearray(36)
    struct.pack_into('<BxxBIiiiibbbbII', p, 0, 0, 0, itow, 26461234, 395712345, 51234, 1234,
                     -45, 67, -9, 8, 140, 210)
    return frame(0x01, 0x14, bytes(p))


def length_mm(n, e, d):
    return int(round(math.sqrt(n * n + e * e + d * d)))


def nav_daheading(itow, n, e, d, heading_deg, flags, version):
    if version == 1:
        p = bytearray(64)  # pre-production layout, only the version byte matters here
        p[0] = 1
        return frame(0x01, 0x45, bytes(p))
    p = bytearray(60)
    p[0] = 2
    struct.pack_into('<Iiiiii', p, 4, itow, n, e, d, length_mm(n, e, d), int(round(heading_deg * 1e5)))
    struct.pack_into('<IIIII', p, 32, 5, 5, 8, 5, 61000)
    struct.pack_into('<I', p, 56, flags)
    return frame(0x01, 0x45, bytes(p))


def write_ubx(path):
    with open(path, 'wb') as out:
        for i, (n, e, d, heading, flags, version) in enumerate(EPOCHS):
            itow = ITOW0 + 1000 * i
            out.write(b'$GNTHS,%.2f,A*00\r\n' % heading)
            out.write(nav_pvt(itow))
            out.write(nav_daheading(itow, n, e, d, heading, flags, version))
            out.write(nav_hpposllh(itow))


def write_bag(path):
    import rosbag
    import rospy
    from ublox_x20d_msgs.msg import NavDAHeading

    with rosbag.Bag(path, 'w') as bag:
        for i, (n, e, d, heading, flags, version) in enumerate(EPOCHS):
            if version != 2:
                continue
            m = NavDAHeading()
            m.header.stamp = rospy.Time(STAMP0 + i)
            m.header.frame_id = 'gps'
            m.version = 2
            m.itow = ITOW0 + 1000 * i
            m.rel_pos_n, m.rel_pos_e, m.rel_pos_d = n, e, d
            m.rel_pos_length = length_mm(n, e, d)
            m.rel_pos_heading = int(round(heading * 1e5))
            m.acc_n, m.acc_e, m.acc_d, m.acc_length, m.acc_heading = 5, 5, 8, 5, 61000
            m.flags = flags
            m.gnss_fix_ok = bool(flags & 0x01)
            m.diff_soln = bool(flags & 0x02)
            m.rel_pos_valid = bool(flags & 0x04)
            m.carr_soln = (flags >> 3) & 0x03
            m.rel_pos_heading_valid = bool(flags & 0x40)
            bag.write('/ublox_x20d/nav_daheading', m, m.header.stamp)


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'test', 'data')
    write_ubx(os.path.join(data_dir, 'x20d_sample.ubx'))
    write_bag(os.path.join(data_dir, 'nav_daheading_sample.bag'))


if __name__ == '__main__':
    main()
