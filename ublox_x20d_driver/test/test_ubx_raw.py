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
"""Tests for scripts/ubx_raw.py and scripts/bag_to_ubx.py on frames recorded from a ZED-X20D."""

import importlib.util
import os
import shutil
import struct
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'scripts')


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ubx_raw = load('ubx_raw')
bag_to_ubx = load('bag_to_ubx')

# RXM-RAWX header and first two measurements of a recorded frame, numMeas set to 2
# (the same payload as in test_ubx_parsers.cpp).
RAWX = bytes.fromhex(
    'dd2406015cd516418609120241021d6d'
    '4634c81cea477441abc349f68be59341'
    '2f7ab14400060700f4fb2c0402070700'
    '32aef0886a5273415190b221b3f49241'
    '1d5817440009070024361c0507080700')
# RXM-SFRBX, GPS SV 9, 10 words.
SFRBX = bytes.fromhex(
    '000900000a9b02200986c122d3896d9e3d0064182ed9189d18a334abaf6c550f'
    '4600f33818dad693b3ea3f004721c815')


def rawx_message(payload):
    """A stand-in for ublox_x20d_msgs/RxmRawx decoded here with struct, independently of the driver."""
    rcv_tow, week, leap_s, num_meas, rec_stat, version, reserved0 = struct.unpack_from('<dHbBBBH', payload)
    meas = []
    for i in range(num_meas):
        f = struct.unpack_from('<ddfBBBBHBBBBB', payload, 16 + 32 * i)
        meas.append(types.SimpleNamespace(
            pr_mes=f[0], cp_mes=f[1], do_mes=f[2], gnss_id=f[3], sv_id=f[4], sig_id=f[5], freq_id=f[6],
            locktime=f[7], cno=f[8], pr_stdev=f[9] & 0x0F, cp_stdev=f[10] & 0x0F, do_stdev=f[11] & 0x0F,
            trk_stat=f[12]))
    return types.SimpleNamespace(rcv_tow=rcv_tow, week=week, leap_s=leap_s, rec_stat=rec_stat, version=version,
                                 reserved0=reserved0, meas=meas)


def sfrbx_message(payload):
    gnss_id, sv_id, sig_id, freq_id, num_words, chn, version, reserved0 = struct.unpack_from('<8B', payload)
    dwrd = list(struct.unpack_from('<%dI' % num_words, payload, 8))
    return types.SimpleNamespace(gnss_id=gnss_id, sv_id=sv_id, sig_id=sig_id, freq_id=freq_id, chn=chn,
                                 version=version, reserved0=reserved0, dwrd=dwrd)


class UbxRawTest(unittest.TestCase):
    def test_rawx_payload_rebuilds_the_recorded_bytes(self):
        self.assertEqual(ubx_raw.rawx_payload(rawx_message(RAWX)), RAWX)

    def test_sfrbx_payload_rebuilds_the_recorded_bytes(self):
        self.assertEqual(ubx_raw.sfrbx_payload(sfrbx_message(SFRBX)), SFRBX)

    def test_frame_header_and_checksum(self):
        f = ubx_raw.sfrbx_frame(sfrbx_message(SFRBX))
        self.assertEqual(f[:6], b'\xb5\x62\x02\x13' + struct.pack('<H', len(SFRBX)))
        self.assertEqual(f[6:-2], SFRBX)
        ck_a = ck_b = 0
        for byte in f[2:-2]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        self.assertEqual(f[-2:], bytes((ck_a, ck_b)))


class BagToUbxTest(unittest.TestCase):
    def test_bag_round_trip(self):
        import rosbag
        from ublox_x20d_msgs.msg import RawxMeas, RxmRawx, RxmSfrbx

        r = rawx_message(RAWX)
        rawx = RxmRawx(rcv_tow=r.rcv_tow, week=r.week, leap_s=r.leap_s, rec_stat=r.rec_stat, version=r.version,
                       reserved0=r.reserved0, meas=[RawxMeas(**vars(m)) for m in r.meas])
        sfrbx = RxmSfrbx(**vars(sfrbx_message(SFRBX)))
        tmp = tempfile.mkdtemp()
        try:
            bag_path = os.path.join(tmp, 'raw.bag')
            with rosbag.Bag(bag_path, 'w') as bag:
                bag.write('/ublox_x20d/rxm_rawx', rawx)
                bag.write('/ublox_x20d/rxm_sfrbx', sfrbx)
            out = os.path.join(tmp, 'raw.ubx')
            self.assertEqual(bag_to_ubx.write_ubx(bag_path, out), (1, 1))
            with open(out, 'rb') as f:
                data = f.read()
            self.assertEqual(data, ubx_raw.frame(0x02, 0x15, RAWX) + ubx_raw.frame(0x02, 0x13, SFRBX))
        finally:
            shutil.rmtree(tmp)


if __name__ == '__main__':
    unittest.main()
