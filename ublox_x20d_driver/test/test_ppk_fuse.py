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
"""Tests for the ROS-independent parts of scripts/ppk_fuse.py and scripts/ppk_process.py."""

import datetime
import importlib.util
import os
import tempfile
import types
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, '..', 'scripts', name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ppk_fuse = load('ppk_fuse')
ppk_process = load('ppk_process')

POS = """% program   : RTKLIB
%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2026/10/01 07:55:03.001   39.637079319    2.644648885   156.7206   1  14   0.0040   0.0030   0.0090   0.0000   0.0000   0.0000   0.00   12.3
2026/10/01 07:55:04.001   39.637083143    2.644640290   155.2985   2  12   0.2132   0.1950   0.4100   0.0000   0.0000   0.0000   0.00    1.1
"""


class TimeTest(unittest.TestCase):
    def test_gps_week_and_time_of_week(self):
        # RXM-RAWX recorded on 2026-10-01: week 2438, rcvTow 374103.001 s (GPS time).
        week, tow = ppk_fuse.week_tow(ppk_fuse.gpst_seconds(datetime.datetime(2026, 10, 1, 7, 55, 3, 1000)))
        self.assertEqual(week, 2438)
        self.assertAlmostEqual(tow, 374103.001, places=6)

    def test_pvt_week_from_utc_date(self):
        # UTC 07:54:45 is GPS 07:55:03 (18 s leap); iTOW 374103001 ms.
        pvt = types.SimpleNamespace(year=2026, month=10, day=1, hour=7, min=54, sec=45, nano=1000000,
                                    itow=374103001)
        self.assertAlmostEqual(ppk_fuse.pvt_gps_seconds(pvt), 2438 * 604800 + 374103.001, places=6)

    def test_pvt_week_at_rollover(self):
        # Saturday 23:59:50 UTC is already in the next GPS week (tow 8 s).
        pvt = types.SimpleNamespace(year=2026, month=10, day=3, hour=23, min=59, sec=50, nano=0, itow=8000)
        week, tow = ppk_fuse.week_tow(ppk_fuse.pvt_gps_seconds(pvt))
        self.assertEqual((week, tow), (2439, 8.0))


class PosTest(unittest.TestCase):
    def test_read_pos(self):
        with tempfile.NamedTemporaryFile('w', suffix='.pos', delete=False) as f:
            f.write(POS)
        try:
            epochs = ppk_fuse.read_pos(f.name)
            self.assertEqual(ppk_process.quality_counts(f.name), {1: 1, 2: 1})
        finally:
            os.unlink(f.name)
        self.assertEqual(len(epochs), 2)
        week, tow = ppk_fuse.week_tow(epochs[0]['t'])
        self.assertEqual(week, 2438)
        self.assertAlmostEqual(tow, 374103.001, places=6)
        self.assertEqual(epochs[1]['q'], 2)
        self.assertAlmostEqual(epochs[1]['sdu'], 0.41)

    def test_old_rtklib_detection(self):
        self.assertTrue(ppk_process.is_old_rtklib('CONVBIN 2.4.3'))
        self.assertFalse(ppk_process.is_old_rtklib('CONVBIN demo5 b34'))


class GeodesyTest(unittest.TestCase):
    def test_local_frame_round_trip(self):
        frame = ppk_fuse.LocalFrame(39.6372, 2.6446, 99.2)
        enu = frame.to_enu(39.6373, 2.6447, 101.0)
        self.assertAlmostEqual(enu[0], 0.0001 * 111319.5 * np.cos(np.radians(39.6372)), delta=0.2)
        self.assertAlmostEqual(enu[1], 11.1, delta=0.2)
        lat, lon, h = frame.to_geodetic(enu)
        self.assertAlmostEqual(lat, 39.6373, places=9)
        self.assertAlmostEqual(lon, 2.6447, places=9)
        self.assertAlmostEqual(h, 101.0, places=4)


class SmootherTest(unittest.TestCase):
    """Synthetic run: 1 Hz, 200 s, 1 m/s east then a turn north."""

    def setUp(self):
        rng = np.random.default_rng(1)
        self.times = np.arange(200.0)
        vel = np.where(self.times[:, None] < 100, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        self.truth = np.cumsum(vel, axis=0)
        self.measurements = []
        for k, t in enumerate(self.times):
            ms = []
            ppk_gap = 120 <= t < 140
            if not ppk_gap:
                ms.append(ppk_fuse.Measurement('pos', self.truth[k] + rng.normal(0, 0.02, 3),
                                               np.eye(3) * 0.02 ** 2, 'ppk'))
            # live solution: 2 m bias east, 1.5 m noise
            ms.append(ppk_fuse.Measurement('live', self.truth[k] + [2.0, 0, 0] + rng.normal(0, 1.5, 3),
                                           np.eye(3) * 1.5 ** 2, 'live'))
            ms.append(ppk_fuse.Measurement('vel', vel[k] + rng.normal(0, 0.05, 3), np.eye(3) * 0.05 ** 2,
                                           'doppler'))
            self.measurements.append(ms)
        self.outlier = ppk_fuse.Measurement('pos', self.truth[60] + [5.0, 0, 0], np.eye(3) * 0.02 ** 2, 'ppk')
        self.measurements[60] = [self.outlier] + self.measurements[60][1:]

    def run_smoother(self):
        return ppk_fuse.smooth(self.times, self.measurements, 0.5, 0.1, 16.27)

    def test_improves_on_ppk_where_available(self):
        states, covs = self.run_smoother()
        mask = (self.times < 120) | (self.times >= 140)
        mask[[60, 99, 100, 101]] = False  # outlier and the turn
        ppk = np.array([[m.value for m in ms if m.source == 'ppk'][0] for ms, keep in
                        zip(self.measurements, mask) if keep])
        raw = np.linalg.norm(ppk - self.truth[mask], axis=1)
        err = np.linalg.norm(states[mask, 0:3] - self.truth[mask], axis=1)
        self.assertLess(np.sqrt(np.mean(err ** 2)), np.sqrt(np.mean(raw ** 2)))

    def test_bridges_a_ppk_gap(self):
        states, covs = self.run_smoother()
        gap = (self.times >= 120) & (self.times < 140)
        err = np.linalg.norm(states[gap, 0:3] - self.truth[gap], axis=1)
        self.assertLess(err.max(), 0.5)
        # the uncertainty grows inside the gap
        self.assertGreater(np.trace(covs[130][0:3, 0:3]), np.trace(covs[110][0:3, 0:3]))

    def test_rejects_an_outlier(self):
        states, _ = self.run_smoother()
        self.assertFalse(self.outlier.accepted)
        self.assertLess(np.linalg.norm(states[60, 0:3] - self.truth[60]), 0.1)

    def test_live_bias_is_estimated_and_kept_out(self):
        states, _ = self.run_smoother()
        self.assertLess(abs(np.mean(states[0:100, 0] - self.truth[0:100, 0])), 0.01)
        self.assertAlmostEqual(np.mean(states[:, 6]), 2.0, delta=0.5)


if __name__ == '__main__':
    unittest.main()
