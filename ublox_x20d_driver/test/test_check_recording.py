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
"""Runs scripts/check_recording.py on the sample data from scripts/make_test_ubx.py."""

import importlib.util
import os
import shutil
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'data')
UBX = os.path.join(DATA, 'x20d_sample.ubx')
BAG = os.path.join(DATA, 'nav_daheading_sample.bag')

spec = importlib.util.spec_from_file_location('check_recording',
                                              os.path.join(HERE, '..', 'scripts', 'check_recording.py'))
check_recording = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_recording)

HAVE_PYUBX2 = importlib.util.find_spec('pyubx2') is not None


@unittest.skipUnless(HAVE_PYUBX2, 'pyubx2 not installed')
class CheckRecordingTest(unittest.TestCase):
    def run_check(self, *extra, ubx=UBX):
        args = check_recording.parse_args(['--bag', BAG, '--ubx', ubx] + list(extra))
        return check_recording.run(args)

    def test_sample_passes(self):
        report, status = self.run_check()
        text = report.render()
        self.assertEqual(status, 0, text)
        self.assertIn('daheading    9 published, 9 matched to frames', text)
        self.assertIn('NAV-DAHEADING versions: {2: 9, 1: 1}', text)
        self.assertIn('NAV-PVT payload lengths: {92: 10}', text)

    def test_wrong_receiver_offset_fails(self):
        report, status = self.run_check('--receiver-offset-deg', '5')
        self.assertEqual(status, 1)
        # Nine epochs; the one without relPosHeadingValid has no heading to compare.
        self.assertEqual(report.failures['heading vs baseline vector'], 8)

    def test_published_message_without_frame_fails(self):
        # Keep only the first half of the raw stream: later published messages have no frame.
        tmp = tempfile.mkdtemp()
        try:
            with open(UBX, 'rb') as f:
                data = f.read()
            truncated = os.path.join(tmp, 'truncated.ubx')
            with open(truncated, 'wb') as f:
                f.write(data[:len(data) // 2])
            report, status = self.run_check(ubx=truncated)
            self.assertEqual(status, 1)
            self.assertGreater(report.failures['published without a frame'], 0)
        finally:
            shutil.rmtree(tmp)


def daheading(n, e, length, heading_deg, heading_valid=True):
    return types.SimpleNamespace(rel_pos_n=n, rel_pos_e=e, rel_pos_length=length,
                                 rel_pos_heading=int(round(heading_deg * 1e5)),
                                 rel_pos_valid=True, rel_pos_heading_valid=heading_valid)


class HeadingVectorMismatchTest(unittest.TestCase):
    def test_invalid_heading_is_not_compared(self):
        # The receiver zeroes relPosHeading when relPosHeadingValid is clear.
        self.assertIsNone(check_recording.heading_vector_mismatch(daheading(76, -650, 6053, 0.0, False), 0.0))

    def test_tolerance_uses_the_horizontal_baseline(self):
        # Float epoch: 0.82 m horizontal baseline, 6.45 m reported length (vertical error).
        diff, tol, vector = check_recording.heading_vector_mismatch(daheading(310, -765, 6454, 292.0936), 0.0)
        self.assertAlmostEqual(vector, -67.941, places=3)
        self.assertLess(diff, tol)

    def test_wrong_heading_is_detected(self):
        diff, tol, _ = check_recording.heading_vector_mismatch(daheading(310, -765, 6454, 292.6), 0.0)
        self.assertGreater(diff, tol)

    def test_receiver_offset_is_removed(self):
        diff, tol, _ = check_recording.heading_vector_mismatch(daheading(-1074, -219, 5158, 191.5606 + 90.0), 90.0)
        self.assertLess(diff, tol)


if __name__ == '__main__':
    unittest.main()
