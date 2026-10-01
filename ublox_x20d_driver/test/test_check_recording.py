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
        self.assertEqual(report.failures['heading vs baseline vector'], 9)

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


if __name__ == '__main__':
    unittest.main()
