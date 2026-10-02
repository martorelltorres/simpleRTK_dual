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
"""Tests for scripts/base_download.py, the station catalog of scripts/ppk_process.py and the
ROS-independent parts of scripts/ppk_pipeline.py. No network: downloads go to a local HTTP
server and crx2rnx is replaced by a stub."""

import datetime
import functools
import gzip
import http.server
import importlib.util
import os
import shutil
import stat
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'scripts')
CONFIG = os.path.join(HERE, '..', 'config')


def load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base_download = load('base_download')
ppk_process = load('ppk_process')
ppk_pipeline = load('ppk_pipeline')

UTC = datetime.datetime


def mal1():
    return ppk_process.load_station('MAL1', os.path.join(CONFIG, 'base_stations.yaml'))


class HoursTest(unittest.TestCase):
    def test_span_inside_one_hour(self):
        hours = base_download.hours_for_span(UTC(2026, 10, 2, 7, 12, 50), UTC(2026, 10, 2, 7, 28, 0))
        self.assertEqual(hours, [UTC(2026, 10, 2, 7)])

    def test_span_across_hours(self):
        hours = base_download.hours_for_span(UTC(2026, 10, 2, 7, 30), UTC(2026, 10, 2, 9, 15))
        self.assertEqual(hours, [UTC(2026, 10, 2, 7), UTC(2026, 10, 2, 8), UTC(2026, 10, 2, 9)])

    def test_margin_reaches_previous_and_next_hour(self):
        hours = base_download.hours_for_span(UTC(2026, 10, 2, 7, 0, 30), UTC(2026, 10, 2, 7, 59, 30), 60)
        self.assertEqual(hours, [UTC(2026, 10, 2, 6), UTC(2026, 10, 2, 7), UTC(2026, 10, 2, 8)])
        hours = base_download.hours_for_span(UTC(2026, 10, 2, 7, 0, 30), UTC(2026, 10, 2, 7, 59, 30), 0)
        self.assertEqual(hours, [UTC(2026, 10, 2, 7)])

    def test_new_year(self):
        hours = base_download.hours_for_span(UTC(2026, 12, 31, 23, 50), UTC(2027, 1, 1, 0, 10))
        self.assertEqual(hours, [UTC(2026, 12, 31, 23), UTC(2027, 1, 1, 0)])
        station = mal1()
        self.assertTrue(base_download.file_urls(station, hours[0])[0].endswith('MAL100ESP_R_20263652300_01H_01S_MO.crx.gz'))
        self.assertTrue(base_download.file_urls(station, hours[1])[0].endswith('MAL100ESP_R_20270010000_01H_01S_MO.crx.gz'))

    def test_gps_to_utc(self):
        # 2026-10-02 07:12:50 UTC = 07:13:08 GPS time.
        gps = ppk_pipeline.ppk_fuse.gpst_seconds(UTC(2026, 10, 2, 7, 13, 8))
        self.assertEqual(ppk_pipeline.gps_to_utc(gps), UTC(2026, 10, 2, 7, 12, 50))


class StationTest(unittest.TestCase):
    def test_ign_file_names(self):
        obs, nav = base_download.file_urls(mal1(), UTC(2026, 10, 2, 7))
        self.assertEqual(obs, 'https://datos-geodesia.ign.es/ERGNSS/horario_1s/20261002/07/'
                              'MAL100ESP_R_20262750700_01H_01S_MO.crx.gz')
        self.assertEqual(nav, 'https://datos-geodesia.ign.es/ERGNSS/horario_1s/20261002/07/'
                              'MAL100ESP_R_20262750700_01H_MN.rnx.gz')

    def test_mal1_options_match_station_sheet(self):
        # Sheet: 39 33' 36.63673" N, 2 38' 14.83584" E, 53.787 m, antenna 0.0350 m above the mark.
        options = dict(line.split('=') for line in ppk_process.station_options(mal1()).splitlines())
        options = {k.strip(): float(v) for k, v in options.items()}
        self.assertAlmostEqual(options['ant2-pos1'], 39 + 33 / 60 + 36.63673 / 3600, places=9)
        self.assertAlmostEqual(options['ant2-pos2'], 2 + 38 / 60 + 14.83584 / 3600, places=9)
        self.assertEqual(options['ant2-pos3'], 53.787)
        self.assertEqual(options['ant2-antdelu'], 0.035)

    def test_options_file_leaves_station_to_catalog(self):
        with open(os.path.join(CONFIG, 'ppk_rtklib.conf')) as f:
            keys = [line.split('=')[0].strip() for line in f if '=' in line and not line.startswith('#')]
        for key in ('ant2-pos1', 'ant2-pos2', 'ant2-pos3', 'ant2-antdelu'):
            self.assertNotIn(key, keys)
        self.assertIn('ant2-postype', keys)

    def test_unknown_station(self):
        with self.assertRaises(KeyError):
            ppk_process.load_station('XXXX', os.path.join(CONFIG, 'base_stations.yaml'))


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class DownloadTest(unittest.TestCase):
    """fetch_span against a local HTTP server that publishes one hour of a fake station."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.www = os.path.join(self.tmp, 'www')
        hour_dir = os.path.join(self.www, '20261002', '07')
        os.makedirs(hour_dir)
        for name, content in (('TEST00ESP_R_20262750700_01H_01S_MO.crx.gz', b'crx observations\n'),
                              ('TEST00ESP_R_20262750700_01H_MN.rnx.gz', b'navigation\n')):
            with gzip.open(os.path.join(hour_dir, name), 'wb') as f:
                f.write(content)
        handler = functools.partial(QuietHandler, directory=self.www)
        self.server = http.server.HTTPServer(('127.0.0.1', 0), handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        # crx2rnx stub: <name>.crx -> <name>.rnx
        self.crx2rnx = os.path.join(self.tmp, 'crx2rnx')
        with open(self.crx2rnx, 'w') as f:
            f.write('#!/bin/sh\nfor a; do case "$a" in *.crx) cp "$a" "${a%.crx}.rnx";; esac; done\n')
        os.chmod(self.crx2rnx, os.stat(self.crx2rnx).st_mode | stat.S_IEXEC)
        self.station = {
            'name': 'TEST', 'id': 'TEST00ESP',
            'network': {
                'url': 'http://127.0.0.1:%d/{date:%%Y%%m%%d}/{date:%%H}/' % self.server.server_port,
                'obs': '{id}_R_{date:%Y%j%H}00_01H_01S_MO.crx.gz',
                'nav': '{id}_R_{date:%Y%j%H}00_01H_MN.rnx.gz',
            },
        }
        self.cache = os.path.join(self.tmp, 'cache')

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        shutil.rmtree(self.tmp)

    def test_download_decompress_and_cache(self):
        obs, nav, hours = base_download.fetch_span(self.station, UTC(2026, 10, 2, 7, 12), UTC(2026, 10, 2, 7, 28),
                                                   self.cache, self.crx2rnx)
        self.assertEqual(hours, [UTC(2026, 10, 2, 7)])
        self.assertTrue(obs[0].endswith('TEST00ESP_R_20262750700_01H_01S_MO.rnx'))
        self.assertTrue(nav[0].endswith('TEST00ESP_R_20262750700_01H_MN.rnx'))
        with open(obs[0], 'rb') as f:
            self.assertEqual(f.read(), b'crx observations\n')
        with open(nav[0], 'rb') as f:
            self.assertEqual(f.read(), b'navigation\n')
        self.assertEqual(sorted(os.listdir(os.path.dirname(obs[0]))), sorted(os.path.basename(p) for p in obs + nav))
        # Second run: served from the cache, even with the server gone.
        shutil.rmtree(self.www)
        self.assertEqual(base_download.fetch_span(self.station, UTC(2026, 10, 2, 7, 12), UTC(2026, 10, 2, 7, 28),
                                                  self.cache, self.crx2rnx)[:2], (obs, nav))

    def test_unpublished_hour(self):
        with self.assertRaises(base_download.NotPublishedError) as ctx:
            base_download.fetch_span(self.station, UTC(2026, 10, 2, 7, 50), UTC(2026, 10, 2, 8, 10),
                                     self.cache, self.crx2rnx)
        self.assertIn('hour 2026-10-02 08:00 UTC not published', str(ctx.exception))


class PipelineConfigTest(unittest.TestCase):
    def test_defaults_point_at_package_files(self):
        config = ppk_pipeline.load_config(os.path.join(CONFIG, 'ppk_pipeline.yaml'))
        self.assertEqual(config['station'], 'MAL1')
        self.assertEqual(config['antex'], '')
        for key, name in (('stations_file', 'base_stations.yaml'), ('rtklib_conf', 'ppk_rtklib.conf'),
                          ('fuse_config', 'ppk_fuse.yaml')):
            self.assertTrue(os.path.exists(config[key]), key)
            self.assertEqual(os.path.basename(config[key]), name)
        self.assertEqual(config['cache_dir'], os.path.expanduser('~/.ros/ppk_base'))


if __name__ == '__main__':
    sys.exit(unittest.main())
