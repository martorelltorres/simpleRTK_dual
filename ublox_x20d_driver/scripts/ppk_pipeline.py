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
"""Post-process a mission with PPK in one command: from the mission bag to the fused solution.

  ppk_pipeline.py --bag mission.bag --out-dir ppk/ [--ubx ~/.ros/ubx/x20d_*.ubx]

Steps:
  1. time span of the mission, from the NAV-PVT messages of the bag (GPS time);
  2. rover raw observations: the given .ubx files, or rebuilt from the rxm_rawx and
     rxm_sfrbx topics of the bag (bag_to_ubx.py);
  3. reference station RINEX files of every hour of the span, downloaded and cached
     (base_download.py), or given with --base-obs/--base-nav;
  4. PPK with RTKLIB (ppk_process.py) -> <out-dir>/<mission>.pos;
  5. fusion with the live solution (ppk_fuse.py) -> <out-dir>/<mission>_ppk.bag and .csv.

Settings: config/ppk_pipeline.yaml (station, ANTEX file, tool directories, cache).
"""

import argparse
import datetime
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bag_to_ubx  # noqa: E402
import base_download  # noqa: E402
import ppk_fuse  # noqa: E402
import ppk_process  # noqa: E402

package_config = ppk_process.package_config


def gps_to_utc(gps_seconds):
    """UTC datetime of a time in GPS seconds (since the GPS epoch)."""
    return ppk_fuse.GPS_EPOCH + datetime.timedelta(seconds=gps_seconds - ppk_fuse.GPS_UTC_LEAP_S)


def mission_span(bag_path, ns):
    """(first, last) GPS time in seconds of the NAV-PVT epochs of the bag, or None."""
    epochs, _ = ppk_fuse.read_driver_epochs(bag_path, ns)
    if not epochs:
        return None
    times = [e['t'] for e in epochs.values()]
    return min(times), max(times)


def load_config(path):
    config = ppk_fuse.load_config(path) or {}
    defaults = {
        'station': 'MAL1', 'stations_file': '', 'antex': '', 'rtklib_bin': '', 'crx2rnx_bin': '',
        'cache_dir': '~/.ros/ppk_base', 'margin_s': 60.0, 'rtklib_conf': '', 'fuse_config': '',
    }
    for key, value in defaults.items():
        if config.get(key) in (None, ''):
            config[key] = value
    if not config['stations_file']:
        config['stations_file'] = package_config('base_stations.yaml')
    if not config['rtklib_conf']:
        config['rtklib_conf'] = package_config('ppk_rtklib.conf')
    if not config['fuse_config']:
        config['fuse_config'] = package_config('ppk_fuse.yaml')
    for key in ('antex', 'rtklib_bin', 'crx2rnx_bin', 'cache_dir', 'stations_file', 'rtklib_conf',
                'fuse_config'):
        config[key] = os.path.expanduser(config[key])
    return config


def run(args, config):
    os.makedirs(args.out_dir, exist_ok=True)
    name = os.path.splitext(os.path.basename(args.bag))[0]

    # 1. Time span of the mission.
    span = mission_span(args.bag, args.driver_ns)
    if span is None:
        print('error: no %s/nav_pvt with a valid date in %s' % (args.driver_ns, args.bag), file=sys.stderr)
        return 2
    start, end = gps_to_utc(span[0]), gps_to_utc(span[1])
    print('[1/5] mission %s: %s to %s UTC (%.0f s)' % (name, start.strftime('%Y-%m-%d %H:%M:%S'),
                                                      end.strftime('%H:%M:%S'), span[1] - span[0]))

    # 2. Rover raw observations.
    if args.ubx:
        rover = args.ubx
        print('[2/5] rover: %d .ubx file(s)' % len(rover))
    else:
        rover = [os.path.join(args.out_dir, name + '_rover.ubx')]
        rawx, sfrbx = bag_to_ubx.write_ubx(args.bag, rover[0], args.driver_ns)
        if rawx == 0:
            print('error: no %s/rxm_rawx in %s and no --ubx given: record with enable_raw_observables: true'
                  % (args.driver_ns, args.bag), file=sys.stderr)
            return 2
        print('[2/5] rover: rebuilt from the bag, %d RXM-RAWX and %d RXM-SFRBX frames -> %s'
              % (rawx, sfrbx, rover[0]))

    # 3. Reference station.
    station = ppk_process.load_station(config['station'], config['stations_file'])
    if args.base_obs:
        base_obs, base_nav = args.base_obs, args.base_nav
        print('[3/5] reference station %s: %d local observation file(s)' % (station['name'], len(base_obs)))
    else:
        crx2rnx = os.path.join(config['crx2rnx_bin'], 'crx2rnx') if config['crx2rnx_bin'] else 'crx2rnx'
        base_obs, base_nav, hours = base_download.fetch_span(station, start, end, config['cache_dir'],
                                                             crx2rnx, float(config['margin_s']))
        print('[3/5] reference station %s: hours %s UTC (cache %s)'
              % (station['name'], ', '.join(h.strftime('%Y-%m-%d %H') for h in hours), config['cache_dir']))

    # 4. PPK.
    pos = os.path.join(args.out_dir, name + '.pos')
    print('[4/5] PPK')
    counts = ppk_process.process(rover, base_obs, base_nav, config['rtklib_conf'], pos, station,
                                 config['antex'], config['rtklib_bin'])
    if not sum(counts.values()):
        print('error: RTKLIB produced no solution', file=sys.stderr)
        return 1

    # 5. Fusion with the live solution.
    print('[5/5] fusion')
    fuse_args = argparse.Namespace(bag=args.bag, pos=pos, out=os.path.join(args.out_dir, name + '_ppk.bag'),
                                   csv=os.path.join(args.out_dir, name + '_ppk.csv'), driver_ns=args.driver_ns)
    status = ppk_fuse.run(fuse_args, ppk_fuse.load_config(config['fuse_config']))
    if status == 0:
        print('done: %s, %s' % (fuse_args.out, fuse_args.csv))
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--bag', required=True, help='bag of the mission with the driver topics')
    parser.add_argument('--out-dir', required=True, help='directory for the .pos, the fused bag and the CSV')
    parser.add_argument('--ubx', nargs='*', default=[],
                        help='rover .ubx file(s), in time order (default: rebuilt from the bag)')
    parser.add_argument('--base-obs', nargs='*', default=[],
                        help='reference station RINEX observation file(s) (default: downloaded)')
    parser.add_argument('--base-nav', nargs='*', default=[], help='reference station RINEX navigation file(s)')
    parser.add_argument('--config', default=package_config('ppk_pipeline.yaml'), help='settings (%(default)s)')
    parser.add_argument('--driver-ns', default=ppk_fuse.DRIVER_NS, help='namespace of the driver topics (%(default)s)')
    args = parser.parse_args(argv)
    # Keep the step lines and the warnings of the stages in order when the output is piped.
    sys.stdout.reconfigure(line_buffering=True)
    try:
        return run(args, load_config(args.config))
    except (base_download.NotPublishedError, FileNotFoundError, KeyError, subprocess.CalledProcessError) as e:
        print('error: %s' % e, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
