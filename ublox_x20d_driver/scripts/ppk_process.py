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
"""Post-process the raw observations of a mission against a reference station (PPK).

  ppk_process.py --rover x20d_*.ubx --base-obs BASE_07.rnx BASE_08.rnx --base-nav BASE.nav --out mission.pos

Runs RTKLIB: convbin turns the rover .ubx files (from the driver's raw log, or rebuilt from
a bag with bag_to_ubx.py) into RINEX, then rnx2rtkp computes the kinematic solution with
the options of config/ppk_rtklib.conf and the coordinates of the reference station from
config/base_stations.yaml. The reference station RINEX files cover the time span of the
mission; ppk_pipeline.py downloads them, or they are downloaded separately.

The output .pos file (GPS time, ellipsoidal height) is the input of ppk_fuse.py.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from collections import Counter

QUALITY = {1: 'fix', 2: 'float', 3: 'sbas', 4: 'dgps', 5: 'single', 6: 'ppp'}


def package_config(name):
    """Path of a file in the config directory of this package, from the source tree or the
    installed share directory."""
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', name)
    if os.path.exists(here):
        return here
    import rospkg
    return os.path.join(rospkg.RosPack().get_path('ublox_x20d_driver'), 'config', name)


def rinex_program(path):
    """The 'PGM / RUN BY / DATE' program field of a RINEX header, e.g. 'CONVBIN 2.4.3'."""
    with open(path, errors='replace') as f:
        for line in f:
            if 'PGM / RUN BY / DATE' in line:
                return line[:20].strip()
            if 'END OF HEADER' in line:
                break
    return ''


def is_old_rtklib(program):
    """RTKLIB 2.4.3 releases (the Ubuntu 20.04 package) decode only L1 of the ZED-X20D."""
    return program.startswith('CONVBIN 2.4.3')


def quality_counts(pos_path):
    """Count of epochs per solution quality (Q column) in an RTKLIB .pos file."""
    counts = Counter()
    with open(pos_path) as f:
        for line in f:
            if line.startswith('%') or not line.strip():
                continue
            fields = line.split()
            if len(fields) > 5:
                counts[int(fields[5])] += 1
    return counts


def load_station(name, stations_file):
    """Station entry of the catalog (config/base_stations.yaml), plus its network entry."""
    import yaml

    with open(stations_file) as f:
        catalog = yaml.safe_load(f)
    stations = catalog.get('stations') or {}
    if name not in stations:
        raise KeyError('station %s not in %s (known: %s)' % (name, stations_file, ', '.join(sorted(stations))))
    station = dict(stations[name])
    station['name'] = name
    station['network'] = dict(catalog['networks'][station['network']], name=station['network'])
    return station


def station_options(station):
    """RTKLIB options with the position and antenna height of the reference station."""
    return ('ant2-pos1          =%.9f\n'
            'ant2-pos2          =%.9f\n'
            'ant2-pos3          =%.4f\n'
            'ant2-antdelu       =%.4f\n') % (station['lat'], station['lon'], station['height'],
                                              station['antenna_height'])


def tool(name, bin_dir):
    path = os.path.join(bin_dir, name) if bin_dir else shutil.which(name)
    if not path or not os.access(path, os.X_OK):
        raise FileNotFoundError('%s not found%s' % (name, ' in ' + bin_dir if bin_dir else ' in PATH'))
    return path


def process(rover, base_obs, base_nav, conf, out, station, antex='', rtklib_bin='', keep_rinex=False):
    """Runs convbin and rnx2rtkp; returns the epoch count per quality of the .pos written."""
    convbin = tool('convbin', rtklib_bin)
    rnx2rtkp = tool('rnx2rtkp', rtklib_bin)
    work = tempfile.mkdtemp(prefix='ppk_')
    try:
        # Rotated raw log files are consecutive parts of one stream.
        rover_ubx = os.path.join(work, 'rover.ubx')
        with open(rover_ubx, 'wb') as dst:
            for path in rover:
                with open(path, 'rb') as f:
                    shutil.copyfileobj(f, dst)
        rover_obs = os.path.join(work, 'rover.obs')
        rover_nav = os.path.join(work, 'rover.nav')
        subprocess.run([convbin, '-r', 'ubx', '-v', '3.04', '-o', rover_obs, '-n', rover_nav, rover_ubx],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        program = rinex_program(rover_obs)
        print('rover RINEX written by %s' % (program or 'unknown program'))
        if is_old_rtklib(program):
            print('warning: RTKLIB 2.4.3 keeps only the L1 signals of the ZED-X20D and no Galileo '
                  'ephemerides; point --rtklib-bin at an RTKLIB-EX build (see the README)', file=sys.stderr)

        # Several base files (e.g. hourly) are one input to RTKLIB when given as a wildcard.
        base_dir = os.path.join(work, 'base')
        os.mkdir(base_dir)
        for i, path in enumerate(base_obs):
            os.symlink(os.path.abspath(path), os.path.join(base_dir, 'base_%03d.obs' % i))
        base_pattern = os.path.join(base_dir, 'base_*.obs')

        # The station and the antenna calibrations are appended to the options file; RTKLIB
        # keeps the last value of an option.
        options = os.path.join(work, 'options.conf')
        with open(conf) as src, open(options, 'w') as dst:
            dst.write(src.read())
            dst.write('\n# reference station %s\n' % station['name'])
            dst.write(station_options(station))
            if antex:
                dst.write('file-rcvantfile    =%s\n' % os.path.abspath(antex))
        if not antex:
            print('warning: no ANTEX file: the phase centre offsets of the base antenna are not '
                  'applied (heights biased by about 0.1 m)', file=sys.stderr)
        nav_files = [rover_nav] + list(base_nav)
        subprocess.run([rnx2rtkp, '-k', options, '-o', out, rover_obs, base_pattern] + nav_files,
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if keep_rinex:
            for path in (rover_obs, rover_nav):
                shutil.copy(path, os.path.splitext(out)[0] + '_' + os.path.basename(path))
    finally:
        shutil.rmtree(work)

    counts = quality_counts(out)
    total = sum(counts.values())
    print('%s: %d epochs (reference station %s)' % (out, total, station['name']))
    for q in sorted(counts):
        print('  %-6s %6d (%.1f %%)' % (QUALITY.get(q, str(q)), counts[q], 100.0 * counts[q] / total))
    return counts


def run(args):
    station = load_station(args.station, args.stations)
    counts = process(args.rover, args.base_obs, args.base_nav, args.conf, args.out, station,
                     args.antex, args.rtklib_bin, args.keep_rinex)
    return 0 if sum(counts.values()) else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--rover', nargs='+', required=True, help='rover .ubx file(s), in time order')
    parser.add_argument('--base-obs', nargs='+', required=True,
                        help='reference station RINEX observation file(s), e.g. hourly files')
    parser.add_argument('--base-nav', nargs='*', default=[], help='extra RINEX navigation file(s)')
    parser.add_argument('--conf', default=package_config('ppk_rtklib.conf'), help='RTKLIB options file (%(default)s)')
    parser.add_argument('--station', default='MAL1', help='reference station in --stations (%(default)s)')
    parser.add_argument('--stations', default=package_config('base_stations.yaml'),
                        help='reference station catalog (%(default)s)')
    parser.add_argument('--out', required=True, help='output .pos file')
    parser.add_argument('--antex', default='', help='ANTEX file with receiver antenna calibrations, e.g. igs20.atx')
    parser.add_argument('--rtklib-bin', default='', help='directory with convbin and rnx2rtkp (default: PATH)')
    parser.add_argument('--keep-rinex', action='store_true', help='keep the rover RINEX files next to --out')
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (FileNotFoundError, KeyError, subprocess.CalledProcessError) as e:
        print('error: %s' % e, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
