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
"""Download of the hourly RINEX files of a reference station (scripts/ppk_pipeline.py).

Stations and networks are described in config/base_stations.yaml. Each hour gives one
observation file (Hatanaka-compressed, gzip) and one navigation file (gzip); they are
decompressed with gzip and crx2rnx (RNXCMP) and kept in a cache directory, so a second
run over the same hours downloads nothing. No ROS dependency.
"""

import datetime
import gzip
import os
import shutil
import subprocess
import urllib.error
import urllib.request

HOUR = datetime.timedelta(hours=1)


class NotPublishedError(RuntimeError):
    """The network has not published the files of an hour (yet)."""


def hours_for_span(start_utc, end_utc, margin_s=60.0):
    """UTC start of every hour that overlaps [start_utc - margin, end_utc + margin]."""
    margin = datetime.timedelta(seconds=margin_s)
    hour = (start_utc - margin).replace(minute=0, second=0, microsecond=0)
    last = end_utc + margin
    hours = []
    while hour <= last:
        hours.append(hour)
        hour += HOUR
    return hours


def file_urls(station, hour):
    """(observation URL, navigation URL) of one hour of a station of the catalog."""
    network = station['network']
    base = network['url'].format(id=station['id'], date=hour)
    return (base + network['obs'].format(id=station['id'], date=hour),
            base + network['nav'].format(id=station['id'], date=hour))


def _download(url, path):
    try:
        with urllib.request.urlopen(url, timeout=60) as response, open(path + '.part', 'wb') as f:
            shutil.copyfileobj(response, f)
    except urllib.error.HTTPError as e:
        if e.code == 404:
            raise NotPublishedError(url)
        raise
    os.replace(path + '.part', path)


def _gunzip(src, dst):
    with gzip.open(src, 'rb') as f_in, open(dst, 'wb') as f_out:
        shutil.copyfileobj(f_in, f_out)


def _strip(name, suffix):
    return name[:-len(suffix)] if name.endswith(suffix) else name


def fetch_hour(station, hour, cache_dir, crx2rnx='crx2rnx'):
    """Decompressed (observation, navigation) RINEX files of one hour, from the cache or
    downloaded. Raises NotPublishedError when the network does not have them."""
    directory = os.path.join(os.path.expanduser(cache_dir), station['id'], hour.strftime('%Y%j%H'))
    os.makedirs(directory, exist_ok=True)
    result = []
    for url in file_urls(station, hour):
        name = _strip(url.rsplit('/', 1)[1], '.gz')
        rinex = os.path.join(directory, _strip(name, '.crx') + ('.rnx' if name.endswith('.crx') else ''))
        if not os.path.exists(rinex):
            compressed = os.path.join(directory, name + '.gz')
            _download(url, compressed)
            unpacked = os.path.join(directory, name)
            _gunzip(compressed, unpacked)
            if name.endswith('.crx'):
                # crx2rnx writes <name>.rnx next to <name>.crx.
                subprocess.run([crx2rnx, '-f', unpacked], check=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                os.remove(unpacked)
            os.remove(compressed)
        result.append(rinex)
    return tuple(result)


def fetch_span(station, start_utc, end_utc, cache_dir, crx2rnx='crx2rnx', margin_s=60.0):
    """(observation files, navigation files, hours) covering a UTC time span."""
    obs, nav = [], []
    hours = hours_for_span(start_utc, end_utc, margin_s)
    for hour in hours:
        try:
            o, n = fetch_hour(station, hour, cache_dir, crx2rnx)
        except NotPublishedError as e:
            raise NotPublishedError(
                'reference station %s: hour %s UTC not published (%s). Files are published once the '
                'hour has ended; retry later or pass the RINEX files with --base-obs/--base-nav'
                % (station['name'], hour.strftime('%Y-%m-%d %H:00'), e))
        obs.append(o)
        nav.append(n)
    return obs, nav, hours
