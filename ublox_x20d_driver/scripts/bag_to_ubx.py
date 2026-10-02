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
"""Rebuild a .ubx raw observation file from the rxm_rawx and rxm_sfrbx topics of a bag.

  bag_to_ubx.py --bag mission.bag --out mission.ubx

The frames are written in the order the bag recorded them and are identical to the ones
the receiver sent, so the result is a valid input for RTKLIB (convbin -r ubx) when only
the bag of a mission was kept.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ubx_raw  # noqa: E402

DRIVER_NS = '/ublox_x20d'


def write_ubx(bag_path, out_path, ns=DRIVER_NS):
    """Returns the number of (RAWX, SFRBX) frames written."""
    import rosbag

    rawx_topic, sfrbx_topic = ns + '/rxm_rawx', ns + '/rxm_sfrbx'
    counts = {rawx_topic: 0, sfrbx_topic: 0}
    with rosbag.Bag(bag_path) as bag, open(out_path, 'wb') as out:
        for topic, msg, _ in bag.read_messages(topics=[rawx_topic, sfrbx_topic]):
            out.write(ubx_raw.rawx_frame(msg) if topic == rawx_topic else ubx_raw.sfrbx_frame(msg))
            counts[topic] += 1
    return counts[rawx_topic], counts[sfrbx_topic]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--bag', required=True, help='bag with the raw observation topics')
    parser.add_argument('--out', required=True, help='.ubx file to write')
    parser.add_argument('--driver-ns', default=DRIVER_NS, help='namespace of the driver topics (%(default)s)')
    args = parser.parse_args(argv)
    rawx, sfrbx = write_ubx(args.bag, args.out, args.driver_ns)
    print('%s: %d RXM-RAWX and %d RXM-SFRBX frames' % (args.out, rawx, sfrbx))
    if rawx == 0:
        print('no %s/rxm_rawx messages: was enable_raw_observables set?' % args.driver_ns, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
