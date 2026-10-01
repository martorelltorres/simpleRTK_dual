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
"""Cross-check a recording of ublox_x20d_driver against the raw UBX stream it came from.

Record a bag of the driver topics while the driver logs raw frames
(enable_raw_observables:=true, raw_log_content:=all), then:

  check_recording.py --bag run.bag --ubx ~/.ros/ubx/x20d_20260918_120000.ubx

The .ubx files are decoded with pyubx2 (pip install pyubx2), independently of the
driver, and every NAV-PVT, NAV-HPPOSLLH and NAV-DAHEADING frame is compared field by
field with the message the driver published for it. pyubx2 does not know NAV-DAHEADING;
that payload is unpacked here from its documented layout, so for it the comparison checks
the driver's publishing path but not the layout itself. The physical consistency checks
(heading against the baseline vector, length against its norm) cover the layout.

It also checks the published NavSatFix, velocity and Imu against the frames, and prints
statistics used by the bring-up checklist: rates, gaps, payload lengths and versions,
carrier-phase solution, heading accuracy, baseline length, flag bits and heading of motion
against dual-antenna heading.

Exit status: 0 when every comparison and consistency check passes, 1 otherwise, 2 on a
usage or input error.
"""

import argparse
import math
import statistics
import struct
import sys
from collections import Counter

DRIVER_NS = '/ublox_x20d'
IMU_TOPIC = '/heading_imu/imu'
UNOBSERVED_VARIANCE = 1e6

# NAV-DAHEADING version 2 payload (60 bytes).
DAHEADING_FORMAT = '<B3xIiiiii4xIIIII4xI'
DAHEADING_FIELDS = ('version', 'itow', 'rel_pos_n', 'rel_pos_e', 'rel_pos_d', 'rel_pos_length',
                    'rel_pos_heading', 'acc_n', 'acc_e', 'acc_d', 'acc_length', 'acc_heading', 'flags')


def wrap_deg(angle):
    """Wrap to (-180, 180]."""
    angle = math.fmod(angle, 360.0)
    if angle > 180.0:
        angle -= 360.0
    elif angle <= -180.0:
        angle += 360.0
    return angle


def wrap_rad(angle):
    return math.radians(wrap_deg(math.degrees(angle)))


def yaw_pitch(q):
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return yaw, pitch


def heading_vector_mismatch(m, receiver_offset_deg):
    """Difference between the reported heading and the direction of the baseline vector.

    Returns None when the epoch carries no heading to compare (heading not valid, or no
    horizontal baseline). The tolerance follows from the 1 mm resolution of relPosN/E across
    the horizontal baseline, which can be much shorter than relPosLength.
    Returns (difference_deg, tolerance_deg, vector_deg) otherwise.
    """
    horizontal = math.hypot(m.rel_pos_n, m.rel_pos_e)
    if not m.rel_pos_valid or not m.rel_pos_heading_valid or horizontal <= 0:
        return None
    heading = m.rel_pos_heading * 1e-5 - receiver_offset_deg
    vector = math.degrees(math.atan2(m.rel_pos_e, m.rel_pos_n))
    tol = math.degrees(1.0 / horizontal) + 0.01
    return abs(wrap_deg(heading - vector)), tol, vector


def stamp_key(header):
    return (header.stamp.secs, header.stamp.nsecs)


class Report:
    def __init__(self, max_lines):
        self.max_lines = max_lines
        self.failures = Counter()
        self.examples = {}
        self.lines = []

    def fail(self, category, text):
        self.failures[category] += 1
        self.examples.setdefault(category, [])
        if len(self.examples[category]) < self.max_lines:
            self.examples[category].append(text)

    def info(self, text=''):
        self.lines.append(text)

    def ok(self):
        return not self.failures

    def render(self):
        out = list(self.lines)
        out.append('')
        if self.failures:
            out.append('FAILURES')
            for category, count in sorted(self.failures.items()):
                out.append('  %s: %d' % (category, count))
                for text in self.examples[category]:
                    out.append('    ' + text)
        out.append('RESULT: %s' % ('PASS' if self.ok() else 'FAIL'))
        return '\n'.join(out)


# --- UBX side -------------------------------------------------------------------------


class UbxData:
    def __init__(self):
        self.pvt = {}
        self.hpposllh = {}
        self.daheading = {}
        self.identities = Counter()
        self.pvt_lengths = Counter()
        self.hpposllh_versions = Counter()
        self.daheading_versions = Counter()
        self.errors = 0


def read_ubx(paths):
    from pyubx2 import ERR_LOG, UBX_PROTOCOL, VALCKSUM, UBXReader

    data = UbxData()

    def on_error(err):
        data.errors += 1

    for path in paths:
        with open(path, 'rb') as stream:
            reader = UBXReader(stream, protfilter=UBX_PROTOCOL, validate=VALCKSUM, quitonerror=ERR_LOG,
                               errorhandler=on_error)
            for raw, msg in reader:
                if msg is None:
                    continue
                cls, mid = raw[2], raw[3]
                payload = raw[6:-2]
                data.identities['NAV-DAHEADING' if (cls, mid) == (0x01, 0x45) else msg.identity] += 1
                if (cls, mid) == (0x01, 0x07):
                    data.pvt_lengths[len(payload)] += 1
                    data.pvt[int(msg.iTOW)] = msg
                elif (cls, mid) == (0x01, 0x14):
                    data.hpposllh_versions[msg.version] += 1
                    data.hpposllh[int(msg.iTOW)] = msg
                elif (cls, mid) == (0x01, 0x45):
                    version = payload[0] if payload else -1
                    data.daheading_versions[version] += 1
                    if version == 2 and len(payload) >= 60:
                        fields = dict(zip(DAHEADING_FIELDS, struct.unpack_from(DAHEADING_FORMAT, payload)))
                        data.daheading[fields['itow']] = fields
    return data


# --- bag side -------------------------------------------------------------------------


def read_bag(path, ns, imu_topic, from_s, to_s):
    import rosbag

    topics = {ns + '/nav_pvt': 'pvt', ns + '/nav_hpposllh': 'hpposllh', ns + '/nav_daheading': 'daheading',
              ns + '/fix': 'fix', ns + '/vel': 'vel', imu_topic: 'imu'}
    msgs = {name: [] for name in topics.values()}
    start = None
    with rosbag.Bag(path) as bag:
        present = set(bag.get_type_and_topic_info().topics)
        for topic, msg, t in bag.read_messages(topics=list(topics)):
            if start is None:
                start = t.to_sec()
            rel = t.to_sec() - start
            if (from_s is not None and rel < from_s) or (to_s is not None and rel > to_s):
                continue
            msgs[topics[topic]].append(msg)
    available = {name for topic, name in topics.items() if topic in present}
    return msgs, available


# --- field-by-field comparison ----------------------------------------------------------


def compare_fields(report, category, itow, pairs):
    for name, ros_value, ubx_value, tol in pairs:
        if tol is None:
            same = int(ros_value) == int(ubx_value)
        else:
            same = abs(float(ros_value) - float(ubx_value)) <= tol
        if not same:
            report.fail(category, 'iTOW %d %s: published %r, frame %r' % (itow, name, ros_value, ubx_value))


def compare_pvt(report, m, u):
    compare_fields(report, 'NAV-PVT field mismatch', m.itow, [
        ('year', m.year, u.year, None), ('month', m.month, u.month, None), ('day', m.day, u.day, None),
        ('hour', m.hour, u.hour, None), ('min', m.min, u.min, None), ('sec', m.sec, u.second, None),
        ('nano', m.nano, u.nano, None), ('t_acc', m.t_acc, u.tAcc, None),
        ('valid_date', m.valid_date, u.validDate, None), ('valid_time', m.valid_time, u.validTime, None),
        ('fully_resolved', m.fully_resolved, u.fullyResolved, None),
        ('fix_type', m.fix_type, u.fixType, None), ('gnss_fix_ok', m.gnss_fix_ok, u.gnssFixOk, None),
        ('diff_soln', m.diff_soln, u.diffSoln, None), ('carr_soln', m.carr_soln, u.carrSoln, None),
        ('num_sv', m.num_sv, u.numSV, None),
        ('lon', m.lon * 1e-7, u.lon, 0.6e-7), ('lat', m.lat * 1e-7, u.lat, 0.6e-7),
        ('height', m.height, u.height, None), ('hmsl', m.hmsl, u.hMSL, None),
        ('h_acc', m.h_acc, u.hAcc, None), ('v_acc', m.v_acc, u.vAcc, None),
        ('invalid_llh', m.invalid_llh, u.invalidLlh, None),
        ('vel_n', m.vel_n, u.velN, None), ('vel_e', m.vel_e, u.velE, None), ('vel_d', m.vel_d, u.velD, None),
        ('g_speed', m.g_speed, u.gSpeed, None), ('head_mot', m.head_mot * 1e-5, u.headMot, 0.6e-5),
        ('s_acc', m.s_acc, u.sAcc, None), ('head_acc', m.head_acc * 1e-5, u.headAcc, 0.6e-5),
        ('p_dop', m.p_dop * 0.01, u.pDOP, 0.006),
    ])


def compare_hpposllh(report, m, u):
    compare_fields(report, 'NAV-HPPOSLLH field mismatch', m.itow, [
        ('version', m.version, u.version, None), ('invalid_llh', m.invalid_llh, u.invalidLlh, None),
        ('lon', m.lon * 1e-7 + m.lon_hp * 1e-9, u.lon, 0.6e-9),
        ('lat', m.lat * 1e-7 + m.lat_hp * 1e-9, u.lat, 0.6e-9),
        ('height', m.height + m.height_hp * 0.1, u.height, 0.06),
        ('hmsl', m.hmsl + m.hmsl_hp * 0.1, u.hMSL, 0.06),
        ('h_acc', m.h_acc * 0.1, u.hAcc, 0.06), ('v_acc', m.v_acc * 0.1, u.vAcc, 0.06),
    ])


def compare_daheading(report, m, u):
    flags = u['flags']
    pairs = [(name, getattr(m, name), u[name], None) for name in DAHEADING_FIELDS]
    pairs += [
        ('gnss_fix_ok', m.gnss_fix_ok, flags & 0x01, None),
        ('diff_soln', m.diff_soln, (flags >> 1) & 0x01, None),
        ('rel_pos_valid', m.rel_pos_valid, (flags >> 2) & 0x01, None),
        ('carr_soln', m.carr_soln, (flags >> 3) & 0x03, None),
        ('rel_pos_heading_valid', m.rel_pos_heading_valid, (flags >> 6) & 0x01, None),
    ]
    compare_fields(report, 'NAV-DAHEADING field mismatch', m.itow, pairs)


def compare_all(report, ubx, msgs, available):
    """Every published NAV message must match its frame, and every frame within the time
    span of the bag must have been published."""
    for name, frames, compare in (('pvt', ubx.pvt, compare_pvt), ('hpposllh', ubx.hpposllh, compare_hpposllh),
                                  ('daheading', ubx.daheading, compare_daheading)):
        if name not in available:
            report.info('  %-12s not in the bag, not compared' % name)
            continue
        published = {m.itow: m for m in msgs[name]}
        matched = 0
        for itow, m in published.items():
            if itow not in frames:
                report.fail('published without a frame', '%s iTOW %d' % (name, itow))
                continue
            compare(report, m, frames[itow])
            matched += 1
        if published:
            lo, hi = min(published), max(published)
            missing = [itow for itow in frames if lo <= itow <= hi and itow not in published]
            for itow in missing:
                report.fail('frame not published', '%s iTOW %d' % (name, itow))
        report.info('  %-12s %d published, %d matched to frames' % (name, len(published), matched))


# --- consistency of derived outputs -------------------------------------------------------


def check_consistency(report, msgs, args):
    daheading_by_stamp = {stamp_key(m.header): m for m in msgs['daheading']}
    hp_by_stamp = {stamp_key(m.header): m for m in msgs['hpposllh']}
    pvt_by_stamp = {stamp_key(m.header): m for m in msgs['pvt']}
    pvt_by_itow = {m.itow: m for m in msgs['pvt']}
    hp_by_itow = {m.itow: m for m in msgs['hpposllh']}

    # Heading against the baseline vector, and length against its norm.
    checked = 0
    for m in msgs['daheading']:
        if not m.rel_pos_valid or m.rel_pos_length <= 0:
            continue
        checked += 1
        mismatch = heading_vector_mismatch(m, args.receiver_offset_deg)
        if mismatch is not None and mismatch[0] > mismatch[1]:
            report.fail('heading vs baseline vector', 'iTOW %d: heading %.3f, atan2(E, N) %.3f deg' %
                        (m.itow, m.rel_pos_heading * 1e-5 - args.receiver_offset_deg, mismatch[2]))
        norm = math.sqrt(m.rel_pos_n ** 2 + m.rel_pos_e ** 2 + m.rel_pos_d ** 2)
        if abs(norm - m.rel_pos_length) > 1.5:
            report.fail('baseline length vs vector norm', 'iTOW %d: length %d, norm %.1f mm' %
                        (m.itow, m.rel_pos_length, norm))
    report.info('  heading/length vs vector: %d epochs' % checked)

    # Imu against the NAV-DAHEADING it came from (same header stamp).
    checked = 0
    for imu in msgs['imu']:
        m = daheading_by_stamp.get(stamp_key(imu.header))
        if m is None:
            continue
        checked += 1
        yaw, pitch = yaw_pitch(imu.orientation)
        heading = math.radians(m.rel_pos_heading * 1e-5 + args.software_offset_deg)
        expected_yaw = wrap_rad(math.pi / 2 - heading)
        if abs(wrap_rad(yaw - expected_yaw)) > 1e-6:
            report.fail('Imu yaw', 'iTOW %d: yaw %.6f, expected %.6f rad' % (m.itow, yaw, expected_yaw))
        yaw_only = imu.orientation_covariance[4] >= UNOBSERVED_VARIANCE
        if not yaw_only and m.rel_pos_length > 0:
            expected_pitch = math.asin(max(-1.0, min(1.0, m.rel_pos_d / m.rel_pos_length)))
            if abs(pitch - expected_pitch) > 1e-6:
                report.fail('Imu pitch', 'iTOW %d: pitch %.6f, expected %.6f rad' % (m.itow, pitch, expected_pitch))
        if imu.orientation_covariance[0] != UNOBSERVED_VARIANCE:
            report.fail('Imu roll variance', 'iTOW %d: %r' % (m.itow, imu.orientation_covariance[0]))
    if msgs['imu']:
        unmatched = len(msgs['imu']) - checked
        report.info('  Imu vs nav_daheading: %d checked, %d without a matching nav_daheading' %
                    (checked, unmatched))

    # NavSatFix against the high-precision position of its epoch.
    checked = 0
    for fix in msgs['fix']:
        key = stamp_key(fix.header)
        source = hp_by_stamp.get(key) or pvt_by_stamp.get(key)
        hp = hp_by_itow.get(source.itow) if source is not None else None
        if hp is None:
            continue
        checked += 1
        lat = hp.lat * 1e-7 + hp.lat_hp * 1e-9
        lon = hp.lon * 1e-7 + hp.lon_hp * 1e-9
        alt = (hp.height + hp.height_hp * 0.1) * 1e-3
        if abs(fix.latitude - lat) > 1e-9 or abs(fix.longitude - lon) > 1e-9 or abs(fix.altitude - alt) > 1e-6:
            report.fail('NavSatFix position', 'iTOW %d' % hp.itow)
    if msgs['fix']:
        report.info('  fix vs nav_hpposllh: %d checked' % checked)

    # Velocity against NAV-PVT of the same stamp.
    checked = 0
    for vel in msgs['vel']:
        pvt = pvt_by_stamp.get(stamp_key(vel.header))
        if pvt is None:
            continue
        checked += 1
        v = vel.twist.twist.linear
        if abs(v.x - pvt.vel_e * 1e-3) > 1e-9 or abs(v.y - pvt.vel_n * 1e-3) > 1e-9 or \
                abs(v.z + pvt.vel_d * 1e-3) > 1e-9:
            report.fail('velocity ENU', 'iTOW %d' % pvt.itow)
    if msgs['vel']:
        report.info('  vel vs nav_pvt: %d checked' % checked)
    return pvt_by_itow


# --- statistics -------------------------------------------------------------------------


def percentile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100.0 * (len(ordered) - 1))))]


def circular_stats_deg(values):
    s = sum(math.sin(math.radians(v)) for v in values) / len(values)
    c = sum(math.cos(math.radians(v)) for v in values) / len(values)
    mean = math.degrees(math.atan2(s, c)) % 360.0
    spread = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(1e-12, math.hypot(s, c))))))
    return mean, spread


def print_statistics(report, ubx, msgs, pvt_by_itow, args):
    report.info('UBX stream')
    report.info('  frames: ' + ', '.join('%s %d' % kv for kv in sorted(ubx.identities.items())))
    report.info('  decode/checksum errors: %d' % ubx.errors)
    report.info('  NAV-PVT payload lengths: %s' % dict(ubx.pvt_lengths))
    report.info('  NAV-HPPOSLLH versions: %s' % dict(ubx.hpposllh_versions))
    report.info('  NAV-DAHEADING versions: %s' % dict(ubx.daheading_versions))
    itows = sorted(ubx.pvt)
    if len(itows) > 2:
        periods = [b - a for a, b in zip(itows, itows[1:])]
        period = statistics.median(periods)
        gaps = [(a, b) for a, b in zip(itows, itows[1:]) if b - a > 1.5 * period]
        report.info('  NAV-PVT period %.0f ms, %d gaps longer than 1.5 periods%s' %
                    (period, len(gaps), (': first at iTOW %d' % gaps[0][0]) if gaps else ''))

    dah = msgs['daheading']
    if dah:
        report.info('Heading (%d nav_daheading)' % len(dah))
        carr = Counter(m.carr_soln for m in dah)
        report.info('  carr_soln share: ' + ', '.join('%d: %.1f%%' % (k, 100.0 * v / len(dah))
                                                        for k, v in sorted(carr.items())))
        valid = [m for m in dah if m.gnss_fix_ok and m.rel_pos_heading_valid]
        report.info('  heading valid: %.1f%%' % (100.0 * len(valid) / len(dah)))
        fixed = [m for m in valid if m.carr_soln == 2]
        if fixed:
            acc = [m.acc_heading * 1e-5 for m in fixed]
            lengths = [m.rel_pos_length * 1e-3 for m in fixed]
            mean, spread = circular_stats_deg([m.rel_pos_heading * 1e-5 for m in fixed])
            report.info('  fixed epochs: %d; accHeading median %.2f deg, p95 %.2f deg' %
                        (len(fixed), statistics.median(acc), percentile(acc, 95)))
            report.info('  heading mean %.2f deg, circular std %.2f deg' % (mean, spread))
            length_std = statistics.pstdev(lengths) if len(lengths) > 1 else 0.0
            report.info('  baseline length mean %.4f m, std %.4f m' % (statistics.mean(lengths), length_std))
            if args.tape_length_m:
                report.info('  baseline length - tape: %+.4f m' % (statistics.mean(lengths) - args.tape_length_m))
            pitches = [math.degrees(math.asin(max(-1.0, min(1.0, m.rel_pos_d / m.rel_pos_length))))
                       for m in fixed if m.rel_pos_length > 0]
            if pitches:
                report.info('  baseline pitch (asin(D/L)) mean %+.2f deg, std %.2f deg' %
                            (statistics.mean(pitches), statistics.pstdev(pitches)))
        bits = Counter()
        transitions = Counter()
        previous = None
        for m in dah:
            for bit in range(32):
                if m.flags >> bit & 1:
                    bits[bit] += 1
            if previous is not None:
                for bit in range(32):
                    if (m.flags >> bit & 1) != (previous >> bit & 1):
                        transitions[bit] += 1
            previous = m.flags
        report.info('  flags bits set (share, transitions): ' + ', '.join(
            'bit %d %.0f%% %dx' % (bit, 100.0 * bits[bit] / len(dah), transitions[bit]) for bit in sorted(bits)))

        # Heading of motion (Doppler) against dual-antenna heading while moving.
        moving = []
        for m in fixed:
            pvt = pvt_by_itow.get(m.itow)
            if pvt is not None and pvt.g_speed * 1e-3 >= args.min_speed:
                moving.append(wrap_deg(pvt.head_mot * 1e-5 - m.rel_pos_heading * 1e-5))
        if moving:
            report.info('  headMot - relPosHeading while moving >= %.1f m/s: %d epochs, mean %+.2f deg, std %.2f deg'
                        % (args.min_speed, len(moving), statistics.mean(moving),
                           statistics.pstdev(moving) if len(moving) > 1 else 0.0))

    if msgs['imu']:
        yaws = [math.degrees(yaw_pitch(m.orientation)[0]) for m in msgs['imu']]
        pitches = [math.degrees(yaw_pitch(m.orientation)[1]) for m in msgs['imu']]
        mean, spread = circular_stats_deg(yaws)
        report.info('Imu (%d): yaw mean %.2f deg (circular std %.2f), pitch mean %+.2f deg' %
                    (len(yaws), wrap_deg(mean), spread, statistics.mean(pitches)))

    if msgs['fix']:
        status = Counter(m.status.status for m in msgs['fix'])
        report.info('NavSatFix (%d): status %s' % (len(msgs['fix']), dict(status)))
        pvts = list(pvt_by_itow.values())
        if pvts:
            report.info('  hAcc median %.3f m, vAcc median %.3f m' % (
                statistics.median(p.h_acc * 1e-3 for p in pvts), statistics.median(p.v_acc * 1e-3 for p in pvts)))
        if len(msgs['fix']) > 1:
            lat0 = math.radians(msgs['fix'][0].latitude)
            north = [math.radians(m.latitude) * 6378137.0 for m in msgs['fix']]
            east = [math.radians(m.longitude) * 6378137.0 * math.cos(lat0) for m in msgs['fix']]
            report.info('  position std: north %.3f m, east %.3f m, up %.3f m' % (
                statistics.pstdev(north), statistics.pstdev(east), statistics.pstdev(m.altitude for m in msgs['fix'])))


def run(args):
    report = Report(args.max_lines)
    try:
        ubx = read_ubx(args.ubx)
    except ImportError:
        print('pyubx2 is required: pip install pyubx2', file=sys.stderr)
        return report, 2
    msgs, available = read_bag(args.bag, args.driver_ns, args.imu_topic, args.from_s, args.to_s)

    # Only frames within the analysed span of the bag.
    published_itows = [m.itow for name in ('pvt', 'hpposllh', 'daheading') for m in msgs[name]]
    if published_itows:
        lo, hi = min(published_itows), max(published_itows)
        for frames in (ubx.pvt, ubx.hpposllh, ubx.daheading):
            for itow in [i for i in frames if i < lo or i > hi]:
                del frames[itow]

    report.info('Comparison with the raw frames (pyubx2; NAV-DAHEADING unpacked from its documented layout)')
    compare_all(report, ubx, msgs, available)
    report.info('Consistency of derived outputs')
    pvt_by_itow = check_consistency(report, msgs, args)
    print_statistics(report, ubx, msgs, pvt_by_itow, args)
    return report, 0 if report.ok() else 1


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--bag', required=True, help='bag with the driver topics')
    parser.add_argument('--ubx', required=True, nargs='+', help='.ubx file(s) recorded at the same time, in order')
    parser.add_argument('--driver-ns', default=DRIVER_NS, help='namespace of the driver topics (%(default)s)')
    parser.add_argument('--imu-topic', default=IMU_TOPIC, help='heading Imu topic (%(default)s)')
    parser.add_argument('--receiver-offset-deg', type=float, default=0.0,
                        help='receiver_heading_offset_deg used during the recording')
    parser.add_argument('--software-offset-deg', type=float, default=0.0,
                        help='heading_offset_deg of heading_imu_node during the recording')
    parser.add_argument('--tape-length-m', type=float, default=None, help='measured antenna distance')
    parser.add_argument('--from-s', type=float, default=None, help='start of the analysed span, s from bag start')
    parser.add_argument('--to-s', type=float, default=None, help='end of the analysed span, s from bag start')
    parser.add_argument('--min-speed', type=float, default=0.8,
                        help='minimum ground speed for the heading of motion comparison, m/s (%(default)s)')
    parser.add_argument('--max-lines', type=int, default=10, help='examples listed per failure type')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        report, status = run(args)
    except (IOError, OSError) as err:
        print('error: %s' % err, file=sys.stderr)
        return 2
    if status == 2:
        return 2
    print(report.render())
    return status


if __name__ == '__main__':
    sys.exit(main())
