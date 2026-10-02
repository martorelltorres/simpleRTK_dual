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
"""Fuse a PPK solution with the GNSS output of the driver recorded in a mission bag.

  ppk_fuse.py --bag mission.bag --pos mission.pos --out mission_ppk.bag --csv mission_ppk.csv

Inputs: the bag of the mission (nav_pvt and nav_hpposllh of ublox_x20d_node) and the .pos
file of ppk_process.py (RTKLIB, GPS time, ellipsoidal height). Epochs are matched by GPS
time (NAV-PVT iTOW), not by the host clock.

A Kalman filter over position, velocity and the bias of the live position in a local
east-north-up frame, followed by a Rauch-Tung-Striebel smoother, combines the PPK
positions with the live position (hAcc, vAcc) and the Doppler velocity (sAcc) of the
driver. PPK dominates wherever it is available and makes the live bias observable; gaps
and rejected epochs are bridged by the velocity and the bias-corrected live position.
Measurements that disagree with the filter beyond a chi-square gate are rejected.

The output bag holds every message of the input bag plus:
  <ns>/fix_ppk    sensor_msgs/NavSatFix, one per PPK epoch used (covariance from RTKLIB)
  <ns>/fix_fused  sensor_msgs/NavSatFix, one per epoch, smoothed (full 3x3 covariance)
Both are positions of the antenna whose raw observations were processed (GPS1), stamped
with the host time of the NAV-PVT of the same epoch. Settings: config/ppk_fuse.yaml.
"""

import argparse
import csv
import datetime
import math
import os
import statistics
import sys

import numpy as np

DRIVER_NS = '/ublox_x20d'

GPS_EPOCH = datetime.datetime(1980, 1, 6)
WEEK_S = 604800.0
# Only used to pick the GPS week of a NAV-PVT epoch from its UTC date; any value within a
# few seconds of the real one gives the same week.
GPS_UTC_LEAP_S = 18.0

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def package_config(name):
    """Path of a file in the config directory of this package, from the source tree or the
    installed share directory."""
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', name)
    if os.path.exists(here):
        return here
    import rospkg
    return os.path.join(rospkg.RosPack().get_path('ublox_x20d_driver'), 'config', name)


# --- time ---------------------------------------------------------------------------------


def gpst_seconds(dt):
    """Seconds since the GPS epoch of a datetime expressed in GPS time."""
    return (dt - GPS_EPOCH).total_seconds()


def week_tow(seconds):
    week = int(seconds // WEEK_S)
    return week, seconds - week * WEEK_S


def pvt_gps_seconds(pvt):
    """GPS time of a NAV-PVT epoch: its iTOW, in the week given by its UTC date."""
    utc = datetime.datetime(pvt.year, pvt.month, pvt.day, pvt.hour, pvt.min, pvt.sec)
    approx = gpst_seconds(utc) + pvt.nano * 1e-9 + GPS_UTC_LEAP_S
    tow = pvt.itow * 1e-3
    week = round((approx - tow) / WEEK_S)
    return week * WEEK_S + tow


# --- RTKLIB .pos --------------------------------------------------------------------------


def read_pos(path):
    """Epochs of an RTKLIB .pos file in llh format with GPS time in hms form.

    Returns a list of dicts: t (GPS seconds), lat, lon (deg), h (m), q, ns, sdn, sde, sdu (m).
    """
    epochs = []
    with open(path) as f:
        for line in f:
            if line.startswith('%') or not line.strip():
                continue
            fields = line.split()
            stamp = datetime.datetime.strptime(fields[0] + ' ' + fields[1], '%Y/%m/%d %H:%M:%S.%f')
            epochs.append({
                't': gpst_seconds(stamp), 'lat': float(fields[2]), 'lon': float(fields[3]), 'h': float(fields[4]),
                'q': int(fields[5]), 'ns': int(fields[6]),
                'sdn': float(fields[7]), 'sde': float(fields[8]), 'sdu': float(fields[9]),
            })
    return epochs


# --- geodesy ------------------------------------------------------------------------------


def geodetic_to_ecef(lat, lon, h):
    lat, lon = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array([(n + h) * math.cos(lat) * math.cos(lon),
                     (n + h) * math.cos(lat) * math.sin(lon),
                     (n * (1.0 - WGS84_E2) + h) * math.sin(lat)])


def ecef_to_geodetic(xyz):
    x, y, z = xyz
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + h)))
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


class LocalFrame:
    """East-north-up frame tangent to the ellipsoid at an origin."""

    def __init__(self, lat, lon, h):
        self.origin = geodetic_to_ecef(lat, lon, h)
        la, lo = math.radians(lat), math.radians(lon)
        self.rot = np.array([
            [-math.sin(lo), math.cos(lo), 0.0],
            [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
            [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])

    def to_enu(self, lat, lon, h):
        return self.rot @ (geodetic_to_ecef(lat, lon, h) - self.origin)

    def to_geodetic(self, enu):
        return ecef_to_geodetic(self.origin + self.rot.T @ np.asarray(enu))


# --- smoother -----------------------------------------------------------------------------


class Measurement:
    """A measurement in ENU with a 3x3 covariance. kind: 'pos' (PPK position), 'live' (live
    position of the driver, affected by a slowly varying bias) or 'vel' (Doppler velocity)."""

    def __init__(self, kind, value, cov, source):
        self.kind = kind
        self.value = np.asarray(value, dtype=float)
        self.cov = np.asarray(cov, dtype=float)
        self.source = source
        self.accepted = False

    def matrix(self):
        h = np.zeros((3, 9))
        if self.kind in ('pos', 'live'):
            h[:, 0:3] = np.eye(3)
        if self.kind == 'live':
            h[:, 6:9] = np.eye(3)
        if self.kind == 'vel':
            h[:, 3:6] = np.eye(3)
        return h


def transition(dt, accel_h, accel_v, bias_walk):
    """State transition and process noise over dt for [position, velocity, live bias]."""
    q_axis = np.array([accel_h, accel_h, accel_v])
    f = np.eye(9)
    f[0:3, 3:6] = dt * np.eye(3)
    q = np.zeros((9, 9))
    q[0:3, 0:3] = np.diag(q_axis * dt ** 3 / 3.0)
    q[0:3, 3:6] = q[3:6, 0:3] = np.diag(q_axis * dt ** 2 / 2.0)
    q[3:6, 3:6] = np.diag(q_axis * dt)
    q[6:9, 6:9] = np.eye(3) * bias_walk * dt
    return f, q


def smooth(times, measurements, accel_h, accel_v, gate_chi2, bias_walk=0.01, bias_sd=5.0):
    """Forward Kalman filter and RTS smoother.

    State: position and velocity (near-constant-velocity model with white acceleration noise
    accel_h, accel_v in m^2/s^3) and the bias of the live position (random walk, bias_walk in
    m^2/s, initial standard deviation bias_sd in m). Wherever PPK is available the bias is
    observable, so that across PPK gaps the live position contributes its relative motion
    rather than its absolute error.

    times: increasing epoch times (s). measurements: per epoch, a list of Measurement; their
    `accepted` attribute is set by the chi-square gate. Returns (states (N, 9),
    covariances (N, 9, 9)).
    """
    n = len(times)
    x = np.zeros(9)
    p = np.diag([1e6] * 3 + [1e2] * 3 + [bias_sd ** 2] * 3)
    initialised = False
    x_pred, p_pred = np.zeros((n, 9)), np.zeros((n, 9, 9))
    x_filt, p_filt = np.zeros((n, 9)), np.zeros((n, 9, 9))
    for k in range(n):
        if k > 0:
            f, q = transition(times[k] - times[k - 1], accel_h, accel_v, bias_walk)
            x = f @ x
            p = f @ p @ f.T + q
        x_pred[k], p_pred[k] = x, p
        # Positions before velocities, the most precise first, so that the gate of the
        # others is tested against the best state.
        for m in sorted(measurements[k], key=lambda m: (m.kind == 'vel', np.trace(m.cov))):
            if m.kind == 'vel' and not initialised:
                continue
            if not initialised:
                x[0:3] = m.value
                p[0:3, 0:3] = m.cov + (p[6:9, 6:9] if m.kind == 'live' else 0.0)
                initialised = True
                m.accepted = True
                continue
            h = m.matrix()
            y = m.value - h @ x
            s = h @ p @ h.T + m.cov
            s_inv = np.linalg.inv(s)
            if float(y @ s_inv @ y) > gate_chi2:
                continue
            gain = p @ h.T @ s_inv
            x = x + gain @ y
            p = (np.eye(9) - gain @ h) @ p
            p = 0.5 * (p + p.T)
            m.accepted = True
        x_filt[k], p_filt[k] = x, p

    x_s, p_s = x_filt.copy(), p_filt.copy()
    for k in range(n - 2, -1, -1):
        f, _ = transition(times[k + 1] - times[k], accel_h, accel_v, bias_walk)
        c = p_filt[k] @ f.T @ np.linalg.inv(p_pred[k + 1])
        x_s[k] = x_filt[k] + c @ (x_s[k + 1] - x_pred[k + 1])
        p_s[k] = p_filt[k] + c @ (p_s[k + 1] - p_pred[k + 1]) @ c.T
    return x_s, p_s


def ppk_measurement(epoch, frame, config):
    floor = config['ppk_min_sd']
    scale = config['ppk_float_sd_scale'] if epoch['q'] == 2 else 1.0
    sd = [max(epoch[k], floor) * scale for k in ('sde', 'sdn', 'sdu')]
    return Measurement('pos', frame.to_enu(epoch['lat'], epoch['lon'], epoch['h']), np.diag(np.square(sd)), 'ppk')


# --- bag side -----------------------------------------------------------------------------


def read_driver_epochs(bag_path, ns):
    """NAV-PVT and NAV-HPPOSLLH of the bag by GPS time, plus the frame id of ~fix."""
    import rosbag

    pvt, hp, frame_id = {}, {}, 'gps'
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, _ in bag.read_messages(topics=[ns + '/nav_pvt', ns + '/nav_hpposllh', ns + '/fix']):
            if topic.endswith('/nav_pvt'):
                if msg.valid_date and msg.valid_time:
                    pvt[msg.itow] = msg
            elif topic.endswith('/nav_hpposllh'):
                hp[msg.itow] = msg
            elif msg.header.frame_id:
                frame_id = msg.header.frame_id
    epochs = {}
    for itow, p in pvt.items():
        t = pvt_gps_seconds(p)
        epochs[round(t * 1000)] = {'t': t, 'pvt': p, 'hp': hp.get(itow)}
    return epochs, frame_id


def navsatfix(stamp, frame_id, lat, lon, h, cov, status):
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    msg = NavSatFix()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status.status = status
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude, msg.longitude, msg.altitude = lat, lon, h
    msg.position_covariance = [float(v) for v in np.asarray(cov).reshape(9)]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    return msg


def run(args, config):
    import rosbag
    import rospy
    from sensor_msgs.msg import NavSatStatus

    ppk = [e for e in read_pos(args.pos) if e['q'] in config['ppk_qualities']]
    driver, frame_id = read_driver_epochs(args.bag, args.driver_ns)
    if not driver:
        print('no %s/nav_pvt with a valid date in %s' % (args.driver_ns, args.bag), file=sys.stderr)
        return 2
    if not ppk:
        print('no PPK epoch with quality in %s in %s' % (config['ppk_qualities'], args.pos), file=sys.stderr)

    epochs = {key: {'t': d['t'], 'pvt': d['pvt'], 'hp': d['hp'], 'ppk': None} for key, d in driver.items()}
    for e in ppk:
        epochs.setdefault(round(e['t'] * 1000), {'t': e['t'], 'pvt': None, 'hp': None, 'ppk': None})['ppk'] = e
    keys = sorted(epochs)
    times = np.array([epochs[k]['t'] for k in keys])

    # Host time of the epochs without a NAV-PVT: GPS time plus the median clock offset.
    offsets = [d['pvt'].header.stamp.to_sec() - d['t'] for d in driver.values()]
    clock_offset = statistics.median(offsets)

    origin = ppk[0] if ppk else None
    if origin is None:
        hp0 = next(d['hp'] for d in driver.values() if d['hp'] is not None)
        origin = {'lat': hp0.lat * 1e-7 + hp0.lat_hp * 1e-9, 'lon': hp0.lon * 1e-7 + hp0.lon_hp * 1e-9,
                  'h': (hp0.height + hp0.height_hp * 0.1) * 1e-3}
    frame = LocalFrame(origin['lat'], origin['lon'], origin['h'])

    measurements = []
    for k in keys:
        e = epochs[k]
        ms = []
        if e['ppk'] is not None:
            ms.append(ppk_measurement(e['ppk'], frame, config))
        p, hp = e['pvt'], e['hp']
        if config['use_live_position'] and hp is not None and p is not None and p.gnss_fix_ok and not hp.invalid_llh:
            lat, lon = hp.lat * 1e-7 + hp.lat_hp * 1e-9, hp.lon * 1e-7 + hp.lon_hp * 1e-9
            h_var = (hp.h_acc * 1e-4) ** 2 / 2.0
            ms.append(Measurement('live', frame.to_enu(lat, lon, (hp.height + hp.height_hp * 0.1) * 1e-3),
                                  np.diag([h_var, h_var, (hp.v_acc * 1e-4) ** 2]), 'live'))
        if config['use_doppler_velocity'] and p is not None and p.gnss_fix_ok:
            ms.append(Measurement('vel', [p.vel_e * 1e-3, p.vel_n * 1e-3, -p.vel_d * 1e-3],
                                  np.eye(3) * (p.s_acc * 1e-3) ** 2, 'doppler'))
        measurements.append(ms)

    states, covs = smooth(times, measurements, config['accel_noise_horizontal'], config['accel_noise_vertical'],
                          config['gate_chi2'], config['live_bias_walk'], config['live_bias_sd'])

    counts = {'ppk': [0, 0], 'live': [0, 0], 'doppler': [0, 0]}
    for ms in measurements:
        for m in ms:
            counts[m.source][0 if m.accepted else 1] += 1

    rows = []
    with rosbag.Bag(args.bag) as bag_in, rosbag.Bag(args.out, 'w') as bag_out:
        for topic, msg, t in bag_in.read_messages():
            bag_out.write(topic, msg, t)
        for i, k in enumerate(keys):
            e = epochs[k]
            stamp_s = e['pvt'].header.stamp.to_sec() if e['pvt'] is not None else e['t'] + clock_offset
            stamp = rospy.Time.from_sec(stamp_s)
            ppk_used = any(m.source == 'ppk' and m.accepted for m in measurements[i])
            if ppk_used:
                p = e['ppk']
                cov = np.diag(np.square([p['sde'], p['sdn'], p['sdu']]))
                bag_out.write(args.driver_ns + '/fix_ppk',
                              navsatfix(stamp, frame_id, p['lat'], p['lon'], p['h'], cov,
                                        NavSatStatus.STATUS_GBAS_FIX), stamp)
            lat, lon, h = frame.to_geodetic(states[i][0:3])
            status = NavSatStatus.STATUS_GBAS_FIX if ppk_used else NavSatStatus.STATUS_FIX
            bag_out.write(args.driver_ns + '/fix_fused',
                          navsatfix(stamp, frame_id, lat, lon, h, covs[i][0:3, 0:3], status), stamp)
            week, tow = week_tow(e['t'])
            sd = np.sqrt(np.diag(covs[i][0:3, 0:3]))
            rows.append([stamp_s, week, '%.3f' % tow, '%.9f' % lat, '%.9f' % lon, '%.4f' % h,
                         '%.4f' % sd[0], '%.4f' % sd[1], '%.4f' % sd[2],
                         e['ppk']['q'] if e['ppk'] is not None else '', int(ppk_used)])
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['stamp', 'gps_week', 'gps_tow', 'lat', 'lon', 'height', 'sd_east', 'sd_north', 'sd_up',
                             'ppk_quality', 'ppk_used'])
            writer.writerows(rows)

    print('%d epochs (%d with PPK, %d from the driver)' % (len(keys), len(ppk), len(driver)))
    for source, (used, rejected) in counts.items():
        print('  %-8s %6d used, %5d rejected by the gate' % (source, used, rejected))
    return 0


def load_config(path):
    import yaml

    with open(path) as f:
        return yaml.safe_load(f)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--bag', required=True, help='bag of the mission with the driver topics')
    parser.add_argument('--pos', required=True, help='RTKLIB .pos file from ppk_process.py')
    parser.add_argument('--out', required=True, help='output bag')
    parser.add_argument('--csv', default='', help='also write the fused trajectory as CSV')
    parser.add_argument('--config', default=package_config('ppk_fuse.yaml'), help='fusion settings (%(default)s)')
    parser.add_argument('--driver-ns', default=DRIVER_NS, help='namespace of the driver topics (%(default)s)')
    args = parser.parse_args(argv)
    return run(args, load_config(args.config))


if __name__ == '__main__':
    sys.exit(main())
