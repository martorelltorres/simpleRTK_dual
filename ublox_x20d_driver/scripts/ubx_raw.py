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
"""UBX encoding of ublox_x20d_msgs/RxmRawx and RxmSfrbx messages.

Rebuilds the RXM-RAWX and RXM-SFRBX payloads the driver parsed, byte for byte, so that a
bag can be turned back into a .ubx file and published messages can be checked against the
frames they came from. No ROS dependency: any object with the message fields works.
"""

import struct

RXM_CLASS = 0x02
RAWX_ID = 0x15
SFRBX_ID = 0x13

_RAWX_HEADER = struct.Struct('<dHbBBBH')
_RAWX_MEAS = struct.Struct('<ddfBBBBHBBBBBB')
_SFRBX_HEADER = struct.Struct('<BBBBBBBB')


def frame(msg_class, msg_id, payload):
    """Complete UBX frame: sync, class, id, length, payload and Fletcher checksum."""
    body = struct.pack('<BBH', msg_class, msg_id, len(payload)) + bytes(payload)
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes((ck_a, ck_b))


def rawx_payload(msg):
    out = bytearray(_RAWX_HEADER.pack(msg.rcv_tow, msg.week, msg.leap_s, len(msg.meas), msg.rec_stat,
                                      msg.version, msg.reserved0))
    for m in msg.meas:
        out += _RAWX_MEAS.pack(m.pr_mes, m.cp_mes, m.do_mes, m.gnss_id, m.sv_id, m.sig_id, m.freq_id,
                               m.locktime, m.cno, m.pr_stdev, m.cp_stdev, m.do_stdev, m.trk_stat, 0)
    return bytes(out)


def sfrbx_payload(msg):
    out = bytearray(_SFRBX_HEADER.pack(msg.gnss_id, msg.sv_id, msg.sig_id, msg.freq_id, len(msg.dwrd),
                                       msg.chn, msg.version, msg.reserved0))
    for word in msg.dwrd:
        out += struct.pack('<I', word)
    return bytes(out)


def rawx_frame(msg):
    return frame(RXM_CLASS, RAWX_ID, rawx_payload(msg))


def sfrbx_frame(msg):
    return frame(RXM_CLASS, SFRBX_ID, sfrbx_payload(msg))
