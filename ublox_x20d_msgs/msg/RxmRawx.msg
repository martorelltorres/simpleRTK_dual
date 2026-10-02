# UBX-RXM-RAWX (class 0x02, id 0x15): raw GNSS measurements of one epoch.
# Published by ublox_x20d_node with enable_raw_observables. The same frames are written to
# the .ubx raw log; scripts/bag_to_ubx.py rebuilds such a file from these messages.

std_msgs/Header header       # stamp: host time at frame reception

float64 rcv_tow              # s, receiver time of week of the measurements (GPS time)
uint16 week                  # GPS week number
int8 leap_s                  # s, GPS leap seconds (GPS - UTC)

uint8 rec_stat               # raw receiver tracking status bitfield, as received
bool leap_sec                # rec_stat bit 0: leap seconds determined
bool clk_reset               # rec_stat bit 1: clock reset applied

uint8 version                # message version
uint16 reserved0             # payload bytes 14-15, documented as reserved; on HDG 2.00 they
                             # advance by 1000 per epoch. Kept to rebuild the frame exactly

RawxMeas[] meas              # one entry per tracked signal
