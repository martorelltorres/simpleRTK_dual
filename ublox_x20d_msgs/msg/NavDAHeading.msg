# UBX-NAV-DAHEADING (class 0x01, id 0x45), payload version 0x02 (60 bytes).
# Dual-antenna heading: relative position of antenna 2 (GPS2) with respect to
# antenna 1 (GPS1) in the local NED frame, computed by the module itself.
# Published for every parsed frame, valid or not; consumers filter on the flags.

uint8 CARR_SOLN_NONE = 0     # no carrier phase range solution
uint8 CARR_SOLN_FLOAT = 1    # carrier phase solution with floating ambiguities
uint8 CARR_SOLN_FIXED = 2    # carrier phase solution with fixed ambiguities

std_msgs/Header header       # stamp: host time at frame reception

uint8 version                # message version, always 0x02
uint32 itow                  # ms, GPS time of week of the navigation epoch

int32 rel_pos_n              # mm, north component of the GPS1 -> GPS2 vector
int32 rel_pos_e              # mm, east component
int32 rel_pos_d              # mm, down component
int32 rel_pos_length         # mm, length of the GPS1 -> GPS2 vector
int32 rel_pos_heading        # 1e-5 deg, clockwise from true north to the horizontal
                             # projection of GPS1 -> GPS2, range [0, 360);
                             # already includes CFG_NAVSPG_DAHEADING_OFFSET

uint32 acc_n                 # mm, accuracy of rel_pos_n
uint32 acc_e                 # mm, accuracy of rel_pos_e
uint32 acc_d                 # mm, accuracy of rel_pos_d
uint32 acc_length            # mm, accuracy of rel_pos_length
uint32 acc_heading           # 1e-5 deg, accuracy of rel_pos_heading

uint32 flags                 # raw flags word, as received
bool gnss_fix_ok             # flags bit 0: fix within DOP and accuracy masks
bool diff_soln               # flags bit 1: differential corrections applied
bool rel_pos_valid           # flags bit 2: relative position, accuracies and length valid
uint8 carr_soln              # flags bits 4:3, see CARR_SOLN_*
bool rel_pos_heading_valid   # flags bit 6: rel_pos_heading valid
