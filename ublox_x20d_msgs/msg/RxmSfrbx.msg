# UBX-RXM-SFRBX (class 0x02, id 0x13): one broadcast navigation data subframe.
# Published by ublox_x20d_node with enable_raw_observables. RTKLIB decodes the ephemerides
# from these words; scripts/bag_to_ubx.py rebuilds the .ubx frames from them.

std_msgs/Header header       # stamp: host time at frame reception

uint8 gnss_id                # GNSS identifier, as in RawxMeas
uint8 sv_id                  # satellite identifier
uint8 sig_id                 # signal identifier
uint8 freq_id                # GLONASS frequency slot + 7; unused for other GNSS
uint8 chn                    # tracking channel the subframe was received on
uint8 version                # message version
uint8 reserved0              # payload byte 7, documented as reserved; not zero on HDG 2.00.
                             # Kept to rebuild the frame exactly

uint32[] dwrd                # data words, as received
