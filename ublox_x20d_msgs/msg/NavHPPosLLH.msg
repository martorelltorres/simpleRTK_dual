# UBX-NAV-HPPOSLLH (class 0x01, id 0x14): high-precision geodetic position.
# Full-precision values:
#   lat_deg    = lat * 1e-7 + lat_hp * 1e-9
#   lon_deg    = lon * 1e-7 + lon_hp * 1e-9
#   height_mm  = height + height_hp * 0.1
#   hmsl_mm    = hmsl + hmsl_hp * 0.1

std_msgs/Header header       # stamp: host time at frame reception

uint8 version                # message version
uint32 itow                  # ms, GPS time of week of the navigation epoch

int32 lon                    # 1e-7 deg
int32 lat                    # 1e-7 deg
int32 height                 # mm, above ellipsoid
int32 hmsl                   # mm, above mean sea level
int8 lon_hp                  # 1e-9 deg
int8 lat_hp                  # 1e-9 deg
int8 height_hp               # 0.1 mm
int8 hmsl_hp                 # 0.1 mm

uint32 h_acc                 # 0.1 mm, horizontal accuracy estimate
uint32 v_acc                 # 0.1 mm, vertical accuracy estimate

bool invalid_llh             # flags bit 0: all position fields above are invalid
