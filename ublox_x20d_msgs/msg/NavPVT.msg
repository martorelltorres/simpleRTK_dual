# UBX-NAV-PVT (class 0x01, id 0x07): navigation position, velocity and time.
# Subset of the 92-byte payload; dead-reckoning (ADR) fields are not carried.

uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING = 4
uint8 FIX_TYPE_TIME_ONLY = 5

uint8 CARR_SOLN_NONE = 0     # no carrier phase range solution
uint8 CARR_SOLN_FLOAT = 1    # carrier phase solution with floating ambiguities
uint8 CARR_SOLN_FIXED = 2    # carrier phase solution with fixed ambiguities

std_msgs/Header header       # stamp: host time at frame reception

uint32 itow                  # ms, GPS time of week of the navigation epoch

# UTC time
uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
int32 nano                   # ns, fraction of second, range [-1e9, 1e9]
uint32 t_acc                 # ns, time accuracy estimate
bool valid_date              # valid bit 0
bool valid_time              # valid bit 1
bool fully_resolved          # valid bit 2: no seconds uncertainty in time of day

# Fix status
uint8 fix_type               # see FIX_TYPE_*
bool gnss_fix_ok             # flags bit 0: fix within DOP and accuracy masks
bool diff_soln               # flags bit 1: differential corrections applied
uint8 carr_soln              # flags bits 7:6, see CARR_SOLN_*
uint8 num_sv                 # satellites used in the solution

# Position (see NavHPPosLLH for the high-precision variant)
int32 lon                    # 1e-7 deg
int32 lat                    # 1e-7 deg
int32 height                 # mm, above ellipsoid
int32 hmsl                   # mm, above mean sea level
uint32 h_acc                 # mm, horizontal accuracy estimate
uint32 v_acc                 # mm, vertical accuracy estimate
bool invalid_llh             # flags3 bit 0: lon, lat, height and hmsl are invalid

# Velocity (NED)
int32 vel_n                  # mm/s
int32 vel_e                  # mm/s
int32 vel_d                  # mm/s
int32 g_speed                # mm/s, 2-D ground speed
int32 head_mot               # 1e-5 deg, 2-D heading of motion
uint32 s_acc                 # mm/s, speed accuracy estimate
uint32 head_acc              # 1e-5 deg, heading of motion accuracy estimate

uint16 p_dop                 # 0.01, position DOP
