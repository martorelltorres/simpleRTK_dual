# One measurement of UBX-RXM-RAWX: a single signal of a single satellite.
# The ZED-X20D reports the measurements of one antenna only.

float64 pr_mes               # m, pseudorange
float64 cp_mes               # cycles, carrier phase
float32 do_mes               # Hz, Doppler

uint8 gnss_id                # GNSS identifier: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS
uint8 sv_id                  # satellite identifier within its GNSS
uint8 sig_id                 # signal identifier within its GNSS
uint8 freq_id                # GLONASS frequency slot + 7 (0..13); unused for other GNSS

uint16 locktime              # ms, carrier phase locktime counter
uint8 cno                    # dBHz, carrier-to-noise density ratio

uint8 pr_stdev               # pseudorange standard deviation index n: 0.01 * 2^n m
uint8 cp_stdev               # carrier phase standard deviation index n: 0.004 * n cycles
uint8 do_stdev               # Doppler standard deviation index n: 0.002 * 2^n Hz

uint8 trk_stat               # raw tracking status bitfield, as received
bool pr_valid                # trk_stat bit 0: pseudorange valid
bool cp_valid                # trk_stat bit 1: carrier phase valid
bool half_cyc                # trk_stat bit 2: half cycle valid
bool sub_half_cyc            # trk_stat bit 3: half cycle subtracted from the phase
