"""NCOM decoding constants based on OxTS manual."""

NCOM_SYNC = 0xE7
NCOM_PACKET_LENGTH = 72

# Batch A scale factors (Table 4)
ACC2MPS2 = 1e-4
RATE2RPS = 1e-5

# Batch B scale factors (Table 6)
VEL2MPS = 1e-4
ANG2RAD = 1e-6

# Accuracy channels
VEL_ACC_SCALE = 1e-3  # 1 mm/s -> m/s
POS_ACC_SCALE = 1e-3  # 1 mm -> m
ORI_ACC_SCALE = 1e-5  # 0.01 mrad -> rad
ACCURACY_AGE_INVALID = 150
INNOVATION_SCALE = 0.1  # 0.1 sigma

# Navigation status (Table 5)
NAV_STATUS = {
    0: "Invalid",
    1: "Raw IMU",
    2: "Initialising",
    3: "Locking",
    4: "Real-Time",
    5: "Unlocked",
    6: "Firmware Expired",
    7: "Firmware Blocked",
    10: "Status Only",
    11: "Internal (Structure-B)",
    20: "Trigger (Init)",
    21: "Trigger (Locking)",
    22: "Trigger (Real-Time)",
}

# GNSS mode mapping (Table 9 / official NCOMdecoder names)
GNSS_MODE = {
    0: "None",
    1: "Search",
    2: "Doppler",
    3: "SPS",
    4: "Differential",
    5: "RTK Float",
    6: "RTK Integer",
    7: "WAAS",
    8: "OmniSTAR",
    9: "OmniSTAR HP",
    10: "No data",
    11: "Blanked",
    12: "Doppler (PP)",
    13: "SPS (PP)",
    14: "Differential (PP)",
    15: "RTK Float (PP)",
    16: "RTK Integer (PP)",
    17: "OmniSTAR XP",
    18: "CDGPS",
    19: "Not Recognised",
    20: "gxDoppler",
    21: "gxSPS",
    22: "gxDifferential",
    23: "gxFloat",
    24: "gxInteger",
    25: "ixDoppler",
    26: "ixSPS",
    27: "ixDifferential",
    28: "ixFloat",
    29: "ixInteger",
    30: "PPP converging",
    31: "PPP",
}
