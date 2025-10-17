import time
from safety_monitor.config import TOPIC_EMERGENCY_STOP_STATUS
from safety_monitor.config import TOPIC_PROXIMITY_STATUS
TOPIC_EMERGENCY_STOP_STATUS
TOPIC_PROXIMITY_STATUS

VALID_OPERATION_MODES = ["EMERGENCY_STOP", "STOP", "SLOW", "FULL_SPEED"]

class RobotOpStatus(object):
    def __init__(self, mode="EMERGENCY_STOP", proximity=0.0):
        assert mode in VALID_OPERATION_MODES, f"Invalid robot operation mode {mode}, valid modes are {VALID_OPERATION_MODES}"
        self.mode = mode
        self.proximity = proximity

STATUS_VELOCITY_MULTIPLIERS = {
    "EMERGENCY_STOP": 0.0,
    "STOP": 0.0,
    "SLOW" : 0.5,
    "FULL_SPEED" : 1.0
}

HYTERESIS_DISTANCE = 200.0 # in millimeters

UPPER_THRESHOLDS = { # units in millimeters
    "STOP": 400,
    "SLOW": 800,
    "FULL_SPEED": float('inf')
}

STATE_TRANSITION_HYSTERESIS_THRESHOLDS = {
    "STOP": {
        "SLOWER": {},
        "FASTER": {
            "SLOW": UPPER_THRESHOLDS["STOP"] + HYTERESIS_DISTANCE,
            "FULL_SPEED": UPPER_THRESHOLDS["SLOW"] + HYTERESIS_DISTANCE,
        }
    },
    "SLOW": {
        "SLOWER": {
            "STOP": UPPER_THRESHOLDS["STOP"],       # no hysteresis when going to more restrictive state
        },
        "FASTER": {
            "FULL_SPEED": UPPER_THRESHOLDS["SLOW"] + HYTERESIS_DISTANCE
        }
    },
    "FULL_SPEED": {
        "SLOWER": {
            "STOP": UPPER_THRESHOLDS["STOP"],
            "SLOW": UPPER_THRESHOLDS["SLOW"]
        },
        "FASTER": {}
    }
}