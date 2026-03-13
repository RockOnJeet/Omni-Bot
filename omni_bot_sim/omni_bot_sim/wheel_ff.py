import math

# ===================== Platform Configuration =====================

PWM_MAX = 255.0

# 4S LiPo
LIPO_4S_V_NOMINAL = 14.8
LIPO_4S_V_FULL = 16.8
LIPO_4S_V_MIN = 13.2   # absolute min under load

# Nominal no-load current near full speed (A)
MOTOR_NOLOAD_CURRENT_A = 2.0

# ===================== Safety / Limits =====================

# Angular velocity limits (rad/s)
OMEGA_MAX = 28.0      # clamp commands above this
OMEGA_MIN = -28.0

# Battery protection
LOW_VOLTAGE_CUTOFF = 13.5   # begin derating below this
CRITICAL_VOLTAGE = 13.0   # hard clamp / near stop

# Current protection (A)
OVERCURRENT_WARN = 8.0    # begin derating above this
OVERCURRENT_CRITICAL = 12.0   # aggressive clamp

# ===================== Feedforward Tuning =====================

WHEEL_OMEGA_KNEE = 6.0  # Speed below which low-speed nonlinearity is significant (rad/s)
WHEEL_K_LOWSPEED = 1.5 # Feedforward scaling factor to compensate for low-speed nonlinearity
WHEEL_LOAD_GAIN = 0.08  # Additional feedforward gain per amp of load current above no-load

# ===================== Per-Wheel Calibration =====================

WHEEL_DB = {1: 25, 2: 15, 3: 20}

WHEEL_K = {
    1: 0.09526422784420614,
    2: 0.1239552112211946,
    3: 0.0988994293812621,
}

WHEEL_S = {
    1: 1.0,
    2: 0.8685374975821694,
    3: 1.0,
}

# ===================== Runtime Inputs =====================

_batt_voltage = 14.8
_batt_current = 0.0


def set_battery_voltage(v: float):
    global _batt_voltage
    _batt_voltage = float(v)


def set_battery_current(i: float):
    global _batt_current
    _batt_current = float(i)

# ===================== Helpers =====================


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sign(x):
    return (x > 0.0) - (x < 0.0)

# ===================== Public API =====================

def omega_to_pwm(wheel_id, omega):
    # Checks
    if wheel_id not in WHEEL_DB:
        raise ValueError(f'Invalid wheel ID: {wheel_id}')
    omega = clamp(omega, OMEGA_MIN, OMEGA_MAX)

    # Implement scaling to slowest wheel
    omega = WHEEL_S[wheel_id] * omega

    # Linear scaling to PWM
    pwm = omega / OMEGA_MAX * PWM_MAX

    # Deadband compensation (PWM units)
    if abs(pwm) < WHEEL_DB[wheel_id] and abs(omega) > 0.0:
        pwm = WHEEL_DB[wheel_id]
    
    # Clamp to max PWM
    pwm = clamp(pwm, -PWM_MAX, PWM_MAX)

    return int(pwm)

