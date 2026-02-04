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

WHEEL_OMEGA_KNEE = 6.0
WHEEL_LOWSPEED_GAIN = 0.85
WHEEL_LOAD_GAIN = 0.08

# ===================== Per-Wheel Calibration =====================

WHEEL_DB = {1: 25, 2: 15, 3: 20}

WHEEL_K = {
    1: 0.09526422784420614,
    2: 0.1239552112211946,
    3: 0.0988994293812621,
}

WHEEL_S = {
    1: 1.0,
    2: 0.7685374975821694,
    3: 0.9632434528712791,
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


def wheel_pwm_from_omega(wheel_id: int, omega_des: float) -> int:
    """
    Compute PWM in [-255, 255] with feedforward + safety derating.
    """

    if wheel_id not in (1, 2, 3):
        return 0

    # Enforce angular velocity limits
    omega_cmd = clamp(omega_des, OMEGA_MIN, OMEGA_MAX)

    k_norm = WHEEL_K[wheel_id] * WHEEL_S[wheel_id]
    deadband = float(WHEEL_DB[wheel_id])

    s = sign(omega_cmd)
    absw = abs(omega_cmd)

    # Piecewise slope
    k_eff = k_norm
    if absw < WHEEL_OMEGA_KNEE:
        k_eff *= WHEEL_LOWSPEED_GAIN

    # Inverse feedforward
    pwm = 0.0
    if absw > 1e-4:
        pwm = absw / k_eff

    # Deadband + breakaway
    pwm += deadband

    # Battery voltage compensation + derating
    v = clamp(_batt_voltage, LIPO_4S_V_MIN, LIPO_4S_V_FULL)
    volt_mult = LIPO_4S_V_NOMINAL / v

    # Voltage safety derate
    if _batt_voltage < LOW_VOLTAGE_CUTOFF:
        # Linear derate down to CRITICAL_VOLTAGE
        derate_v = clamp(
            (_batt_voltage - CRITICAL_VOLTAGE) /
            (LOW_VOLTAGE_CUTOFF - CRITICAL_VOLTAGE),
            0.0, 1.0
        )
        volt_mult *= derate_v

    pwm *= volt_mult

    # Load compensation (from measured current)
    i_norm = _batt_current / MOTOR_NOLOAD_CURRENT_A
    load_mult = 1.0 + WHEEL_LOAD_GAIN * clamp(i_norm, 0.0, 3.0)

    # Current safety derate
    if _batt_current > OVERCURRENT_WARN:
        derate_i = clamp(
            (OVERCURRENT_CRITICAL - _batt_current) /
            (OVERCURRENT_CRITICAL - OVERCURRENT_WARN),
            0.0, 1.0
        )
        load_mult *= derate_i

    pwm *= load_mult

    pwm *= s

    # Final clamp
    pwm = clamp(pwm, -PWM_MAX, PWM_MAX)

    return int(round(pwm))
