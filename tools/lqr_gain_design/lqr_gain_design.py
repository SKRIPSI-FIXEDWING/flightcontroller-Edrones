#!/usr/bin/env python3
"""Offline LQR gain design for AttitudeController (roll/pitch/yaw).

Computes ONE fixed K per axis at cruise trim (18 m/s), then validates
stability/damping across the full 14-22 m/s airspeed envelope with that
SAME K (see AttitudeController's speed-scaler post-multiplier, which
compensates for the envelope instead of gain-scheduling K itself).

Run: (from this directory, using the project-local venv)
    .venv/Scripts/python.exe lqr_gain_design.py

Requires: numpy, scipy, control (installed into tools/lqr_gain_design/.venv,
NOT the system Python -- see docs/attitude-lqr.md).

All aircraft stability/control derivatives below are GENERIC ESTIMATES for a
2.8 kg / 1.8 m wingspan / 0.45 m^2 conventional trainer-type airframe, sized
from target max rates at max surface deflection (not measured from this
specific airframe). Refine from aileron/elevator/rudder doublet flight-test
system ID before trusting in flight -- see docs/attitude-lqr.md's
"Assumptions to verify before flight" list.
"""

import numpy as np
import control

# ---------------------------------------------------------------------------
# Aircraft constants (documentation only, not directly consumed below --
# they're what the derivative estimates were derived from).
# ---------------------------------------------------------------------------
MASS_KG = 2.8
WINGSPAN_M = 1.8
WING_AREA_M2 = 0.45
CHORD_M = WING_AREA_M2 / WINGSPAN_M  # ~0.25 m
AIR_DENSITY_KG_M3 = 1.225
CRUISE_SPEED_MPS = 18.0
MIN_AIRSPEED_MPS = 14.0
MAX_AIRSPEED_MPS = 22.0

CONTROL_LOOP_HZ = 200.0
DT = 1.0 / CONTROL_LOOP_HZ

DEG = np.pi / 180.0


# ---------------------------------------------------------------------------
# Per-axis continuous-time state-space models at cruise trim.
#
# Roll:  x = [phi, p, integral(phi_err)],      u = aileron (rad)
# Pitch: x = [theta, q, integral(theta_err)],  u = elevator (rad)
# Yaw:   x = [r],                              u = rudder (rad)   (rate-only, no integral)
# ---------------------------------------------------------------------------

def augmented_2state(a_rate_damping, b_effectiveness):
    """Builds the un-augmented [angle, rate] A/B pair for one axis."""
    a = np.array([[0.0, 1.0],
                  [0.0, a_rate_damping]])
    b = np.array([[0.0],
                  [b_effectiveness]])
    return a, b


def augment_with_integrator(a, b):
    """Servomechanism/type-1 augmentation: adds integral(angle_error) as a
    third state. C selects the angle (first state)."""
    n = a.shape[0]
    c = np.zeros((1, n))
    c[0, 0] = 1.0
    a_aug = np.zeros((n + 1, n + 1))
    a_aug[:n, :n] = a
    a_aug[n, :n] = -c
    b_aug = np.zeros((n + 1, 1))
    b_aug[:n, :] = b
    return a_aug, b_aug


# Representative dimensional derivatives at cruise (18 m/s) -- see
# docs/attitude-lqr.md for the formulas/reasoning behind these numbers.
L_P = -8.0        # roll damping, 1/s
L_DELTA_A = 40.0  # roll control effectiveness, 1/s^2 per rad

M_Q = -7.0         # pitch damping, 1/s
M_DELTA_E = 17.0   # pitch control effectiveness, 1/s^2 per rad

N_R = -1.5         # yaw damping, 1/s
N_DELTA_R = 1.8    # yaw control effectiveness, 1/s^2 per rad

A_ROLL, B_ROLL = augmented_2state(L_P, L_DELTA_A)
A_ROLL, B_ROLL = augment_with_integrator(A_ROLL, B_ROLL)

A_PITCH, B_PITCH = augmented_2state(M_Q, M_DELTA_E)
A_PITCH, B_PITCH = augment_with_integrator(A_PITCH, B_PITCH)

# Yaw: rate-only, no integrator (see docs/attitude-lqr.md -- washout via
# rate damping + kinematic feedforward is the intended design, not integral
# action on a sustained nonzero coordinated-turn rate command).
A_YAW = np.array([[N_R]])
B_YAW = np.array([[N_DELTA_R]])


# ---------------------------------------------------------------------------
# Bryson's rule: Q_ii = 1 / (max acceptable deviation of x_i)^2,
#                R_jj = 1 / (max acceptable deviation of u_j)^2
# ---------------------------------------------------------------------------

def bryson_q(max_devs):
    return np.diag([1.0 / (d ** 2) for d in max_devs])


def bryson_r(max_u):
    return np.array([[1.0 / (max_u ** 2)]])


MAX_SURFACE_DEFLECTION_RAD = 25.0 * DEG  # matches the firmware's ~25 deg output_limit_deg

Q_ROLL = bryson_q([
    7.5 * DEG,     # max roll angle error
    60.0 * DEG,    # max transient roll rate
    0.3,           # integral bound: null a 7.5 deg trim error over ~2s (0.3 rad*s)
])
R_ROLL = bryson_r(MAX_SURFACE_DEFLECTION_RAD)

Q_PITCH = bryson_q([
    5.0 * DEG,     # max pitch angle error
    40.0 * DEG,    # max transient pitch rate
    0.26,          # integral bound, slower than roll
])
R_PITCH = bryson_r(MAX_SURFACE_DEFLECTION_RAD)

Q_YAW = bryson_q([
    15.0 * DEG,    # max yaw-rate deviation from the coordinated-turn command
])
R_YAW = bryson_r(MAX_SURFACE_DEFLECTION_RAD)


def solve_discrete_lqr(a, b, q, r, dt):
    sys_c = control.ss(a, b, np.eye(a.shape[0]), 0)
    sys_d = control.c2d(sys_c, dt, method="zoh")
    k, _, e = control.dlqr(sys_d.A, sys_d.B, q, r)
    return np.asarray(k), sys_d, e


def closed_loop_eigs_at_speed(a_cruise, b_cruise, k, speed_ratio_sq, dt):
    """Re-derives B at a different airspeed (control effectiveness scales
    with dynamic pressure, i.e. V^2) and checks eigenvalues of (Ad - Bd*K)
    with the SAME cruise-designed K -- this is the dual-airspeed check
    described in docs/attitude-lqr.md."""
    b_scaled = b_cruise * speed_ratio_sq
    sys_c = control.ss(a_cruise, b_scaled, np.eye(a_cruise.shape[0]), 0)
    sys_d = control.c2d(sys_c, dt, method="zoh")
    a_cl = sys_d.A - sys_d.B @ k
    return np.linalg.eigvals(a_cl)


def report_axis(name, a, b, q, r, has_integrator):
    k, sys_d, disc_eigs = solve_discrete_lqr(a, b, q, r, DT)
    print(f"\n=== {name} axis ===")
    print("K =", np.array2string(k, precision=4, suppress_small=True))
    print("Closed-loop discrete poles @ cruise:",
          np.array2string(disc_eigs, precision=4))

    for label, speed in (("min (14 m/s)", MIN_AIRSPEED_MPS), ("max (22 m/s)", MAX_AIRSPEED_MPS)):
        ratio_sq = (CRUISE_SPEED_MPS / speed) ** 2  # B ~ dynamic pressure ~ V^2 relative to cruise
        eigs = closed_loop_eigs_at_speed(a, b, k, 1.0 / ratio_sq, DT)
        max_mag = np.max(np.abs(eigs))
        stable = max_mag < 1.0
        print(f"  @ {label}: max |eig| = {max_mag:.4f} ({'STABLE' if stable else 'UNSTABLE -- retune Q/R!'})")

    return k


def main():
    print(f"Discretization: zero-order hold at {CONTROL_LOOP_HZ:.0f} Hz (dt={DT*1e3:.2f} ms)")

    k_roll = report_axis("Roll", A_ROLL, B_ROLL, Q_ROLL, R_ROLL, has_integrator=True)
    k_pitch = report_axis("Pitch", A_PITCH, B_PITCH, Q_PITCH, R_PITCH, has_integrator=True)
    k_yaw = report_axis("Yaw", A_YAW, B_YAW, Q_YAW, R_YAW, has_integrator=False)

    print("\n=== Firmware LqrAxisConfig values (degrees-based, matching AttitudeController) ===")
    # K is solved in the radian domain (state and control input both in
    # radians), but the firmware's LqrAxisController operates entirely in
    # degrees/deg-per-second. Because u = -K*x is linear and BOTH x and u
    # are rescaled by the same deg<->rad factor, that factor cancels exactly
    # -- K's numeric values are IDENTICAL whether the loop runs in radians
    # or degrees. No unit conversion factor is applied below.
    #
    # SIGN NOTE (see docs/attitude-lqr.md for the full derivation): the
    # augmented integrator state here is defined as xi_dot = -x1 (x1 = the
    # angle state), which is algebraically the same state the firmware
    # accumulates as integral_state_ = integral(setpoint - measured) dt
    # (assuming a quasi-static setpoint). The control law is u = -K*x_aug
    # for ALL three terms, i.e. -(K1*x1 + K2*rate + K3*xi). Substituting
    # x1 = (measured - setpoint) and rewriting -K1*x1 = K1*(setpoint -
    # measured) shows k_primary=K1 and k_rate=K2 carry over UNCHANGED into
    # the firmware's "+k_primary*error - k_rate*rate" form, but the
    # integral term needs an explicit sign flip: firmware computes
    # "+k_integral*integral_state", so k_integral = -K3.
    print("Roll:  k_primary(deg)=%.6f  k_rate(deg/s)=%.6f  k_integral=%.6f" %
          (k_roll[0, 0], k_roll[0, 1], -k_roll[0, 2]))
    print("Pitch: k_primary(deg)=%.6f  k_rate(deg/s)=%.6f  k_integral=%.6f" %
          (k_pitch[0, 0], k_pitch[0, 1], -k_pitch[0, 2]))
    print("Yaw:   k_primary(deg/s)=%.6f" % (k_yaw[0, 0]))
    print("\n(Paste the values above directly into AttitudeControllerConfig's")
    print(" roll/pitch/yaw LqrAxisConfig fields -- k_integral's sign is already flipped.)")


if __name__ == "__main__":
    main()
