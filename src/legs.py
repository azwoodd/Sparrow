"""Sagittal leg kinematics of the V7E leg (hip, knee, ankle all rotate about local X).
Coordinates are (y, z) in the torso frame relative to the hip joint; forward is -y."""
import math
import numpy as np

A = np.array([-0.035, -0.06])     # hip -> knee at q=0
B = np.array([0.06, -0.08])       # knee -> ankle at q=0
LA, LB = float(np.linalg.norm(A)), float(np.linalg.norm(B))
PA, PB = math.atan2(A[1], A[0]), math.atan2(B[1], B[0])
G0 = (PB - PA) % (2 * math.pi)    # inter-link angle at q2 = 0
NEUTRAL = A + B                   # ankle position at q = 0  -> (0.025, -0.14)
Q1_RANGE = (-1.0472, 1.0472); Q2_RANGE = (-1.0472, 1.13446); Q3_RANGE = (-1.22173, 1.22173)


def fk(q1, q2):
    a1, a2 = PA + q1, PB + q1 + q2
    return np.array([LA * math.cos(a1) + LB * math.cos(a2), LA * math.sin(a1) + LB * math.sin(a2)])


def ik(y, z):
    """Hip and knee angles placing the ankle at (y, z). Clamps to the reachable annulus and joint ranges."""
    D = math.hypot(y, z)
    D = min(max(D, 0.075), LA + LB - 1e-4)
    c = (D * D - LA * LA - LB * LB) / (2 * LA * LB)
    g = math.acos(max(-1.0, min(1.0, c)))
    q2 = g - G0
    q2 = min(max(q2, Q2_RANGE[0]), Q2_RANGE[1]); g = q2 + G0
    q1 = math.atan2(z, y) - math.atan2(LB * math.sin(g), LA + LB * math.cos(g)) - PA
    q1 = (q1 + math.pi) % (2 * math.pi) - math.pi
    q1 = min(max(q1, Q1_RANGE[0]), Q1_RANGE[1])
    return q1, q2


def leg_targets(y, z, foot_pitch=0.0):
    """Joint targets (hip, knee, ankle) with the foot sole kept parallel to the torso's XY plane (+foot_pitch)."""
    q1, q2 = ik(y, z)
    return np.array([q1, q2, -(q1 + q2) + foot_pitch])


if __name__ == '__main__':
    print('LA', LA, 'LB', LB, 'neutral', NEUTRAL, 'G0 deg', math.degrees(G0))
    for p in ((0.025, -0.14), (0.02, -0.12), (0.0, -0.16), (0.03, -0.10)):
        q = ik(*p); print(p, np.round(q, 4), np.round(fk(*q), 4))
