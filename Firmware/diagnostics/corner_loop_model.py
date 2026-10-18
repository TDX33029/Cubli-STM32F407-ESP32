"""Withdrawn corner gain model; explanatory command entry point only.

The previous implementation is preserved in Git at f7f7733. Its gain
recommendations must not be used for hardware tuning:

* For p=g-R and gdot=g x omega, gravity torque is proportional to
  g x R = -skew(R) p, not (I-R R') p. The old plant mixed gravity-vector
  coordinates with angular-acceleration coordinates.
* The fitted wheel drag includes back EMF/internal torques. Applying that
  drag only to the wheel while applying only voltage torque to the body
  omits the corresponding internal reaction. External aerodynamic drag
  must be identified separately.
* Choosing wheel-axis signs by the most stable eigenvalues is circular.
  Signs must come from measured voltage/encoder/gyro conventions.
* Dormant integrators and the gravity normalization constraint introduce
  zero modes; they cannot establish asymptotic stability.

A replacement requires consistent inertias, measured signs and high-speed
actuator identification. This review does not substitute another unvalidated
model. Confirmed 80/5/-0.030/0, BYAW=3, BTRIM=0 remains the starting point;
long-term free balance is not yet verified.
"""
import sys


def main():
    print(__doc__)
    return 2  # Deliberately fail automated gain-sweep workflows.


if __name__ == '__main__':
    sys.exit(main())
