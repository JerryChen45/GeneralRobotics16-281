from motorgo import ControlMode
from motorgo.plink import Plink
import time

LEFT_CH = 3
RIGHT_CH = 1

def clamp(v, lo, hi):
    return max(min(v, hi), lo)

def main():
    plink = Plink()
    plink.connect()
    imu = plink.imu

    left = getattr(plink, f"channel{LEFT_CH}")
    right = getattr(plink, f"channel{RIGHT_CH}")

    left.control_mode = ControlMode.POWER
    right.control_mode = ControlMode.POWER

    # -------------------------
    # Inner loop (balance PD)
    # -------------------------
    k_p = 15.0
    k_d = 0.23
    u_max = 0.7

    prev_error = 0.0
    prev_t = time.time()

    # -------------------------
    # Outer loop (position hold) PD -> theta_set
    # -------------------------
    outer_k_p = 0.8
    outer_k_d = 0.4
    theta_max = 0.05  # rad-ish (depends what gravity_vector[1] represents for you)

    outer_hz = 60.0
    outer_dt_target = 1.0 / outer_hz
    outer_accum = 0.0
    outer_prev_t = time.time()

    # simple low-pass for xdot
    xdot_f = 0.0
    xdot_alpha = 0.2  # 0..1 (higher = less filtering)

    def read_wheel_pos():
        l = float(left.position)
        r = float(right.position)
        return l, r

    # Reference position (start point)
    l0, r0 = read_wheel_pos()
    x_ref = 0.5 * (l0 + r0)

    prev_x = x_ref
    theta_set = 0.0

    while True:
        t = time.time()
        dt = t - prev_t
        if dt <= 0:
            continue
        prev_t = t

        # -------------------------
        # Outer loop update
        # -------------------------
        outer_accum += dt
        while outer_accum >= outer_dt_target:
            outer_accum -= outer_dt_target

            lpos, rpos = read_wheel_pos()
            x = 0.5 * (lpos + rpos)

            now = time.time()
            outer_dt = now - outer_prev_t
            if outer_dt <= 0:
                outer_dt = outer_dt_target
            outer_prev_t = now

            xdot = (x - prev_x) / outer_dt
            prev_x = x

            # low-pass filter velocity
            xdot_f = (1 - xdot_alpha) * xdot_f + xdot_alpha * xdot

            e_x = x_ref - x
            theta_cmd = outer_k_p * e_x - outer_k_d * xdot_f
            theta_set = clamp(theta_cmd, -theta_max, theta_max)

        # -------------------------
        # Inner loop (balance around theta_set)
        # -------------------------
        theta = float(imu.gravity_vector[1])

        error = theta_set - theta
        d_error = (error - prev_error) / dt
        prev_error = error

        u = k_p * error + k_d * d_error
        u = clamp(u, -u_max , u_max )
        print(u)

        # motor directions: may need sign flips depending on your wiring/convention
        left.power_command = -u
        right.power_command =  u

        time.sleep(0.005)

if __name__ == "__main__":
    main()
