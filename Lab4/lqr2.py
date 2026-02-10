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
    k_p = 15
    k_d = 0.237
    u_max = 0.7

    prev_error = 0.0
    prev_t = time.time()

    # -------------------------
    # Outer loop (position hold) PD -> theta_set
    # -------------------------
    outer_k_p = 0.75
    outer_k_d = 0.4
    theta_max = 0.04

    outer_hz = 60.0
    outer_dt_target = 1.0 / outer_hz
    outer_accum = 0.0
    outer_prev_t = time.time()

    xdot_f = 0.0
    xdot_alpha = 0.2

    # threshold to activate outer loop
    pos_deadzone = 0.5  # tune this — encoder units of drift before outer loop kicks in

    def read_wheel_pos():
        l = float(left.position)
        r = float(right.position)
        return l, r

    l0, r0 = read_wheel_pos()
    x_ref = 0.5 * (l0 + r0)

    prev_x = x_ref
    theta_set = 0.0

    time.sleep(1.0)  # give yourself a second to balance it
    l0, r0 = read_wheel_pos()
    x_ref = 0.5 * (l0 + r0)
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
            x = 0.2 * (lpos + rpos)

            now = time.time()
            outer_dt = now - outer_prev_t
            if outer_dt <= 0:
                outer_dt = outer_dt_target
            outer_prev_t = now

            xdot = (x - prev_x) / outer_dt
            prev_x = x

            xdot_f = (1 - xdot_alpha) * xdot_f + xdot_alpha * xdot

            e_x = x_ref - x

            if abs(e_x) > pos_deadzone:
                theta_cmd = -(outer_k_p * e_x - outer_k_d * xdot_f)
                theta_set = clamp(theta_cmd, -theta_max, theta_max)
                print(f"e_x={e_x:.3f}  xdot_f={xdot_f:.3f}  theta_set={theta_set:.4f}")
            else:
                theta_set = 0.0

        # -------------------------
        # Inner loop (balance around theta_set)
        # -------------------------
        theta = float(imu.gravity_vector[1])

        error = theta_set - theta - 0.07
        d_error = (error - prev_error) / dt
        prev_error = error

        u = k_p * error + k_d * d_error
        u = clamp(u, -u_max - 0.1, u_max)
        print(u)

        left.power_command = -u
        right.power_command = u

        time.sleep(0.005)

if __name__ == "__main__":
    main()