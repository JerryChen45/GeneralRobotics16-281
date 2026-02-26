
def update_odometry(state, prev_left, prev_right, accumulated_angle, left, right):
    curr_left  = left.position
    curr_right = right.position

    delta_left  = LEFT_ENC_SIGN  * (curr_left  - prev_left)
    delta_right = RIGHT_ENC_SIGN * (curr_right - prev_right)

    dist_left  = (delta_left  / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS
    dist_right = (delta_right / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS

    v     = 0.5 * (dist_left + dist_right) / DT
    omega = (dist_right - dist_left) / (WHEELBASE * DT)

    state = rk4_step(state, v, omega, DT)

    delta_theta = (dist_right - dist_left) / WHEELBASE
    accumulated_angle += delta_theta

    return state, curr_left, curr_right, accumulated_angle