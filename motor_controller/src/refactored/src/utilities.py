PKG_NAME = "motion_control"

PID_SETTINGS_FILE = "pid_settings"
ENCODER_SETTINGS_FILE = "encoder_settings"

LINE = 0.15  # M/S
TURN = 0.075  # M/S
BASE_WIDTH = 0.13  # m
TICKS_PER_METER = 209  # TICKS_PER_REV / (WHEEL_RADIUS * 2  * PI)
MAX_PWM = 16  # hard max to preserve motors via clamp
SPEED_AT_100_PWM = 3.0  # m/s, used to scale PID errors
DEFAULT_TURNING_RADIUS = 1 / (3.14 * BASE_WIDTH)

def update_PID_msg(msg, pwm, target_vel, encoder_vel, scale=1):
    msg.PWM = pwm
    msg.targetVel = target_vel * scale
    msg.encoderVel = encoder_vel * scale


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


# numpad indexing
def twist_to_index(twist):
    vec_angular, vec_linear = twist.angular, twist.linear
    if vec_angular.z > 0.05:
        turn = 1
    elif vec_angular.z < -0.05:
        turn = -1
    else:
        turn = 0
    if vec_linear.x > 0.05:
        forward = 1
    elif vec_linear.x < -0.05:
        forward = -1
    else:
        forward = 0
    return 5 - turn + 3 * forward


def twist_to_wheel_vel(twist):
    vec_angular, vec_linear = twist.angular, twist.linear
    left_vel = vec_linear.x - 0.5 * vec_angular.z * BASE_WIDTH
    right_vel = vec_linear.x + 0.5 * vec_angular.z * BASE_WIDTH
    return left_vel, right_vel

