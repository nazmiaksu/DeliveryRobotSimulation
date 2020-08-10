"""Webots controller for delivering food to a customer.

Running this script requires coordinates (x, y) of the customer to deliver
to as arguments.
"""
import sys

from controller import Robot
from utils import Vector, normalise_angle

robot = Robot()

# set max speed, units and simulation constants
TIME_STEP = 64
SENSOR_RANGE = 512
MAX_SPEED = 10.0
SPEED_UNIT = 0.1
WHEEL_RADIUS = 0.0825
WHEEL_BASE = 0.1603 * 2  # from translation of robot wheel - distance between wheels
TURNING_COEFFICIENT = 5  # how fast to turn in relation to how far away target is
TARGET_THRESHOLD = 0.5  # distance away from target at which to stop

# used to calculate how much to turn to avoid things based on the 16 distance sensors
BRAITENBERG_COEFFICIENTS = [
    [-1, 15],
    [-3, 13],
    [-3, 8],
    [-2, 7],
    [-3, -4],
    [-4, -2],
    [-3, -2],
    [-1, -1],
    [-1, -1],
    [-2, -3],
    [-2, -4],
    [4, -3],
    [7, -5],
    [7, -3],
    [10, -2],
    [11, -1],
]

# IR distance sensors used to avoid obstacles
DISTANCE_SENSORS = []
for i in range(16):
    sensor = robot.getDistanceSensor(f"ds{i}")
    sensor.enable(TIME_STEP)
    DISTANCE_SENSORS.append(sensor)

# motors on each wheel for movement
MOTORS = []
for side in ("left", "right"):
    motor = robot.getMotor(f"{side} wheel motor")
    motor.setPosition(float("INF"))
    motor.setVelocity(0.0)
    MOTORS.append(motor)

# GPS used to track current position
GPS = robot.getGPS("gps")
GPS.enable(TIME_STEP)

# compass used to track current angle from the x-axis
COMPASS = robot.getCompass("compass")
COMPASS.enable(TIME_STEP)


def polar_to_differential(v, omega):
    """Calculate the speed of each wheel from a polar velocity (v, omega)."""
    left_speed = (10 * v - omega * WHEEL_BASE) / 2 * WHEEL_RADIUS
    right_speed = (10 * v + omega * WHEEL_BASE) / 2 * WHEEL_RADIUS
    return left_speed, right_speed


def bind_max_speed(speed):
    """Ensure the MAX_SPEED is not exceeded."""
    if speed < -MAX_SPEED:
        return -MAX_SPEED
    elif speed > MAX_SPEED:
        return MAX_SPEED
    else:
        return speed


def set_motor_speeds(left, right):
    """Set the speed of the left and right motors of the robot."""
    MOTORS[0].setVelocity(bind_max_speed(left))
    MOTORS[1].setVelocity(bind_max_speed(right))


def get_destination():
    """Return the robot's destination parsed from command line arguments.

    Raises SystemExit if there aren't 2 arguments that can be parsed as floats.
    """
    try:
        x, y = map(float, sys.argv[1:3])
    except ValueError:
        raise SystemExit("Destination: x, y (floats) must be passed as controllerArgs.")
    return Vector(x, y)


def get_position():
    """Return the current GPS values as a Vector."""
    x, y, z = GPS.getValues()  # y is not used in this controller
    return Vector(x, z)


def get_angle():
    """Return the current angle of the robot counter-clockwise from the x-axis.

    It is assumed the compass faces the x-axis.
    """
    x, y, z = COMPASS.getValues()
    return Vector(x, z).angle_from_x()


def get_distance_sensor_values():
    """Return the current distance sensor values anti-clockwise starting from front left."""
    return [sensor.getValue() for sensor in DISTANCE_SENSORS]


# --------------------------------- MAIN --------------------------------------
robot.step(TIME_STEP)  # populate initial sensor values
START_POINT = get_position()
DESTINATION = get_destination()
print("Starting coordinates:", START_POINT)
print("Destination coordinates:", DESTINATION)

while robot.step(TIME_STEP) != -1:
    robot_position = get_position()  # robot position (x, y)
    distance_to_target = DESTINATION.subtract(robot_position).mag()
    print(f"Distance to target: {distance_to_target}.")

    if distance_to_target < TARGET_THRESHOLD:
        # stop the robot when it reaches its target
        set_motor_speeds(0, 0)
        print("Arrived at the destination.")
        break

    sensor_values = get_distance_sensor_values()  # used for obstacle avoidance

    if all(value == 0.0 for value in sensor_values):  # nothing in the way, go to target
        theta = get_angle()  # angle of robot from x-axis given by compass
        target_vector = DESTINATION.subtract(robot_position)  # from robot to goal
        # angle robot must turn by in order to face the target
        target_angle = normalise_angle(target_vector.angle() - theta)
        # multiply by a constant to produce a speed
        angular_velocity = TURNING_COEFFICIENT * target_angle
        # movement speed falls off as angular_velocity increases
        translational_velocity = MAX_SPEED / (abs(angular_velocity) + 1) ** 0.5
        motor_speeds = polar_to_differential(translational_velocity, angular_velocity)

    else:  # something in the way, try to avoid
        motor_speeds = [0, 0]
        for i in range(len(motor_speeds)):
            for j, sensor_value in enumerate(sensor_values):
                motor_speeds[i] += (
                    SPEED_UNIT
                    * BRAITENBERG_COEFFICIENTS[j][i]
                    * (1 - sensor_value / SENSOR_RANGE)
                )

    set_motor_speeds(*motor_speeds)
