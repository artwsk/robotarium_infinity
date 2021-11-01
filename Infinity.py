import math
import numpy
import time
import rps.robotarium as robotarium
from rps.utilities.barrier_certificates import *
from rps.utilities.controllers import *

"""
Name: Infinity.py
Author: Arthur WSK
Description:
This code is part of a assignment with Robotarium

"""


# based on /  code found in the "si_go_to_point.py" file in the robotarium/robotarium_python_simulator github example
# library (https://github.com/robotarium/robotarium_python_simulator)
# The Function takes in the current pos and the target pos and calculate a speed and angele for the robot in the
# next step
def move_func(current_pos, goal_pos):
    single_integrator_position_controller = create_si_position_controller()

    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

    # Create mapping from single integrator velocity commands to unicycle velocity commands
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

    # Create single-integrator control inputs
    dxi = single_integrator_position_controller(current_pos, goal_pos)

    # Create safe control inputs (i.e., no collisions)
    dxi = si_barrier_cert(dxi, current_pos)

    # Transform single integrator velocity commands to unicycle
    robot.set_velocities(numpy.arange(N), si_to_uni_dyn(dxi, robot_pos_uni))


# based on code found in the "Drive a robot in circles" example on the Robotarium tutorial homepage
# (https://www.robotarium.gatech.edu/tutorial)
# The function generates 360 points the robot can follow a circle, with a given radius'
# it can go from a give degree to another and both CW and CCW
def circle_pos_func(radius, CW, active_path, start_angle, stop_angle, center):
    num_waypoints = 360
    vec = numpy.linspace(0, 2 * math.pi, num_waypoints)

    path = []

    for i in vec:
        pos1 = radius * math.cos(i)
        pos2 = radius * math.sin(i)

        pos1 += center[0]
        pos2 += center[1]

        path.append(numpy.array(numpy.mat(f'{pos1}; {pos2}')))

    if CW:
        path = list(reversed(path))

    for n in range(abs(stop_angle - start_angle)):
        if (n + start_angle) >= 360:
            n = n - 360
        active_path.append(path[n + start_angle])

    return active_path


# Instantiate Robotarium object
N = 1
robot = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)

_, uni_to_si_states = create_si_to_uni_mapping()

error_margin = 0.1

robot_pos_uni = robot.get_poses()
robot.step()


# programming the path of the robot
waypoints = [numpy.array(numpy.mat('0; 0')), numpy.array(numpy.mat('-0.4; -0.4'))]
waypoints = circle_pos_func(0.4, True, waypoints, 90, 270, (-1, 0))
waypoints.append(numpy.array(numpy.mat('-0.4; 0.4')))
waypoints.append(numpy.array(numpy.mat('0.4; -0.4')))
waypoints = circle_pos_func(0.4, False, waypoints, 270, 90, (1, 0))
waypoints.append(numpy.array(numpy.mat('0.4; 0.4')))
waypoints.append(numpy.array(numpy.mat('0; 0')))


index = 0
target_pos = waypoints[index]


while index <= len(waypoints):
    robot_pos_uni = robot.get_poses()
    robot_pos_si = uni_to_si_states(robot_pos_uni)

    if numpy.linalg.norm(robot_pos_si - target_pos) <= error_margin:
        if index + 1 <= len(waypoints):
            target_pos = waypoints[index]
        index += 1

    move_func(robot_pos_si, target_pos)
    robot.step()

time.sleep(3)
robot.call_at_scripts_end()
