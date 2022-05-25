#!/usr/bin/env python3
from pandas import array
import rospy
import numpy as np
from controller import *
from transform import *
from raspi import *
from random import uniform

# This script used for experimenting with real robots.

# FIXME: Magic number 6 will not work
N = 8  # no. of robots

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=False)
        rate = rospy.Rate(50)
        num_cycles = 2
        counter = 0
        flag = 0

        # x_goal = np.array([[uniform(-0.8,0.8), uniform(-0.8,0.8)], [uniform(-0.8,0.8), uniform(-0.8,0.8)]]) #(x, y)
        # x_goal = np.array([[uniform(0.6,0.7), uniform(-0.6,-0.7)], [uniform(0.1,-0.1), uniform(0.1,-0.1)]]) # two bots
        # x_goal = np.array([[0.7, 0, -0.7], [0.5, -0.5, 0.5]]) # three bots
        # x_goal = np.array([[uniform(0.6,0.7), uniform(-0.1,0.1), uniform(-0.6,-0.7)], [uniform(0.5,0.6), uniform(-0.5,-0.6), uniform(0.5,0.6)]]) # three bots
        # x_goal = np.array([[0.6, 0, -0.6], [0.5, -0.5, 0.5]]) # three bots / triangle
        # x_goal = np.array([[0.6, 0, -0.6], [0.5, 0.5, 0.5]]) # three bots / bottom line
        # x_goal = np.array([[0.85, -0.85, 0, 0], [0, 0, -0.65, 0.65]])  # four bots
        # x_goal = np.array([[0, -0.85, 0.85, -0.85, 0.85], [0.65, 0, 0, -0.65, -0.65]])  # five bots
        # x_goal = np.array([[0.85, 0.85, -0.85, -0.85, 0, 0], [0.2, -0.2, 0.2, -0.2, -0.65, 0.65]])  # six bots
        x_goal = np.array([[0.85, 0.85, 0.2, -0.2, -0.85, -0.85, -0.2, 0.2], [0.2, -0.2, -0.65, -0.65, -0.2, 0.2, 0.65, 0.65]])  # eight bots

        while not rospy.is_shutdown():
            pose = get_robot_position(N)

            print("pose: ", pose)
            pose_si = uni_to_si_states(pose)

            # if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.04):
            #     flag = 1 - flag
            # rate.sleep()
            # for i in range(len(x_goal)):
            #     for j in range(len(x_goal[i])):
            #         x_goal = np.array([[uniform(-0.8,0.8), uniform(-0.8,0.8)], [uniform(-0.8,0.8), uniform(-0.8,0.8)]])

            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1 - flag
                counter += 1

            if counter == num_cycles:
                break

            # Switch goals depending on the state of flag
            if(flag == 0):
                # x_goal = np.array([[0.7, -0.7], [0, 0]]) # two bots
                # x_goal = np.array([[uniform(0.6,0.7), uniform(-0.6,-0.7)], [uniform(0.1,-0.1), uniform(0.1,-0.1)]]) # two bots
                # x_goal = np.array([[uniform(0.6,0.7), uniform(-0.1,0.1), uniform(-0.6,-0.7)], [uniform(0.50,0.55), uniform(-0.5,-0.6), uniform(0.5,0.6)]]) # three bots
                # x_goal = np.array([[0.6, 0, -0.6], [0.5, -0.5, 0.5]]) # three bots  / triangle
                # x_goal = np.array([[0.6, 0, -0.6], [0.5, 0.5, 0.5]]) # three bots / bottom line
                # x_goal = np.array([[0.85, -0.85, 0, 0], [0, 0, -0.65, 0.65]])  # four bots
                # x_goal = np.array([[0, -0.85, 0.85, -0.85, 0.85], [0.65, 0, 0, -0.65, -0.65]])  # five bots
                # x_goal = np,array([[0.85, 0.85, -0.85, -0.85, 0, 0], [0.2, -0.2, 0.2, -0.2, -0.65, 0.65]])  # six bots
                x_goal = np.array([[0.85, 0.85, 0.2, -0.2, -0.85, -0.85, -0.2, 0.2], [0.2, -0.2, -0.65, -0.65, -0.2, 0.2, 0.65, 0.65]])  # eight bots
            else:
                # x_goal = np.array([[-0.7, 0.7], [0, 0]]) # two bots
                # x_goal = np.array([[uniform(-0.6,-0.7), uniform(0.6,0.7)], [uniform(0.1,-0.1), uniform(0.1,-0.1)]]) # two bots
                # x_goal = np.array([[uniform(-0.6,-0.7), uniform(-0.1,0.1), uniform(0.6,0.7)], [uniform(-0.50,-0.55), uniform(0.5,0.6), uniform(-0.5,-0.6)]]) # three bots
                # x_goal = np.array([[-0.6, 0, 0.6], [-0.5, 0.5, -0.5]]) # three bots  / triangle
                # x_goal = np.array([[-0.6, 0, 0.6], [-0.5, -0.5, -0.5]]) # three bots / bottom line
                # x_goal = np.array([[-0.85, 0.85, 0, 0], [0, 0, 0.65, -0.65]])  # four bots
                # x_goal = np.array([[0.85, 0.85, -0.85, 0, -0.85], [-0.65, 0, -0.65, 0.65, 0]])  # five bots
                # x_goal = np,array([[-0.85, -0.85, 0.85, 0.85, 0, 0], [0.2, -0.2, 0.2, -0.2, 0.65, -0.65]])  # six bots
                x_goal = np.array([[-0.85, -0.85, 0.2, -0.2, 0.85, 0.85, -0.2, 0.2], [0.2, -0.2, 0.65, 0.65, -0.2, 0.2, -0.65, -0.65]])  # eight bots

            dxu = robotFeedbackControl(pose, x_goal)
            dxi = uni_to_si_dyn(dxu, pose)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
