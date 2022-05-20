#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

#This script used for experimenting with simulation.

'''
start from the beginning (from pose)
put in value like normal CVT
then input coord into bots (basically x and z)(x -> velocity, z -> turning)
'''

N = 4

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False) # initiate node
        # initiate x_goal into a circle
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        flag = 0 
        x_goal = p_circ[:, :N] 

        # The most common usage patterns for testing for shutdown in rospy
        while not rospy.is_shutdown():
            pose = getposition(N) # get position in the gazebo simulation
            print("Pose: ", pose)
            print(" ")
            # So that we can see the x and y coordinate only
            print(np.column_stack((pose[0:2]))) # It convert into [[x1,y1], [x2,y2], [x3,y3], ...]
            pose_si = uni_to_si_states(pose) # transformer

            # Check if all the agents are close enough to the goals
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            # Switch goals depending on the state of flag
            if(flag == 0):
                x_goal = p_circ[:, :N]
            else:
                x_goal = p_circ[:, N:]

            # Use a position controller to drive to the goal position
            dxi = si_position_controller(pose_si, x_goal) # from the original position to the goal position
            # Use the barrier certificates to make sure that the agents don't collide
            dxi = si_barrier_cert(dxi, pose_si) # this what make the cbf function
            dxu = si_to_uni_dyn(dxi, pose) # transformer
            # Set the velocities of agents 1,...,N to dxu
            k = set_velocities(N, dxu) 
            put_velocities(N, k) # put velocity into /cmd_vel

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
