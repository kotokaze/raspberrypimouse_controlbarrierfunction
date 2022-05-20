#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

#This script used for experimenting with simulation.

N = 4

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        '''
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        flag = 0
        x_goal = p_circ[:, :N] 
        print("x_goal: ", x_goal)
        '''
        flag = 0

        # Original position of the bots: [[-1, 0, 1, 0], [0, -1, 0, 1]]
        # raspi0 and raspi2 will exchange postion, while raspi1 and raspi4
        x_goal = [[1, 0, -1, 0], [0, 1, 0, -1]]
        '''
        bot1 will go to [0,1], bot2 go to [-1,0], bot3 go to [0,-1], bot4 go to [1,0]
        '''
        while not rospy.is_shutdown():
            pose = getposition(N)
            print (np.column_stack((pose[0:2])))
            pose_si = uni_to_si_states(pose)

            # Check if all the agents are close enough to the goals
            # Without this line of code, it will constantly try to find the EXACT position
            # With this line of code, it will check if bots are close enough to the goals by 0.05
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            if(flag == 0):
                #x_goal = p_circ[:, :N]
                x_goal = [[1, 0, -1, 0], [0, 1, 0, -1]]
            else:
                #x_goal = p_circ[:, N:]
                # the bot will go back to original position
                x_goal = [[-1, 0, 1, 0], [0, -1, 0, 1]]


            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            print("dxu: ", dxu)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
