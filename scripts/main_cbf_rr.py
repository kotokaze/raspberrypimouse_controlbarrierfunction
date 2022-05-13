#!/usr/bin/env python3
import rospy
import numpy as np
from controller import *
from transform import *
from raspi import *
from random import uniform

#This script used for experimenting with real robots.

N = 4 #no. of robots

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(50)
        # x_goal = np.array([[uniform(-0.8,0.8), uniform(-0.8,0.8)], [uniform(-0.8,0.8), uniform(-0.8,0.8)]]) #(x, y)
        # x_goal = np.array([[uniform(-0.8, 0), 0], [uniform(0, 0.8), 0], [0, uniform(0,0.8)], [0, uniform(0, -0.8)]])
        #x_goal = np.array([[-0.5, 0, 0.5, 0], [0, -0.5, 0, 0.5]])
        # x_goal = np.array([[uniform(-0.6, 0), uniform(0, 0.6), 0, 0], [0, 0, uniform(0, -0.6), uniform(0, 0.6)]])
        
        #x_goal = np.array([[0.5, -0.5], [0, 0]])
        #x_goal = np.array([[0.5, -0.5, 0, 0], [0, 0, -0.5, 0.5]])
        #x_goal = np.array([[0, 0.5, -0.5, 0], [-0.5, 0, 0, 0.5]])
        #x_goal = np.array([[0, -0.5, 0.5, 0], [0.5, 0, 0, -0.5]])
        x_goal = np.array([[0.5, -0.5, 0, 0], [0, 0, -0.5, 0.5]])
        '''
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        x_goal = p_circ[:, :N]
        '''

        num_cycles = 3
        counter = 0

        flag = 0
        # x_goal = np.array([[-0.8,0.8],[0,0]])
        #x_goal = np.array([[-0.6, 0.6, 0], [0, 0, 0.6]])
        # x_goal = np.array([[-0.6, 0.6, 0, 0], [0, 0, -0.6, 0.6]])
        while not rospy.is_shutdown():
            pose = get_robot_position(N)
            print("THE POSE: ", pose)

            print(np.column_stack((pose[0:2])))
            pose_si = uni_to_si_states(pose)
            
            # if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.09):
            #     rate.sleep()

            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                flag = 1-flag
                '''
                for i in range(len(x_goal)):
                    
                    for j in range(len(x_goal[i])):
                        # x_goal = np.array([[-0.8, 0.8], [0, 0]])
                        x_goal = np.array([[-0.5, 0, 0.5, 0], [0, -0.5, 0, 0.5]])
                        # x_goal = np.array([[-0.6, 0.6, 0, 0], [0, 0, -0.6, 0.6]])
                '''
                

            
            if counter == num_cycles:
                break

            if(flag == 0):
                #x_goal = np.array([[0.5, -0.5], [0, 0]])
                #x_goal = np.array([[0.5, -0.5, 0, 0], [0, 0, -0.5, 0.5]])
                #x_goal = np.array([[0, -0.5, 0.5, 0], [0.5, 0, 0, -0.5]])
                x_goal = np.array([[0.5, -0.5, 0, 0], [0, 0, -0.5, 0.5]])
                #print("flag!!!!!!!!!!!!!!!!!: ", flag)
            else:
                #x_goal = np.array([[-0.5, 0.5], [0, 0]])
                #x_goal = np.array([[-0.5, 0.5, 0, 0], [0, 0, 0.5, -0.5]])
                x_goal = np.array([[-0.5, 0.5, 0, 0], [0, 0, 0.5, -0.5]])
                #print("flag22222222222222222: ", flag)

        

            dxu = robotFeedbackControl(pose, x_goal)
            dxi = uni_to_si_dyn(dxu, pose)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
