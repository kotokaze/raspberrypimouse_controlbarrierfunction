#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *
import matplotlib.pyplot as plt
from voronoi_gen import *

from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

#This script used for experimenting with simulation. (CVT)

'''
start from the beginning (from pose)
put in value like normal CVT
then input coord into bots (basically x and z)(x -> velocity, z -> turning)

.....

diff pose
use the pose to input coord
put coord into velocities
the general idea
'''

N = 4

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        #x_traj = np.empty((0, N), float) # csv
        #y_traj = np.empty((0, N), float) # csv
        pose = getposition(N)
        coords = np.column_stack((pose[0:2]))

        poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates= 0)
        poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])

        #plotting
        fig = plt.figure(figsize=(6,5)) # the size of the screen
        fig.subplots_adjust(wspace=0.3, hspace=0, top=0.9, bottom=0.2)
        ax = plt.subplot()
        major_locator=MultipleLocator(1.5)
        ax.xaxis.set_major_locator(major_locator)
        ax.yaxis.set_major_locator(major_locator)
        font = {'size':20}
        ax.set_xlabel('x(m)', font, labelpad=10)
        ax.set_ylabel('y(m)', font, labelpad=10)
        plt.axis('scaled')
        plt.tick_params(labelsize=13) #设置刻度字体大小
        plt.pause(0.001)
        ax.clear()

        while not rospy.is_shutdown():
            major_locator=MultipleLocator(1.5)
            ax.set_xlim([-7,7])
            ax.set_ylim([-7,7])
            font = {'size':20}
            ax.xaxis.set_major_locator(major_locator)
            ax.yaxis.set_major_locator(major_locator)
            ax.set_xlabel('x(m)', font, labelpad=15)
            ax.set_ylabel('y(m)', font, labelpad=15)    
            plt.tick_params(labelsize=13) #设置刻度字体大小
            plt.pause(0.001)
            ax.clear()
            
            # pose
            pose = getposition(N) # if turn into experimental, replace to get_robot_position(N)
            
            # note that pose have (x,y,z)
            #x_traj = np.append(x_traj, pose[0:1], axis=0) # csv
            #y_traj = np.append(y_traj, pose[1:2], axis=0) # csv

            outer = [(5,0), (-5,0), (0,-5), (0, 5)]
            pose = np.column_stack((pose[0:2]))
            print (np.column_stack((pose[0:2])))

            print("here1")
            area_shape = Polygon(outer)
            print("here2")
            poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(pose, area_shape, accept_n_coord_duplicates=0)
            print("here3")
            new_coords = reshape_coords(pose)
            poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])
            new_centroids = match_pair(poly_shapes, new_coords, poly_centroids)

            # how this gen_voronoi move information?
            #(new_coords, area, shape, poly2pt, centroids) = gen_voronoi_2(pose) # plotting related 
            plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shapes, np.column_stack((pose[0:2])), poly_to_pt_assignments,voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=30, voronoi_and_points_cmap=None)

            for x in np.column_stack((new_centroids)):
                c1 = x
                ax.plot(c1[0],c1[1], 'rs', markersize = 12, alpha = 0.4)

            #print("new_coords: ", new_coords)
            #print("centroids: ", centroids)
            point_list = cal_tra_fatii(new_coords,new_centroids)
            print("point_list:::: ", point_list)

            put_velocities_2(N, point_list)

            ##################
            ''' We need to put the point_list into bot_x and bot_y coord '''
            ##################
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
