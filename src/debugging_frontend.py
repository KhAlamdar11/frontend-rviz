#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import quaternion_from_euler
import roslib

# import 
import matplotlib.pyplot as plt
import numpy as np
import copy
import sys
import cv2
import matplotlib.pyplot as plt
import time
import skimage
import math
from skimage import measure, draw
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError

class visualize_topics:       
    def __init__(self):
        self.rate = rospy.Rate(1)
        self.myMarker = Marker()
        self.myMarker.header.frame_id = "/map"
        self.myMarker.header.stamp  = rospy.get_rostime()
        self.motion_busy = None
        #self.myMarker.ns = "window"
        self.myMarker.id = 1
        self.myMarker.type = self.myMarker.SPHERE # sphere
        self.myMarker.action = self.myMarker.ADD

        self.myPoint = Point()
        self.myPoint.z = 1.5
        self.myMarker.pose.position = self.myPoint
        
        q = quaternion_from_euler(0, 0, 0)
        self.myMarker.pose.orientation.x=q[0]
        self.myMarker.pose.orientation.y=q[1]
        self.myMarker.pose.orientation.z=q[2]
        self.myMarker.pose.orientation.w=q[3]
        self.myMarker.color=ColorRGBA(0, 1, 0, 1)
        self.myMarker.scale.x = 0.2
        self.myMarker.scale.y = 0.2
        self.myMarker.scale.z = 0.2
        self.myMarker.lifetime = rospy.Duration(0)
        
        self.marker_Arr = MarkerArray()
        self.marker_Arr.markers = []

        self.marker_Arr_humans = MarkerArray()
        self.marker_Arr_humans.markers = []


        self.first_publisher = rospy.Publisher("/sphere",Marker,queue_size=1) #occupancy grid publisher
        self.path_publisher = rospy.Publisher("/vis/path",MarkerArray,queue_size=1) #occupancy grid publisher
        self.human_publisher = rospy.Publisher("/vis/humans",MarkerArray,queue_size=1) #occupancy grid publisher

        self.waypoints_sub = rospy.Subscriber("/frontier/waypoints",Float64MultiArray,self.path_pub) #occupancy grid publisher
        self.human_loc_sub = rospy.Subscriber("human_detection/human_locations",Float64MultiArray,self.human_pub)

        self.motion_planner_busy = rospy.Subscriber("/motion_planner/motion_busy",Int8,self.motion_busy_update) #occupancy grid publisher
        print("Hello Im here")
        #self.path_pub([3,1,1,1,2,2,2, 3,3,3])
        #self.frontier_status_publisher = rospy.Publisher("/frontier/status",Int32MultiArray,queue_size=1) #occupancy grid publisher
        #self.frontier_grid_goal_publisher = rospy.Publisher("/frontier/grid_goal",Int32MultiArray,queue_size=1) #occupancy grid publisher
        #self.grid_pos_publisher = rospy.Publisher("/frontier/grid_pos",Int32MultiArray,queue_size=1) #occupancy grid publisher

        #self.occupancy_filtered_grid_publisher = rospy.Publisher("/frontier/filtered_occupancy",OccupancyGrid,queue_size=1) #occupancy grid publisher

        #self.map_subscriber = rospy.Subscriber("/projected_map", OccupancyGrid, self.__projected_map_callback) #/rtabmap/grid_map.
        #self.drone_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.__pose_callback)

        
        #self.waypoints_sub = rospy.Subscriber("/frontier/waypoints",Float64MultiArray,queue_size=10) #occupancy grid publisher
    def motion_busy_update(self,data):
        self.motion_busy = data.data

    def path_pub(self,data):   
        data = data.data
        print(data)
        # while not rospy.is_shutdown():
        self.marker_Arr.markers = []
        for i in range(1,len(data),3):
            if self.motion_busy==0:
                self.myMarker = Marker()
                self.myMarker.header.frame_id = "/map"
                self.myMarker.type = self.myMarker.SPHERE # sphere
                self.myMarker.action = self.myMarker.ADD

                self.myPoint = Point()
                self.myPoint.x = data[i+1]
                self.myPoint.y = -data[i]
                print(self.myPoint.x,self.myPoint.y)
                self.myMarker.pose.position = self.myPoint
                
                q = quaternion_from_euler(0, 0, 0)
                self.myMarker.pose.orientation.x=q[0]
                self.myMarker.pose.orientation.y=q[1]
                self.myMarker.pose.orientation.z=q[2]
                self.myMarker.pose.orientation.w=q[3]
                self.myMarker.color=ColorRGBA(0, 1, 0, 1)
                self.myMarker.scale.x = 0.2
                self.myMarker.scale.y = 0.2
                self.myMarker.scale.z = 0.2
                self.myMarker.lifetime = rospy.Duration(0)
                
                self.marker_Arr.markers.append(self.myMarker)
                id = 0
                for m in self.marker_Arr.markers:
                    m.id = id
                    id += 1
                self.path_publisher.publish(self.marker_Arr)
                print("Im inside")
            else:
                print("Currently Busy Executing Path")
            

    def human_pub(self,data):
        human_loc_arr = data.data
        print('humans: ', human_loc_arr)
        self.marker_Arr_humans.markers = []
        for i in range(0,len(human_loc_arr),2):
            self.myMarker = Marker()
            self.myMarker.header.frame_id = "/map"
            self.myMarker.type = self.myMarker.CUBE # sphere
            self.myMarker.action = self.myMarker.ADD

            self.myPoint = Point()
            self.myPoint.x = human_loc_arr[i]
            self.myPoint.y = human_loc_arr[i+1]
            print(self.myPoint.x,self.myPoint.y)
            self.myMarker.pose.position = self.myPoint
            
            q = quaternion_from_euler(0, 0, 0)
            self.myMarker.pose.orientation.x=q[0]
            self.myMarker.pose.orientation.y=q[1]
            self.myMarker.pose.orientation.z=q[2]
            self.myMarker.pose.orientation.w=q[3]
            self.myMarker.color=ColorRGBA(1, 0, 0, 1)
            self.myMarker.scale.x = 0.4
            self.myMarker.scale.y = 0.4
            self.myMarker.scale.z = 0.4
            self.myMarker.lifetime = rospy.Duration(0)
            
            self.marker_Arr_humans.markers.append(self.myMarker)
        id = 0
        for m in self.marker_Arr_humans.markers:
            m.id = id
            id += 1

        #self.first_publisher.publish(self.myMarker)
        self.human_publisher.publish(self.marker_Arr_humans)
        print("Im inside human")
        #        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('visualize_topics', anonymous=True)
    n = visualize_topics()
    rospy.spin()
