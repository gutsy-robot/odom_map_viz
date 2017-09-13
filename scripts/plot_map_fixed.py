#!/usr/bin/env python



import cv2
import geodesy.props
import geodesy.utm
import geodesy.wu_point
import rospy
import numpy as np
import itertools
import socket
import sys
import pickle
from decimal import *
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math


class MapViz(object):
    def __init__(self):
        rospy.loginfo("Map Visualization started...")

        #subscribe to odom topic.
        self.odom_sub =  rospy.Subscriber('/vo_new', Odometry, self.odom_cb, queue_size=1)
        self.gps_sub = rospy.Subscriber('/gps_new', NavSatFix, self.gps_cb, queue_size=1)

        #seems like redundant to me.
        self.gps_flag = False
        self.odom_flag = False
        self.initial_lat = 28.5306000
        self.initial_long = 77.1618000
        #redundant variables end.

        self.p_initial = geodesy.utm.fromLatLong(self.initial_lat,self.initial_long).toPoint()

        #give the path to map file.
        self.map_file_path = '/home/siddharth/map_file.jpg'
        #load it as an opencv image 
        self.map_img = cv2.imread(self.map_file_path,cv2.IMREAD_COLOR)
        print self.map_img.shape

        self.last_odom_x = None
        self.last_odom_y = None
        self.last_gps = None
        #self.scaling_factor = 10000

        #opencv window declaration.
        self.cv_window = cv2.namedWindow("MapDisplay",0)

        rospy.sleep(19)
        rospy.loginfo("all objects for map visualization initialised...")


    def odom_cb(self, data):
        self.last_odom_x = data.pose.pose.position.x
        self.last_odom_y = data.pose.pose.position.y

    def gps_cb(self, data):
        self.last_gps = data


    #for all the computations.
    def do_work(self):

        p = geodesy.utm.fromLatLong(self.last_gps.latitude,self.last_gps.longitude).toPoint()
        cv2.circle(self.map_img,(int(p.x-self.p_initial.x),int(p.y-self.p_initial.y)),1,(0,0,255),-1)

        print self.map_img.shape
        zx=p.x-self.p_initial.x
        zy=p.y-self.p_initial.y
        if zx +self.last_odom_x >=0 and zx+ self.last_odom_x < 7164 and zy+self.last_odom_y>=0 and zy+self.last_odom_y< 3358:
                   cv2.circle(self.map_img,(int(zx+self.last_odom_x),int(zy+self.last_odom_y)),1,(255,0,0),-1)
        cv2.imshow("MapDisplay", self.map_img)
        cv2.waitKey(1)







    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.do_work()
            r.sleep()

if __name__ =='__main__':
    rospy.init_node('map_visualization')
    viz = MapViz()
    viz.run()