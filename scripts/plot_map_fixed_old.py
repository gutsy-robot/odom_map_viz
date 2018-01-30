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
import os
import rospkg

class MapViz(object):
    def __init__(self):
        rospy.loginfo("Map Visualization started...")

        
        self.vo_odom_sub1 =  rospy.Subscriber('/odom_topic1', Odometry, self.odom_cb, queue_size=1)
        # self.vo_odom_sub2 =  rospy.Subscriber('/odom_topic2', Odometry, self.odom_cb2, queue_size=1)
        self.gps_sub = rospy.Subscriber('/gps_new', NavSatFix, self.gps_cb, queue_size=1)
       
        self.initial_lat = 28.5306000
        self.initial_long = 77.1618000
        #redundant variables end.
        self.initial_odom = None
        self.offset_x = None
        self.offset_y = None

        self.p_initial = geodesy.utm.fromLatLong(self.initial_lat,self.initial_long).toPoint()

        #give the path to map file.
        rp = rospkg.RosPack()
        self.map_file_path = os.path.join(rp.get_path('odom_visualizer'), 'resources', 'map_file.jpg')
        #load it as an opencv image 
        self.map_img = cv2.imread(self.map_file_path,cv2.IMREAD_COLOR)
        self.first_msg_received = False
        self.first_x = None
        self.first_y = None

        self.last_odom_x = None
        self.last_odom_y = None

        self.last_odom_x2 = None
        self.last_odom_y2 = None
        self.last_gps = None
        #self.scaling_factor = 10000

        #opencv window declaration.
        self.cv_window = cv2.namedWindow("MapDisplay",0)

        rospy.sleep(10)
        rospy.loginfo("all objects for map visualization initialised...")


    def odom_cb(self, data):
            #print "in if block"
            self.last_odom_x = data.pose.pose.position.x
            self.last_odom_y = data.pose.pose.position.y
            #self.first_x = data.pose.pose.position.x
            #self.first_y = data.pose.pose.position.y

    # def odom_cb2(self, data):
    #         #print "in if block"
    #         self.last_odom_x2 = data.pose.pose.position.x
    #         self.last_odom_y2 = data.pose.pose.position.y
            

    def gps_cb(self, data):
        self.last_gps = data

  

    #for all the computations.
    def do_work(self):

        if(self.first_msg_received == False and self.last_gps!=None):
            p = geodesy.utm.fromLatLong(self.last_gps.latitude,self.last_gps.longitude).toPoint()
            cv2.circle(self.map_img,(int(p.x-self.p_initial.x),int(p.y-self.p_initial.y)),1,(0,0,255),-1)
            rospy.loginfo("Drawing gps on map..")

       
            zx=p.x-self.p_initial.x
            zy=p.y-self.p_initial.y
            self.offset_x = zx
            self.offset_y = zy
            if zx +self.last_odom_x >=0 and zx+ self.last_odom_x < 7164 and zy+self.last_odom_y>=0 and zy+self.last_odom_y< 3358:
                   cv2.circle(self.map_img,(int(zx+self.last_odom_x),int(zy+self.last_odom_y)),1,(0,255,0),-1)
                   # cv2.circle(self.map_img,(int(zx+self.last_odom_x2),int(zy+self.last_odom_y2)),1,(0,0,255),-1)
                   rospy.loginfo("Drawing the odom topic")
            cv2.imshow("MapDisplay", self.map_img)
            cv2.waitKey(1)
            self.first_x = int(zx+self.last_odom_x)
            self.first_y = int(zy+self.last_odom_y)
            self.first_msg_received = True

        elif(self.last_gps!=None):
            p = geodesy.utm.fromLatLong(self.last_gps.latitude,self.last_gps.longitude).toPoint()
            cv2.circle(self.map_img,(int(p.x-self.p_initial.x),int(p.y-self.p_initial.y)),1,(255,0,0),-1)
            rospy.loginfo("Drawing gps on map...")

       
            #zx=p.x-self.p_initial.x
            #zy=p.y-self.p_initial.y
            if self.offset_x +self.last_odom_x >=0 and self.offset_x+ self.last_odom_x < 7164 and self.offset_y+self.last_odom_y>=0 and self.offset_y+self.last_odom_y< 3358:
                   cv2.circle(self.map_img,(int(self.offset_x+self.last_odom_x),int(self.offset_y+self.last_odom_y)),1,(0,255,0),-1)
                   # cv2.circle(self.map_img,(int(self.offset_x+self.last_odom_x2),int(self.offset_y+self.last_odom_y2)),1,(0,0,255),-1)
                   rospy.loginfo("Drawing the odom topic")
            cv2.imshow("MapDisplay", self.map_img)
            cv2.waitKey(1)
            rospy.loginfo("xy difference is ...")
            #print int(zx+self.last_odom_x) - self.first_x
            #print int(zy+self.last_odom_y) - self.first_y





    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.do_work()
            r.sleep()

if __name__ =='__main__':
    rospy.init_node('map_visualization')
    viz = MapViz()
    viz.run()