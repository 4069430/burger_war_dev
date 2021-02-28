#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program
by Takuya Yamaguhi.
'''

import rospy
import random
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from mask1 import detect_red_color
from mask1 import detect_green_color
from mask1 import detect_blue_color
from mask1 import detect_yellow_color
from mask1 import detect_white_color
#from mask import mask_green
#from mask import mask_red
from sensor_msgs.msg import LaserScan


class RandomBot():
    def __init__(self, bot_name="NoName", use_camera=False,use_lidar=False):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        if use_camera:
            # for convert image topic to opencv obj
            self.img_red = 0
            self.img_green = 0
            self.img_blue = 0
            self.img_yellow = 0
            self.img_white = 0
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.back_count = 0
            self.blue_count = 0


        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

    def calcTwist(self):


        if (230 < self.img_white) and (self.img_blue < 10) and  (self.img_red < 55):  #CASE1:near only the white wall 
            x = -1
            r_int = random.randint(-1,1)
            th = r_int
            self.back_count += 1
            
            print('--------')
            print('|   2   |')
            print('--------')
            print(self.back_count)
        elif (self.img_blue > 10):   #CASE2:Hit the  blue wall 
            x = 0.1
            th = 0
            self.blue_count += 1

            if (self.blue_count > 3):
                x = -1.5
                
            if (self.blue_count > 5):
                self.back_count = 0 

            print('--------')
            print('|   3   |')
            print(self.blue_count)
            print('--------')
        elif (self.img_white > 150) and (self.img_red > 55):
            x = -2
            th = 0
            print('--------')
            print('|   4   |')
            print('--------')
        else:          #CASE1:Don't see the enemy, random
            r_int = random.randint(-1,1)
            th = r_int
            x = 0.5
            print('--------')
            print('|   1  |')
            print('--------')


        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        #masked_img_r = mask_red(self.img)
        #masked_img_g = mask_green(self.img)
        self.img_red = detect_red_color(self.img)
        self.img_green = detect_green_color(self.img)
        self.img_blue = detect_blue_color(self.img)
        self.img_yellow = detect_yellow_color(self.img)
        self.img_white = detect_white_color(self.img)
        
        print('Red')
        print(self.img_red)
        print('green')
        print(self.img_green)
        print('bule')
        print(self.img_blue)
        print('yellow')
        print(self.img_yellow)
        print('white')
        print(self.img_white)      

        #grayimg_r = cv2.cvtColor(masked_img_r, cv2.COLOR_BGR2GRAY) #グレースケール化
        #grayimg_g = cv2.cvtColor(masked_img_g, cv2.COLOR_BGR2GRAY) #グレースケール化
        #self.gray_pix_num = len(grayimg_r[grayimg_r<255])+len(grayimg_g[grayimg_g<255])   #黒い点の数を数える

        #neg_grayimg_r = cv2.bitwise_not(grayimg_r)
        #self.mu_r = cv2.moments(neg_grayimg_r, False)

        #neg_grayimg_g = cv2.bitwise_not(grayimg_g)
        #self.mu_g = cv2.moments(neg_grayimg_g, False)
       
        cv2.waitKey(1)

    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)
        #print(self.scan)
   

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random', use_camera=True,use_lidar=True)
    bot.strategy()