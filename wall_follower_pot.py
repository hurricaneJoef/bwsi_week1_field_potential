#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PotentialField:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    Coor = []
    vecTor = []

    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()

        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

        #cartesian points -- to be filled (tuples)
        #self.cartPoints = [None for x in range(100)]

        self.Coor = []
        self.vecTor = []

        #[speed, angle]
        #self.finalVector = [0.5, 0]

    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()

    def drive_callback(self):
        '''Publishes drive commands'''
        #make sure to publish cmd here
        self.convertPoints()
        self.calcFinalVector()
        self.cmd.drive.speed = self.vecTor[0]
        print("speed", self.vecTor[0]) 
        self.cmd.drive.steering_angle = self.vecTor[1]
        print("angle", self.vecTor[1]) 
        self.drive_pub.publish(self.cmd)
        self.Coor = []
        self.vector = []
        

    def convertPoints(self):
        '''Convert all current LIDAR data to cartesian coordinates'''
        i = 0
        for cor in self.data:
            self.Coor.append(math.cos(float(i) / len(self.data) * 0.75 * 2 * math.pi) * cor)
            # print(math.cos(float(i) / len(self.data) * 0.75 * 2 * math.pi) * cor)
            self.Coor.append(math.sin(float(i) / len(self.data) * 0.75 * 2 * math.pi) * cor)
            # print(math.sin(float(i) / len(self.data) * 0.75 * 2 * math.pi) * cor)
            i += 1

    def calcFinalVector(self):
        '''Calculate the final driving speed and angle'''
        xsum = 0
        ysum = 0
        for cor in self.Coor:
            if cor % 2 == 0:
                xsum += cor
            elif not cor % 2 == 0:
                ysum += cor
        speed = math.sqrt(xsum * xsum + ysum * ysum) / 10
        angle = np.arctan2(ysum, xsum) # * 2 / math.pi
        self.vecTor = [speed, angle]

if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()