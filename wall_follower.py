#!/usr/bin/env python

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")

    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    def scan(self, data):
    	  #stores the lidar data so you can work with it
        self.data = data

    	  #calls function that controls driving
        self.drive()

    def drive(self):
        """controls driving"""
        #gets the angle required
        self.angle = self.find_wall()
        #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle

        #publishes the command
        self.drive_pub.publish(self.cmd)

    def go_to_wall(self):
        """Goes to the wall when the car is far from the wall"""

    def find_wall(self):
	  tempAngle = 0
    	  # if lidar data has not been received, do nothing
    	  if self.data == None:
              return 0
    	  ## TO DO: Find Alg for Wall Following ##
          if self.data.ranges[0] > self.data.ranges[33]:
              tempAngle = 0.5
	  if self.data.ranges[0] < self.data.ranges[33]:
	      tempAngle = -0.5
    	  """Lidar data is now stored in self.data, which can be accessed
    	  using self.data.ranges (in simulation, returns an array).
    	  Lidar data at an index is the distance to the nearest detected object
    	  self.data.ranges[0] gives the leftmost lidar point
    	  self.data.ranges[99] gives the rightmost lidar point
    	  self.data.ranges[50] gives the forward lidar point
    	  """
    	  #returns the output of your alg, the new angle to drive in
    	  #print(tempAngle)
	  return tempAngle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
