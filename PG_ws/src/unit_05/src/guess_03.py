#!/usr/bin/env python
#### guess_03
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
##from sensor_msgs.msg import LaserScan



class guess_03:
    """docstring fs guess_03."""


    def __init__(self):
        rospy.init_node("guess_03", anonymous=True)
        self.bridge = CvBridge()
        self.col_pos = 320
        self.row_pos = 0
        self.cv_image = None
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
        #self.laserSubscriber = rospy.Subscriber('laserscan', LaserScan, queue_size=20, callback=self.laserCallback)

        self.commandPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=20)
        self.roombaCommand = Twist()

    def callback(self,data):

        #rospy.loginfo("data image \n")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cv_image = cv_image

        except CvBridgeError as e:
            print(e)
            ROS_INFO(e)

    def imagecolor(self):
        frame = self.cv_image
        if frame is None:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv',hsv)
        # define range of blue color in HSV
        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # define range of blue color in HSV
        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        # Threshold the HSV image to get only blue colors
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask1 + mask2
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
        #midd = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)
        # Apply cv2.threshold() to get a binary image

        rows, cols = mask.shape

        """
        ymin = 480
        ymax = 0
        xmin = 640
        xmax = 0
        for row in range(rows):
            for col in range(cols):
                rgb = mask[row,col]
                if rgb > 250:
                    if row < ymin:
                        ymin = row
                    if row < ymax:
                        ymax = row
                    if col < xmin:
                        xmin = col
                    if col < xmax:
                        xmax = col

        self.col_pos = int(xmin+(xmax - xmin)/2)
        self.row_pos = int(ymin+(ymax - ymin)/2)
        """
        #a = np.arange(mask).reshape(rows, cols, )
        indices = np.where(mask > 1)
        self.col_pos, self.row_pos = tuple(np.average(indices, 1).astype(int))
        #print(self.col_pos, self.row_pos)
        center =  (self.row_pos, self.col_pos)

        strr = str(center) + "center"
        #print(strr)
        color2 = (250,250,250)
        cv2.circle(res, center, 50, color2)

        #print(rows, cols)
        #print(self.row_pos, self.col_pos)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            return

    def calculateCommand(self):

        z = 0
        self.col_pos = float(self.col_pos)
        self.row_pos = float (self.row_pos)

        x = (480 - self.col_pos)/480
        
        if self.row_pos < 320:
            z = -(320-self.row_pos)/320
        if self.row_pos > 320:
            z = (self.row_pos-320)/320
        #print(z)
        #print ( self.row_pos)
        self.roombaCommand.linear.x = 0.3  * x
        self.roombaCommand.angular.z = 0.3 * z

    #def laserCallback(self, laserData):
        # rospy.loginfo(f'got laserData {laserData.ranges}')
        #self.laserData = np.fromiter(laserData.ranges, dtype=float)

    def mainLoop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #rospy.loginfo(" mainLoop \n")

            self.imagecolor()
            self.calculateCommand()
            self.commandPublisher.publish(self.roombaCommand)
            rate.sleep()



def main(args):
    #flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
    #print (flags)
    A = guess_03()
    A.mainLoop()
    #A.imagecolor()


if __name__ == '__main__':
    main(sys.argv)
