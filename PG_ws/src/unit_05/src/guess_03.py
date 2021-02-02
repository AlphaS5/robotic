#!/usr/bin/env python
#### guess_03
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

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
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

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

        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        #mask = mask1 + mask2
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
        #midd = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)
        # Apply cv2.threshold() to get a binary image
        rows, cols = mask.shape


        kernel = np.ones((3,3), np.uint8) # set kernel as 3x3 matrix from numpy
        #Create erosion and dilation image from the original image
        erosion_image = cv2.erode(gray, kernel, iterations=1)
        dilation_image = cv2.dilate(gray, kernel, iterations=1)

        indices = np.where(mask > 1)

        self.col_pos, self.row_pos = tuple(np.median(indices, 1).astype(int))
        center =  (self.row_pos, self.col_pos)
        color2 = (255,255,255)
        cv2.circle(res, center, 40, color2)
        """y, x = tuple(np.mean(indices, 1).astype(int))
        center3 =  (x, y)
        color3 = (0,0,250)
        cv2.circle(res, center, 15, color3)
        y, x = tuple(np.average(indices, 1).astype(int))
        center4 =  (x, y)
        color4 = (250,0,0)
        cv2.circle(res, center4, 30, color4)"""

        edges = cv2.Canny(frame,100,200)
        edges2 = cv2.Canny(mask,100,200)

        src = gray
        dst = cv2.Canny(src, 50, 200, None, 3)


        # Copy edges to the images that will display the results in BGR
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst, pt1, pt2, (0,0,255), 2, cv2.LINE_AA)


        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)

        cv2.imshow("Source", src)
        cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

        #cv2.imshow('Input', gray)
        #cv2.imshow('Erosion', erosion_image)
        #cv2.imshow('Dilation', dilation_image)

        #cv2.imshow('gray',gray)
        #cv2.imshow('edges of object',edges2)
        #cv2.imshow('edges of image',edges)

        #cv2.imshow('frame',frame)
        #cv2.imshow('hsv',hsv)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            return

    def calculateCommand(self):
        mirror = -1
        z = 0
        self.col_pos = float(self.col_pos)
        self.row_pos = float (self.row_pos)

        x = (480 - self.col_pos)/480

        if self.row_pos < 320:
            z = mirror*(320-self.row_pos)/320
        if self.row_pos > 320:
            z = -1 * mirror * (self.row_pos-320)/320
        #print(z)
        #print ( self.row_pos)
        self.roombaCommand.linear.x = 0.3  * x
        self.roombaCommand.angular.z = 0.3 * z


    def mainLoop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            #rospy.loginfo(" mainLoop \n")
            self.imagecolor()
            self.calculateCommand()
            self.commandPublisher.publish(self.roombaCommand)
            rate.sleep()

def main(args):
    A = guess_03()
    A.mainLoop()

if __name__ == '__main__':
    main(sys.argv)
