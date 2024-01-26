#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import cv2
from ackermann_msgs.msg import AckermannDriveStamped


# This file is for part 1 of the project
class cybertruck2:
    latest_img = None
    ackermann_msg_id = 0
    def __init__(self):
        self.a = [0.0, 0.0, 0.0]
        self.sub = rospy.Subscriber('/csi_cam_0/image_raw', Image, self.im_callback)
        self.if_flag = False
        self.x = 0.0
        self.flag_reached = False
        
        # send
        self.pub = rospy.Publisher(
            rospy.get_param(
                "~ctrl_topic",
                default = "mux/ackermann_cmd_mux/input/navigation"
            ),
            AckermannDriveStamped,
            queue_size = 10
        )

        self.im_pub = rospy.Publisher("/debug_image", Image)
        self.timer = rospy.Timer(rospy.Duration(1.0/50), self.pub_callback)



    def im_callback(self, data):
        latest_img = data
        rospy.loginfo(rospy.get_caller_id() + 'Received a new image.')
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(latest_img, desired_encoding = 'passthrough')
        self.if_flag, self.x, self.flag_reached = self.detect_flag(cv_image)
        
        img = cv_image[:,:,::-1]

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        pixel_min = np.array([36, 50, 70])
        pixel_max = np.array([86, 255, 255])

        mask = cv2.inRange(img, pixel_min, pixel_max)
        ret, thresh = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            max_contour = max(contours, key = len)
            box = cv2.boxPoints(cv2.minAreaRect(max_contour))
            box = np.int0(box)
            img_bgr = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
            # frame_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            draw_box = cv2.drawContours(img_bgr,[box],0,255,2)
            img_to_pub = bridge.cv2_to_imgmsg(draw_box)
            self.im_pub.publish(img_to_pub)
         

    def pub_callback(self, event):

        self.a = self.search_env()
        ctrl = [0.0, 0.0]
        ctrl[0] = float(self.a[1])
        ctrl[1] = float(self.a[0])

        self.publish_ctrl(self.pub, ctrl)


    def detect_flag(self, img):
        if_flag = False
        x = 0
        flag_reached = False
        img = img[:,:,::-1]
        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pixel_file = open("flag.txt", "r")
        pixel_list = pixel_file.read().split(",")
        pixel = list(map(int, pixel_list))

        pixel_min = np.array([36, 50, 70])
        pixel_max = np.array([86, 255, 255])
        mask = cv2.inRange(img, pixel_min, pixel_max)
        count = np.sum(mask)
        if count == 0:
            if_flag = False
        else:
            if_flag = True
        
        ret, thresh = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0: 
            max_contour = max(contours, key = len)
            flag = cv2.minAreaRect(max_contour)[0]
            x = flag[0]
        else:
            x = -1
    
        
        
        if(len(contours) > 0):
            box = cv2.boxPoints(cv2.minAreaRect(max_contour))
            box_width = box[0][0] - box[1][0]
            box_height = box[0][1] - box[3][1]
            
            if(box_height < 10 or box_width < 10):
                if_flag = False
            if(box_height > 250 or box_width > 506):
                flag_reached = True
            else:
                flag_reached = False
        
        output = [False, 0, False]
        output[0] = if_flag
        output[1] = x
        output[2] = flag_reached

        return output

    def search_env(self):
        if(self.if_flag == False):
            a = [1.0, 0.05, 0.0]
            return a
        elif(self.flag_reached == True):
          #  print("The car has reached the flag")
            a = [0.0, 0.0, 0.0]
            return a

        a = [0.0, 0.3, 0.0]
        center = 640

        kp = 0.0005
        kd = 0.0003
        goal = 0
        goal_vel = 0
        cur = center - self.x
        max_a = 1


        # error is goal minus cur
        des_a = kp*(goal-cur) 
        

        cl_a = np.clip(des_a, -max_a, max_a)
        a[0] = cl_a
        
        # since our camera only produced up side down image, and we did not have extra camera, we return -a value which can solve the problem
        a[0] = -a[0]
        return a           
        


    def publish_ctrl(self, rp_ctrls, ctrl):
        assert len(ctrl) == 2
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        # ctrlmsg.header.seq = ackermann_msg_id
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        rp_ctrls.publish(ctrlmsg)    
        rospy.loginfo("publishing ctrl")


def register_flag(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    pixel_min = np.array([36, 0, 0])
    pixel_max = np.array([86, 255, 255])
    mask = cv2.inRange(img, pixel_min, pixel_max)
    cv2.imwrite("take_picture.jpg", mask)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key = len)
    flag = cv2.minAreaRect(max_contour)[0]
    x = int(flag[0])
    y = int(flag[1])

    pixel = img[y, x]    
    
    
    box = cv2.boxPoints(cv2.minAreaRect(max_contour))
    box = np.int0(box)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    draw_box = cv2.drawContours(img_bgr,[box],0,255,2)
    cv2.imwrite("input_box.png", draw_box)
    f = open('flag.txt', 'w')
    output = str(pixel[0]) + ','+  str(pixel[1]) + ',' + str(pixel[2])
    f.write(output)
    return pixel

if __name__ == '__main__':
    rospy.init_node("drive")

    node = cybertruck2()
    rospy.spin()

