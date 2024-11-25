#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ackermann_msgs.msg import AckermannDriveStamped


class cybertruck2:
        latest_img = None
        ackermann_msg_id = 0
        def __init__(self):
                # receive
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
                

        def pub_callback(self, event):
                self.a = self.search_env()
                ctrl = [0.0, 0.0]
                print("check self.a: ", self.a[1])
                ctrl[0] = float(self.a[1])
                ctrl[1] = float(self.a[0])
                self.publish_ctrl(self.pub, ctrl)


        def detect_flag(self, img):
                if_flag = False
                x = 0
                flag_reached = False
                img = img[:,:,::-1]
                cv2.imwrite('yellow_bgr002.png', img)

                img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                # range for blue flag
                pixel_min = np.array([90, 70, 90])
                pixel_max = np.array([126, 225, 225])

                # range for yellow flag
                pixel_min2 = np.array([20, 40, 60])
                pixel_max2 = np.array([55, 255, 255])

                mask = cv2.inRange(img, pixel_min, pixel_max)
                mask2 = cv2.inRange(img, pixel_min2, pixel_max2)

                count = np.sum(np.nonzero(mask))
                count = np.sum(mask)

                count2 = np.sum(mask2)
                if count == 0 and count2 == 0:
                if_flag = False
                else:
                if_flag = True


                if if_flag == False:
                    return [False, -1, False]

                # combine two masks
                mask = cv2.bitwise_or(mask, mask2)
                ret, thresh = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
                _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                if len(contours) > 0:
                        max_contour = max(contours, key = len)
                        flag = cv2.minAreaRect(max_contour)[0]
                                    print("check x:", flag[0])
                        x = flag[0]
                else:
                        x = -1

        
        
                if(len(contours) > 0):
                        box = cv2.boxPoints(cv2.minAreaRect(max_contour))
                        box_width = box[0][0] - box[1][0]
                        box_height = box[0][1] - box[3][1]
                    
                        print("box_width: ", box_width)
                        print("box_height: ", box_height)
                    
                        if(box_height < 5 or box_width < 5):
                                if_flag = False
                                print("You removed the noise: ", if_flag)
                    
                
                output = [False, 0, False]
                output[0] = if_flag
                output[1] = x
                output[2] = flag_reached

                return output

        def search_env(self):
                if(self.if_flag == False):
                    #a = [0.002, 0.05, 0.0]
                        a = [0.002, 0.05, 0.0]
                        return a

                # a is [steering, gas, brake(useless for this task)]
                a = [0.0, 0.05, 0.0]
                center = 640
                kp = 0.0006
                kd = 0.0003
                goal = 0
                goal_vel = 0
                cur = center - self.x
                max_a = 1

                #the error is calculated by goal minus cur
                des_a = kp*(goal-cur) 
                cl_a = np.clip(des_a, -max_a, max_a)
                a[0] = cl_a
                
                # since the camera only produces mirror image and we do not have extra camera, we return -a value
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
                pixel_min = np.array([15, 50, 70])
                pixel_max = np.array([55, 255, 255])
                mask = cv2.inRange(img, pixel_min, pixel_max)
                cv2.imwrite("take_picture_blue.jpg", mask)
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
                cv2.imwrite("input_box_blue.png", draw_box)

                f = open('flag2.txt', 'w')
                output = str(pixel[0]) + ','+ str(pixel[1]) + ',' + str(pixel[2])
                    print('output', output)
                f.write(output)
                return pixel



if __name__ == '__main__':
        rospy.init_node("drive")

        node = cybertruck2()
        rospy.spin()
