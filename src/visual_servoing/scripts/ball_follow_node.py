#! /usr/bin/python
# image recognization and location
import rospy
import numpy as np
import math
import cv2
import tf
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

img_bridge = CvBridge()

pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
pub_switch = rospy.Publisher('/vrep/laser_switch', Bool, queue_size=10)

target_flag = False

def detect_ball(img, rgb):
    scale = 4
    (rows, cols, ch) = img.shape
    img_rs = cv2.resize(img, (rows/4, cols/4))
    xy_list = []
    (rows, cols, ch) = img_rs.shape
    bgr = rgb[::-1]
    for r in range(rows):
        for c in range(cols):
            if np.abs(img_rs[r, c, :] - bgr).sum() < 50:
                xy_list.append([c, r])
        
    if len(xy_list) == 0:
        return None

    xy_array = np.array(xy_list)
    rect = [xy_array[:, 0].min(), 
            xy_array[:, 1].min(),
            xy_array[:, 0].max(),
            xy_array[:, 1].max()]

    # rospy.loginfo(rect)

    cv2.rectangle(img, (rect[0]*scale, rect[1]*scale), (rect[2]*scale, rect[3]*scale), (0, 255, 0), 2)
    target = np.array([(rect[0]+rect[2])*0.5, (rect[1]+rect[3])*0.5, rect[2]-rect[0], rect[3]-rect[1]])*scale

    return target

def image_callback(data):
    global target_flag
    try:
        img = img_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    img = cv2.flip(img, 1)
    rgb = np.array([255, 255, 0])
    target = detect_ball(img, rgb)

    if (target is not None) and target[2] > 100 and (not target_flag):
        scan_switch = Bool()
        scan_switch.data = False
        pub_switch.publish(scan_switch)
        target_flag = True
        rospy.loginfo('Target is detected. Start auto-tracking...')
    
    if target_flag:
        if target is None:
            rospy.loginfo('Target is lost! Use key to control...')
            target_flag = False
            return
        speed_factor =  0.4 * img.shape[1] / target[2]
        angle_factor = (img.shape[1]*0.5 - target[0]) / (img.shape[1]/2)
        twist = Twist()
        twist.linear.x = np.min([speed_factor * 0.5, 0.8])
        twist.angular.z = angle_factor * 1.0
        pub.publish(twist)


    # cv2.imshow('img_ball', img)
    # cv2.waitKey(10)

def main():
    
    rospy.init_node('ball_follow', anonymous=True)
    rospy.loginfo(('Start Visual Servoing.'))
    rospy.Subscriber('/vrep/image', Image, image_callback)

    rospy.spin()
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass