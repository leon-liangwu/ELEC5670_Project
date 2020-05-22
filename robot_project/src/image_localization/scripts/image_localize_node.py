#! /usr/bin/python
# image recognization and location
import rospy
import numpy as np
import math
import cv2
import dlib
import tf
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError

face_detector = dlib.get_frontal_face_detector()
img_bridge = CvBridge()

pub1 = rospy.Publisher('/localize/image_flip', Image, queue_size=10)
pub2 = rospy.Publisher('/localize/marker', MarkerArray, queue_size=10)
pub3 = rospy.Publisher('/localize/room', String, queue_size=10)
markerArray = MarkerArray()

img_w = 512
img_h = 512
cam_ang = 45
pix_ang = np.pi/4/512 
box_thresh = 100

last_img = None
last_rects = None
last_pos = None
last_room = None

area_div = [-3.6, -6.7, 5]
pre_area = 'Unknown'

img_add_list = []

def pos_callback(data):
    global last_pos, pre_area
    # rospy.loginfo('pos callback')

    if data.pose.position.x > area_div[2]:
        cur_area = 'D'
    elif data.pose.position.y > area_div[0]:
        cur_area = 'A'
    elif data.pose.position.y < area_div[1]:
        cur_area = 'C'
    else:
        cur_area = 'B'
    
    if cur_area != pre_area:
        rospy.loginfo("Robot enters in area %s" % cur_area)
        pre_area = cur_area
        
    room_msg = String()
    room_msg.data = cur_area
    pub3.publish(room_msg)

    last_pos = data

def image_callback(data):
    global last_rects
    global last_img
    try:
        img = img_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    img = cv2.flip(img, 1)

    # img_resized = cv2.resize(img, (320, 320))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # detect faces in the grayscale image
    rects = face_detector(gray, 1)
    last_img = img
    if len(rects) == 0:
        last_rects = None
    else:
        last_rects = rects
        # rospy.loginfo(('img callback', last_rects))
    
    for rect in rects:
        x1, y1 = (rect.left(), rect.top())
        x2, y2 = (rect.right(), rect.bottom())
        if x2-x1 < box_thresh or y2-y1 < box_thresh:
            continue
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # cv2.imshow('img', img)
    # cv2.waitKey(10)

    try:
        img_flip = img_bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    
    pub1.publish(img_flip)


def verify_image(pose, dist_thresh = 1.0):
    for i, pos_added in enumerate(img_add_list):
        dist = np.sum((pose - pos_added)**2)**0.5
        if dist < dist_thresh:
            return i+1
    return None


def image_bounding(img, face_rect):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
    binary, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    max_rect = face_rect
    center = (face_rect[0] + face_rect[2]/2, face_rect[1] + face_rect[3]/2)
    if len(contours) > 0:
        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
                
            rect = cv2.boundingRect(cnt)
            print(rect)

            if center[0] > rect[0] and center[0] < rect[0] + rect[2] \
                and center[1] > rect[1] and center[1] < rect[1] + rect[3]:
                if rect[2] * rect[3] > max_rect[2] * max_rect[3]:
                    max_rect = rect
        
        return max_rect

    return None



def scan_callback(data):

    if (last_rects is None) or (last_pos is None):
        return
    
    for last_rect in last_rects:
        
        rect_left = last_rect.left()
        rect_right = last_rect.right()
        rect_top = last_rect.top()
        rect_bottom = last_rect.bottom()
        face_rect = (rect_left, rect_top, rect_right-rect_left, rect_bottom-rect_top)
        # img_rect = image_bounding(last_img, face_rect)
        if face_rect[2] < box_thresh or face_rect[3] < box_thresh:
            # rospy.loginfo(('Image is too far. Go closer.', face_rect))
            continue


        angle_range = ((rect_left-img_w/2)*pix_ang + np.pi/2, (rect_right-img_w/2)*pix_ang + np.pi/2)

        las_ang_min = data.angle_min
        las_ang_max = data.angle_max
        las_len = len(data.ranges)
        las_ang_inc = np.abs(data.angle_increment)
        
        index_range = (int((angle_range[0]-las_ang_min) / las_ang_inc),
                    int((angle_range[0]-las_ang_min) / las_ang_inc))

        img_range = np.array(data.ranges[index_range[0]:index_range[1]+1]).mean()
        # rospy.loginfo((las_len, las_ang_min, las_ang_max, 
        #                 angle_range, las_ang_inc, index_range, img_range))
        
        robot_pos = last_pos.pose.position
        robot_rot = last_pos.pose.orientation

        robot_euler = tf.transformations.euler_from_quaternion((robot_rot.x, robot_rot.y, 
                                                                robot_rot.z, robot_rot.w))
        img_ang = np.pi/2- np.mean(angle_range)
        img_quat = tf.transformations.quaternion_from_euler(0, 0, img_ang)

        

        R_r2o = tf.transformations.quaternion_matrix((robot_rot.x, robot_rot.y, 
                                                        robot_rot.z, robot_rot.w))
        R_i2r = tf.transformations.quaternion_matrix(img_quat)                                                
        

        # pos_marker = np.dot(np.linalg.inv(np.dot(R_i2r, R_r2o)), np.array([[img_range], [0], [0], [1]]))
        pos_marker = np.dot(np.dot(R_i2r, R_r2o), np.array([[0], [-1*img_range], [0], [1]]))

        pos_marker = pos_marker[0:2] + np.array([[robot_pos.x], [robot_pos.y]])
        index_img = verify_image(pos_marker)
        if index_img is None:
            img_add_list.append(pos_marker)

            # rospy.loginfo((img_ang, robot_euler, pos_marker))
            # pos_marker = [robot_pos.x, robot_pos.y]
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.id = len(img_add_list)
            marker.ns = 'images'
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pos_marker[0]
            marker.pose.position.y = pos_marker[1]
            markerArray.markers.append(marker)
            rospy.loginfo(('Marked one image with id: %d' %  marker.id))
        else:
            # marker = markerArray.markers[index_img-1]
            markerArray.markers[index_img-1].pose.position.x = 0.8*pos_marker[0] + 0.2*img_add_list[index_img-1][0]
            markerArray.markers[index_img-1].pose.position.y = 0.8*pos_marker[1] + 0.2*img_add_list[index_img-1][1]
            img_add_list[index_img-1][0] = markerArray.markers[index_img-1].pose.position.x
            img_add_list[index_img-1][1] = markerArray.markers[index_img-1].pose.position.y

        pub2.publish(markerArray)


def main():
    
    rospy.init_node('image_localize', anonymous=True)
    rospy.loginfo(('Start Robot Localization.'))
    rospy.Subscriber('/vrep/image', Image, image_callback)
    rospy.Subscriber('/vrep/scan', LaserScan, scan_callback)
    rospy.Subscriber('/slam_out_pose', PoseStamped, pos_callback)

    rospy.spin()
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass