#! /usr/bin/python
# image recognization and location
import rospy
import cv2
import dlib
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

face_detector = dlib.get_frontal_face_detector()
pub1 = rospy.Publisher('vrep/image_flip', Image, queue_size=10)
img_bridge = CvBridge()

last_img = None
last_rect = None

def image_callback(data):

    try:
        img = img_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    img = cv2.flip(img, 1)

    # img_resized = cv2.resize(img, (500)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # detect faces in the grayscale image
    rects = face_detector(gray, 1)
    last_img = img
    if len(rects) == 0:
        last_rect = None
    else:
        last_rect = rects[0]
    
    for rect in rects:
        x1, y1 = (rect.left(), rect.top())
        x2, y2 = (rect.right(), rect.bottom())
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        rospy.loginfo(rect)
    
    cv2.imshow('img', img)
    cv2.waitKey(10)

    try:
        img_flip = img_bridge.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    
    pub1.publish(img_flip)


def scan_callback(data):
    rospy.loginfo(len(data.ranges))

def main():
    
    rospy.init_node('image_localize', anonymous=True)
    rospy.Subscriber('vrep/image', Image, image_callback)
    rospy.Subscriber('vrep/scan', LaserScan, scan_callback)

    rospy.spin()
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass