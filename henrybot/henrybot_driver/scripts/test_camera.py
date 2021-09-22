
import numpy as np

import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image

import cv2
from cv_bridge import CvBridge

def callback(rgb_msg, camera_info):
    rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
    camera_info_K = np.array(camera_info.K).reshape([3, 3])
    camera_info_D = np.array(camera_info.D)
    rgb_undist = cv2.undistort(rgb_image, camera_info_K, camera_info_D)
    cv2.imshow("Image Window", rgb_undist)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('camera_node')
    info_sub = message_filters.Subscriber('camera_floor_left/depth/camera_info', CameraInfo)
    image_sub = message_filters.Subscriber('camera_floor_left/depth/image_rect_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.2)
    ts.registerCallback(callback)
    rospy.spin()