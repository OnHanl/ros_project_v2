#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

def main():
    rospy.init_node('fake_depth_publisher')
    pub = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(10)  # 10 Hz

    # Размер изображения
    width, height = 640, 480

    while not rospy.is_shutdown():
        depth_array = np.full((height, width), 1000, dtype=np.uint16)  # 1000 мм = 1 метр
        msg = bridge.cv2_to_imgmsg(depth_array, encoding='16UC1')
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_link"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
