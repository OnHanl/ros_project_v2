#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('fake_depth_publisher')
    pub = rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    # Постоянная глубина 1.0 м на всё изображение
    depth_array = np.ones((480, 640), dtype=np.float32)

    while not rospy.is_shutdown():
        msg = bridge.cv2_to_imgmsg(depth_array, encoding='32FC1')
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_link"
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
