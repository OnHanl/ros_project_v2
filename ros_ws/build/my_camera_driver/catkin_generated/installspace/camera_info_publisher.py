#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo

def main():
    rospy.init_node('fake_camera_info')
    pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)
    rate = rospy.Rate(10)
    
    msg = CameraInfo()
    msg.header.frame_id = "camera_link"
    msg.width = 640
    msg.height = 480

    # Оставим все матрицы по умолчанию (нули)
    # Это допустимо для тестирования, RTAB-Map примет

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
