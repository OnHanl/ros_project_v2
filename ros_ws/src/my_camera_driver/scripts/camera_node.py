#!/usr/bin/env python3
import rospy
import cv2
import yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class IPCameraNode:
    def __init__(self):
        rospy.init_node('ip_camera_node', anonymous=True)

        self.video_url = rospy.get_param('~video_url', 'http://192.168.124.119:8080/video')
        self.camera_info_url = rospy.get_param('~camera_info_url', '')
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        self.publish_rate = rospy.get_param('~publish_rate', 30)

        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        
        self.bridge = CvBridge()
        self.cap = None
        self.camera_info_msg = None

        if self.camera_info_url:
            try:
                # ВЫЗЫВАЕМ ИСПРАВЛЕННУЮ ФУНКЦИЮ ЗАГРУЗКИ
                self.camera_info_msg = self.load_camera_info_from_url(self.camera_info_url)
                self.camera_info_msg.header.frame_id = self.frame_id
                rospy.loginfo(f"Successfully loaded camera info from {self.camera_info_url}")
            except Exception as e:
                rospy.logerr(f"!!! FAILED TO LOAD CAMERA INFO: {e} !!!")
                self.camera_info_msg = None
        else:
            rospy.logwarn("camera_info_url not set, publishing empty or no CameraInfo.")

    def load_camera_info_from_url(self, url):
        """
        ИСПРАВЛЕННАЯ ФУНКЦИЯ: Загружает данные калибровки из URL, 
        правильно обрабатывая префикс 'file://'.
        """
        # --- ВОТ ИСПРАВЛЕНИЕ ---
        path = url
        if url.startswith('file://'):
            path = url[7:] # Убираем 'file://' чтобы получить чистый путь
        
        rospy.loginfo(f"Attempting to open calibration file at path: {path}")
        with open(path, "r") as f:
            calib_data = yaml.safe_load(f)
        
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def connect_to_stream(self):
        """Попытка подключения к видеопотоку."""
        rospy.loginfo(f"Attempting to connect to video stream at {self.video_url}...")
        self.cap = cv2.VideoCapture(self.video_url)
        if not self.cap.isOpened():
            rospy.logwarn("Failed to open video stream. Retrying in 5 seconds...")
            self.cap = None
            return False
        rospy.loginfo("Video stream opened successfully.")
        return True

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            if self.cap is None:
                if not self.connect_to_stream():
                    rospy.sleep(5.0)
                    continue
            
            ret, frame = self.cap.read()
            if ret:
                now = rospy.Time.now()
                
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = now
                ros_image.header.frame_id = self.frame_id
                self.image_pub.publish(ros_image)
                
                if self.camera_info_msg:
                    self.camera_info_msg.header.stamp = now
                    self.info_pub.publish(self.camera_info_msg)
            else:
                rospy.logwarn("Failed to grab frame. Reconnecting...")
                self.cap.release()
                self.cap = None
            
            rate.sleep()

        if self.cap:
            self.cap.release()

if __name__ == '__main__':
    try:
        node = IPCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass