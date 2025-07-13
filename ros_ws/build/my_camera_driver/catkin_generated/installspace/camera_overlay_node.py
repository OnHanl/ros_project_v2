#!/usr/bin/env python3
import rospy
import cv2
import requests
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature

class CameraOverlayNode:
    def __init__(self):
        rospy.init_node('camera_overlay_node')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/overlay', Image, queue_size=1)

        self.camera_url = rospy.get_param('~camera_url', 'http://192.168.100.229:8080/shot.jpg')
        self.sensor_url = rospy.get_param('~sensor_url', 'http://192.168.100.229:8081/sensors.json')

        self.rate = rospy.Rate(5)  # 5 Гц

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Получение изображения
                img_resp = requests.get(self.camera_url, timeout=1)
                img_array = np.asarray(bytearray(img_resp.content), dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                # Получение данных с сенсоров
                sensor_resp = requests.get(self.sensor_url, timeout=1)
                data = sensor_resp.json()

                # Отображение текста на изображении
                if 'temp' in data:
                    temp = f"Temp: {float(data['temp']['data']):.1f} °C"
                    cv2.putText(frame, temp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

                if 'light' in data:
                    light = f"Light: {float(data['light']['data']):.1f}"
                    cv2.putText(frame, light, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

                # Преобразуем и публикуем
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(image_msg)

            except Exception as e:
                rospy.logwarn(f"Ошибка получения изображения или сенсоров: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    import numpy as np
    node = CameraOverlayNode()
    node.run()
