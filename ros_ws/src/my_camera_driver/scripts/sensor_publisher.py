#!/usr/bin/env python3
import rospy
import requests
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, Temperature
import math

def get_sensor_data(sensor_url):
    try:
        res = requests.get(sensor_url, timeout=1)
        return res.json()
    except:
        return None

def main():
    rospy.init_node('sensor_publisher')

    sensor_url = rospy.get_param('~sensor_url', 'http://192.168.100.6:8081/sensors.json')

    temp_pub = rospy.Publisher('/sensor/temperature', Temperature, queue_size=1)
    light_pub = rospy.Publisher('/sensor/light', Float32, queue_size=1)
    accel_pub = rospy.Publisher('/sensor/accel', Imu, queue_size=1)

    rate = rospy.Rate(1)  # 1 Гц
    while not rospy.is_shutdown():
        data = get_sensor_data(sensor_url)
        if data:
            if 'temp' in data:
                temp = Temperature()
                temp.temperature = float(data['temp']['data'])
                temp_pub.publish(temp)

            if 'light' in data:
                light = Float32()
                light.data = float(data['light']['data'])
                light_pub.publish(light)

            if 'accel' in data:
                imu = Imu()
                imu.linear_acceleration.x = float(data['accel']['data'][0])
                imu.linear_acceleration.y = float(data['accel']['data'][1])
                imu.linear_acceleration.z = float(data['accel']['data'][2])
                accel_pub.publish(imu)

        rate.sleep()

if __name__ == '__main__':
    main()
