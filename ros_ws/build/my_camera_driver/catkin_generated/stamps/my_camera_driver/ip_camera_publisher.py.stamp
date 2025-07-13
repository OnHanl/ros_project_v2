#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

bridge = CvBridge()
trajectory_pub = None
anomaly_pub = None
image_pub = None

def generate_snake_scan_trajectory(step=0.1, turn_smoothness=10):
    trajectory = []
    anomalies = []

    wall_width = 2.0
    wall_height = 1.5
    x_fixed = 1.0
    direction = 1
    y = 0.0

    while y < wall_height:
        z_start = 0.0 if direction % 2 else wall_width
        z_end = wall_width if direction % 2 else 0.0
        z_values = np.linspace(z_start, z_end, int(abs(z_end - z_start) / step))

        for z in z_values:
            z_perturb = z + np.random.normal(0, 0.005)
            y_perturb = y + np.random.normal(0, 0.003)
            trajectory.append((x_fixed, y_perturb, z_perturb))
            if 0.9 < z < 1.1 and 0.7 < y < 0.9:
                for i in range(3):  # "гусеница" — 3 аномалии подряд
                    anomalies.append((x_fixed, y_perturb + i * 0.01, z_perturb + np.random.uniform(-0.01, 0.01)))

        if y + step < wall_height:
            y_turn = np.linspace(y, y + step, turn_smoothness)
            z_turn = np.linspace(z_end, z_end, turn_smoothness)
            offset = (z_end - z_start) * 0.1
            z_curve = z_turn + offset * np.sin(np.linspace(0, np.pi, turn_smoothness))

            for yy, zz in zip(y_turn, z_curve):
                yy_perturb = yy + np.random.normal(0, 0.003)
                zz_perturb = zz + np.random.normal(0, 0.005)
                trajectory.append((x_fixed, yy_perturb, zz_perturb))
                if 0.9 < zz < 1.1 and 0.7 < yy < 0.9:
                    for i in range(2):
                        anomalies.append((x_fixed, yy_perturb + i * 0.01, zz_perturb + np.random.uniform(-0.01, 0.01)))

        y += step
        direction += 1

    return trajectory, anomalies

def publish_marker_array(points):
    marker_array = MarkerArray()
    for i, (x, y, z) in enumerate(points):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "trajectory"
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        marker_array.markers.append(m)
    trajectory_pub.publish(marker_array)

def publish_anomaly_marker(x, y, z, id_):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "anomaly"
    m.id = id_
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0
    anomaly_pub.publish(m)

def send_fake_video_loop():
    cap = cv2.VideoCapture(0)  # Можешь заменить на свой IP camera URL
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_pub.publish(msg)
            except Exception as e:
                rospy.logwarn(f"Ошибка видео: {e}")
        rate.sleep()

def simulate_movement():
    trajectory, anomalies = generate_snake_scan_trajectory()
    rate = rospy.Rate(20)
    traj_so_far = []

    for i, point in enumerate(trajectory):
        traj_so_far.append(point)
        publish_marker_array(traj_so_far)
        for j, a in enumerate(anomalies):
            if np.linalg.norm(np.array(point) - np.array(a)) < 0.03:
                publish_anomaly_marker(*a, id_=i*10 + j)
        rate.sleep()

def main():
    global trajectory_pub, anomaly_pub, image_pub
    rospy.init_node("ip_camera_publisher")
    trajectory_pub = rospy.Publisher("trajectory_markers", MarkerArray, queue_size=1)
    anomaly_pub = rospy.Publisher("anomaly_markers", Marker, queue_size=10)
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=10)

    # Запуск потоков
    threading.Thread(target=send_fake_video_loop, daemon=True).start()
    threading.Thread(target=simulate_movement, daemon=True).start()

    rospy.spin()

if __name__ == "__main__":
    main()
