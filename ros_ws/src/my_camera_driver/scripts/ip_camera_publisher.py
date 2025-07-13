#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import websocket
import json
import threading
import numpy as np
import time
from collections import deque
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

MAGNETIC_THRESHOLD = 60.0
EMA_ALPHA = 0.1
MEDIAN_FILTER_SIZE = 5
MIN_DT = 0.005
STATIONARY_WINDOW_SEC = 1.0
STATIONARY_ACCEL_THRESHOLD = 0.05
DEAD_ZONE = 0.08  # новая мёртвая зона

latest_magnetic = [0.0, 0.0, 0.0]
trajectory = []
anomalies = []

x = y = z = 0.0
vx = vy = vz = 0.0
last_time = time.time()

accel_raw = np.array([0.0, 0.0, 0.0])
accel_ema = np.array([0.0, 0.0, 0.0])
accel_buffer = deque(maxlen=MEDIAN_FILTER_SIZE)
accel_time_buffer = deque()

trajectory_pub = None
anomaly_pub = None

def is_stationary():
    now = time.time()
    while accel_time_buffer and (now - accel_time_buffer[0][0]) > STATIONARY_WINDOW_SEC:
        accel_time_buffer.popleft()
    if not accel_time_buffer:
        return True
    accels = np.array([a for t, a in accel_time_buffer])
    avg = np.mean(np.linalg.norm(accels, axis=1))
    return avg < STATIONARY_ACCEL_THRESHOLD

def apply_dead_zone(a, threshold=DEAD_ZONE):
    return np.array([v if abs(v) > threshold else 0.0 for v in a])

def update_motion(accel, dt):
    global x, y, z, vx, vy, vz
    if is_stationary():
        vx = vy = vz = 0
        return

    accel = apply_dead_zone(accel)

    vx += accel[0] * dt
    vy += accel[1] * dt
    vz += accel[2] * dt

    # затухание скорости, если движение почти нулевое
    vx *= 0.95 if abs(accel[0]) < 0.1 else 1.0
    vy *= 0.95 if abs(accel[1]) < 0.1 else 1.0
    vz *= 0.95 if abs(accel[2]) < 0.1 else 1.0

    dx = vx * dt
    dy = vy * dt
    dz = vz * dt

    if np.linalg.norm([dx, dy, dz]) < 0.002:
        return

    x += dx
    y += dy
    z += dz

    trajectory.append((x, y, z))
    publish_trajectory()

def publish_trajectory():
    marker_array = MarkerArray()
    for i, (px, py, pz) in enumerate(trajectory):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "trajectory"
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = px
        m.pose.position.y = py
        m.pose.position.z = pz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        marker_array.markers.append(m)
    trajectory_pub.publish(marker_array)

def publish_anomaly(x, y, z):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "anomaly"
    marker.id = int(time.time() * 1000) % 100000
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    anomaly_pub.publish(marker)

def on_message_accel(ws, message):
    global accel_raw, accel_ema
    try:
        data = json.loads(message)
        accel_raw[:] = data.get("values", [0.0, 0.0, 0.0])
        accel_ema[:] = EMA_ALPHA * accel_raw + (1 - EMA_ALPHA) * accel_ema
        accel_buffer.append(accel_ema.copy())
        accel_time_buffer.append((time.time(), accel_ema.copy()))
    except: pass

def on_message_mag(ws, message):
    global latest_magnetic, last_time
    try:
        data = json.loads(message)
        mag = np.array(data.get("values", [0.0, 0.0, 0.0]))
        latest_magnetic[:] = mag.tolist()
        now = time.time()
        dt = now - last_time
        if dt < MIN_DT or len(accel_buffer) < MEDIAN_FILTER_SIZE or is_stationary():
            return
        last_time = now

        accel = accel_ema.copy()
        update_motion(accel, dt)

        if np.linalg.norm(mag) > MAGNETIC_THRESHOLD:
            anomalies.append((x, y, z))
            publish_anomaly(x, y, z)
            rospy.loginfo(f"🧲 Аномалия! Поле: {np.linalg.norm(mag):.1f} µT в точке ({x:.2f}, {y:.2f}, {z:.2f})")
    except: pass

def start_ws(url, on_message, name):
    def on_error(ws, err): rospy.logwarn(f"[{name}] ❌ {err}")
    def on_close(ws, code, reason): rospy.loginfo(f"[{name}] 🔌 Закрыто: {code} — {reason}")
    def on_open(ws): rospy.loginfo(f"[{name}] ✅ Подключено")
    ws = websocket.WebSocketApp(url, on_message=on_message, on_error=on_error, on_close=on_close, on_open=on_open)
    ws.run_forever()

def plot_trajectory():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if trajectory:
        X, Y, Z = zip(*trajectory)
        ax.plot(X, Y, Z, label="Trajectory", color='blue')
    if anomalies:
        Xa, Ya, Za = zip(*anomalies)
        ax.scatter(Xa, Ya, Za, color='red', label='Anomalies')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Траектория движения и магнитные аномалии")
    ax.legend()
    plt.show()

def main():
    global trajectory_pub, anomaly_pub
    rospy.init_node('ip_camera_publisher')
    phone_ip = rospy.get_param('~phone_ip', '192.168.100.6')
    camera_url = f"http://{phone_ip}:8080/video"
    port = 8081

    trajectory_pub = rospy.Publisher("trajectory_markers", MarkerArray, queue_size=1)
    anomaly_pub = rospy.Publisher("anomaly_markers", Marker, queue_size=10)

    sensor_urls = {
        "accel": f"ws://{phone_ip}:{port}/sensor/connect?type=android.sensor.linear_acceleration",
        "mag": f"ws://{phone_ip}:{port}/sensor/connect?type=android.sensor.magnetic_field"
    }

    threading.Thread(target=start_ws, args=(sensor_urls["accel"], on_message_accel, "Акселерометр"), daemon=True).start()
    threading.Thread(target=start_ws, args=(sensor_urls["mag"], on_message_mag, "Магнитометр"), daemon=True).start()

    cap = cv2.VideoCapture(camera_url)
    if not cap.isOpened():
        rospy.logerr("❌ Не удалось открыть камеру по адресу: " + camera_url)
        return

    bridge = CvBridge()
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(10)
    rospy.loginfo("📷 Камера и сенсоры запущены")

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                try:
                    text = f"Magnetic: X={latest_magnetic[0]:.1f}, Y={latest_magnetic[1]:.1f}, Z={latest_magnetic[2]:.1f}"
                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    pub.publish(msg)
                except Exception as e:
                    rospy.logwarn(f"Ошибка кадра: {e}")
            rate.sleep()
    finally:
        rospy.loginfo("🛑 Завершено")
        plot_trajectory()

if __name__ == "__main__":
    main()
