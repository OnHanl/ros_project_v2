#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header
import websocket
import json
import time

pub = None  # Глобальный паблишер

def on_message(ws, message):
    try:
        data = json.loads(message)
        values = data.get("values", [0.0, 0.0, 0.0])

        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "magnetometer_link"
        msg.magnetic_field.x = values[0] * 1e-6  # μT → T
        msg.magnetic_field.y = values[1] * 1e-6
        msg.magnetic_field.z = values[2] * 1e-6

        pub.publish(msg)
    except Exception as e:
        rospy.logwarn("Ошибка обработки сообщения: %s", e)

def on_open(ws):
    rospy.loginfo("✅ Подключено к Sensor Server")

def on_error(ws, error):
    rospy.logerr("❌ Ошибка WebSocket: %s", error)

def on_close(ws, code, reason):
    rospy.loginfo("🔌 Соединение закрыто: %s — %s", code, reason)

if __name__ == "__main__":
    rospy.init_node('magnetometer_publisher')

    # Получаем параметры из launch-файла
    phone_ip = rospy.get_param("~phone_ip", "127.0.0.1")
    sensor_type = rospy.get_param("~sensor_type", "android.sensor.magnetic_field")
    port = 8081
    url = f"ws://{phone_ip}:{port}/sensor/connect?type={sensor_type}"

    rospy.loginfo("🌐 Подключение к %s на %s", sensor_type, url)

    pub = rospy.Publisher('/magnetometer/data', MagneticField, queue_size=10)

    ws = websocket.WebSocketApp(
        url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )

    # Запуск WebSocket-клиента
    ws.run_forever()
