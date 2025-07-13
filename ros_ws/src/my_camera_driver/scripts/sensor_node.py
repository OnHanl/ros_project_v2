#!/usr/bin/env python3

import rospy
import websocket
import json
import urllib.parse  # <-- Важный импорт для кодирования URL
from sensor_msgs.msg import Imu, MagneticField

# --- Глобальные переменные ---
last_accel = None
last_gyro = None
imu_pub = None
mag_pub = None

def on_message(ws, message):
    """
    Колбэк, вызываемый при получении сообщения от WebSocket сервера.
    """
    global last_accel, last_gyro, imu_pub, mag_pub
    
    if imu_pub is None or mag_pub is None:
        return

    try:
        data = json.loads(message)
        # Согласно документации, при запросе нескольких сенсоров,
        # в ответе будет поле 'type', указывающее, от какого сенсора пришли данные.
        sensor_type = data.get('type') 
        values = data.get('values')
        ros_time = rospy.Time.now()

        if sensor_type == 'android.sensor.accelerometer':
            last_accel = values
            if last_gyro:
                publish_imu(ros_time)

        elif sensor_type == 'android.sensor.gyroscope':
            last_gyro = values
            if last_accel:
                publish_imu(ros_time)
        
        elif sensor_type == 'android.sensor.magnetic_field':
            mag_msg = MagneticField()
            mag_msg.header.stamp = ros_time
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = values[0] / 1e6 
            mag_msg.magnetic_field.y = values[1] / 1e6
            mag_msg.magnetic_field.z = values[2] / 1e6
            mag_pub.publish(mag_msg)

    except json.JSONDecodeError:
        rospy.logwarn(f"Received non-JSON message or invalid JSON: {message}")
    except Exception as e:
        rospy.logerr(f"Error processing sensor message: {e}")

def publish_imu(time):
    """
    Вспомогательная функция для сборки и публикации сообщения Imu.
    """
    global last_accel, last_gyro, imu_pub
    if not last_accel or not last_gyro:
        return

    imu_msg = Imu()
    imu_msg.header.stamp = time
    imu_msg.header.frame_id = 'imu_link'
    imu_msg.linear_acceleration.x = last_accel[0]
    imu_msg.linear_acceleration.y = last_accel[1]
    imu_msg.linear_acceleration.z = last_accel[2]
    imu_msg.angular_velocity.x = last_gyro[0]
    imu_msg.angular_velocity.y = last_gyro[1]
    imu_msg.angular_velocity.z = last_gyro[2]
    imu_msg.orientation_covariance[0] = -1 
    imu_pub.publish(imu_msg)
    
    last_accel = None
    last_gyro = None
    
def on_error(ws, error):
    rospy.logerr(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    rospy.loginfo(f"### WebSocket closed ### Status: {close_status_code}, Msg: {close_msg}")

def on_open(ws):
    """
    Колбэк, вызываемый при успешном открытии соединения.
    Теперь ему не нужно ничего отправлять. Вся конфигурация была в URL.
    """
    rospy.loginfo("Connection successful. Waiting for data...")

if __name__ == "__main__":
    try:
        rospy.init_node('sensor_server_node', anonymous=True)
        
        imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

        phone_ip = rospy.get_param('~phone_ip', '10.182.178.244')
        ws_port = int(rospy.get_param('~ws_port', 8081))
        
        # --- ПОСТРОЕНИЕ ПРАВИЛЬНОГО URL СОГЛАСНО README.md ---
        
        # 1. Список сенсоров, которые мы хотим использовать.
        #    Имена берем из документации: android.sensor.*
        sensor_list = [
            "android.sensor.accelerometer",
            "android.sensor.gyroscope",
            "android.sensor.magnetic_field"
        ]

        # 2. Преобразуем список в строку формата JSON.
        #    Результат: '["android.sensor.accelerometer", ...]'
        sensor_list_json = json.dumps(sensor_list)

        # 3. Кодируем JSON-строку для безопасной передачи в URL.
        #    Это заменит кавычки и пробелы на %22, %20 и т.д.
        encoded_sensor_list = urllib.parse.quote(sensor_list_json)

        # 4. Собираем финальный URL.
        ws_address = f"ws://{phone_ip}:{ws_port}/sensors/connect?types={encoded_sensor_list}"
        
        rospy.loginfo(f"Connecting to the documented multi-sensor URL:")
        rospy.loginfo(ws_address)

        # --- Настройка и запуск WebSocket клиента ---
        ws_app = websocket.WebSocketApp(ws_address,
                                      on_open=on_open,
                                      on_message=on_message,
                                      on_error=on_error,
                                      on_close=on_close)
        
        ws_app.run_forever(ping_interval=10, ping_timeout=5)

    except rospy.ROSInterruptException:
        rospy.loginfo("Sensor node shut down.")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred: {e}")