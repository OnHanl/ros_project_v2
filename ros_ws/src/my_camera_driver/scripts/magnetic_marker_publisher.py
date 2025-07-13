#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MagneticMarkerPublisher:
    def __init__(self):
        rospy.init_node('magnetic_marker_publisher')
        
        self.threshold = rospy.get_param('~threshold', 60.0)  # микротесла
        self.marker_pub = rospy.Publisher('magnetic_markers', Marker, queue_size=10)
        self.sub = rospy.Subscriber('/magnetometer/data', MagneticField, self.callback)
        
        self.marker_id = 0
        self.frame_id = "map"

    def callback(self, msg):
        # Рассчитываем норму магнитного поля
        magnitude = (msg.magnetic_field.x**2 + msg.magnetic_field.y**2 + msg.magnetic_field.z**2)**0.5 * 1e6  # μT
        rospy.loginfo(f"Mag field: {magnitude:.2f} μT")

        if magnitude > self.threshold:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "magnetic_anomalies"
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(self.marker_id * 0.1, 0, 0)  # просто визуально смещаем точки по x
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)
            self.marker_id += 1

if __name__ == '__main__':
    try:
        MagneticMarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
