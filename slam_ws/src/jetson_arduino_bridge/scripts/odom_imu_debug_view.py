#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

last_raw = None
last_parsed = None
last_odom = None
last_imu = None
last_mag = None

def raw_cb(msg):
    global last_raw
    last_raw = msg.data

def parsed_cb(msg):
    global last_parsed
    last_parsed = msg.data

def odom_cb(msg):
    global last_odom
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    last_odom = {
        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "th": yaw,
        "vx": msg.twist.twist.linear.x,
        "wz": msg.twist.twist.angular.z,
    }

def imu_cb(msg):
    global last_imu
    last_imu = {
        "gz": msg.angular_velocity.z,
        "ax": msg.linear_acceleration.x,
        "ay": msg.linear_acceleration.y,
        "az": msg.linear_acceleration.z,
    }

def mag_cb(msg):
    global last_mag
    last_mag = {
        "mx": msg.magnetic_field.x,
        "my": msg.magnetic_field.y,
        "mz": msg.magnetic_field.z,
    }

def main():
    rospy.init_node("odom_imu_debug_view")

    rospy.Subscriber("/arduino/raw_odom", String, raw_cb, queue_size=20)
    rospy.Subscriber("/arduino/parsed_odom", String, parsed_cb, queue_size=20)
    rospy.Subscriber("/wheel_odom", Odometry, odom_cb, queue_size=20)
    rospy.Subscriber("/imu/data", Imu, imu_cb, queue_size=20)
    rospy.Subscriber("/imu/mag", MagneticField, mag_cb, queue_size=20)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        if last_raw:
            rospy.loginfo("RAW  | %s", last_raw)

        if last_parsed:
            rospy.loginfo("ARD  | %s", last_parsed)

        if last_odom and last_imu and last_mag:
            rospy.loginfo(
                "PROC | x=%+.3f y=%+.3f th=%+.3f vx=%+.3f wz=%+.3f | "
                "gz=%+.3f | mx=%+.6f my=%+.6f mz=%+.6f | "
                "ax=%+.3f ay=%+.3f az=%+.3f",
                last_odom["x"], last_odom["y"], last_odom["th"],
                last_odom["vx"], last_odom["wz"],
                last_imu["gz"],
                last_mag["mx"], last_mag["my"], last_mag["mz"],
                last_imu["ax"], last_imu["ay"], last_imu["az"]
            )

        rate.sleep()

if __name__ == "__main__":
    main()

