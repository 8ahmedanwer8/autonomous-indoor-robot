#!/usr/bin/env python
import math
import threading

import rospy
import serial
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
import tf2_ros


class ArduinoBridge(object):
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.publish_tf = rospy.get_param("~publish_tf", True)


        self.wheel_base = float(rospy.get_param("~wheel_base", 0.13))
        self.k_pwm = float(rospy.get_param("~k_pwm", 500.0))
        self.max_pwm = int(rospy.get_param("~max_pwm", 255))
        self.deadman_timeout = float(rospy.get_param("~deadman_timeout", 0.3))

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")

        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        rospy.sleep(2.0)

        self.lock = threading.Lock()
        self.last_cmd_time = rospy.Time(0)
        self.last_cmd_left = 0
        self.last_cmd_right = 0

        self.rx_buffer = ""

        self.reset_odom_state()
        self.prev_arduino_ms = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=20)
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)
        self.mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=50)

        self.raw_odom_pub = rospy.Publisher("/arduino/raw_odom", String, queue_size=50)
        self.parsed_odom_pub = rospy.Publisher("/arduino/parsed_odom", String, queue_size=50)

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.spin_once)
        self.deadman_timer = rospy.Timer(rospy.Duration(0.05), self.deadman_cb)

        rospy.loginfo("arduino_bridge started on %s @ %d", self.port, self.baud)

    def reset_odom_state(self):
        self.have_prev = False
        self.prev_left_dist = 0.0
        self.prev_right_dist = 0.0
        self.prev_theta = 0.0
        self.prev_stamp = None
        self.x = 0.0
        self.y = 0.0

    def cmd_vel_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_l = v - 0.5 * self.wheel_base * w
        v_r = v + 0.5 * self.wheel_base * w

        left_pwm = int(max(-self.max_pwm, min(self.max_pwm, round(self.k_pwm * v_l))))
        right_pwm = int(max(-self.max_pwm, min(self.max_pwm, round(self.k_pwm * v_r))))
        rospy.loginfo_throttle(0.2, "CMD | v=%.3f w=%.3f -> L=%d R=%d", v, w, left_pwm, right_pwm)

        line = "VEL,{0},{1}\n".format(left_pwm, right_pwm)
        self.write_line(line)

        with self.lock:
            self.last_cmd_time = rospy.Time.now()
            self.last_cmd_left = left_pwm
            self.last_cmd_right = right_pwm

    def deadman_cb(self, _event):
        with self.lock:
            last = self.last_cmd_time

        if last == rospy.Time(0):
            return

        if (rospy.Time.now() - last).to_sec() > self.deadman_timeout:
            self.write_line("VEL,0,0\n")
            with self.lock:
                self.last_cmd_time = rospy.Time.now()
                self.last_cmd_left = 0
                self.last_cmd_right = 0

    def write_line(self, line):
        try:
            self.ser.write(line)
        except serial.SerialException as e:
            rospy.logerr_throttle(2.0, "Serial write failed: %s", str(e))

    def spin_once(self, _event):
        try:
            data = self.ser.read(256)
        except serial.SerialException as e:
            rospy.logerr_throttle(2.0, "Serial read failed: %s", str(e))
            return

        if not data:
            return

        self.rx_buffer += data

        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            if line.startswith("ODOM,"):
                self.handle_odom_line(line)

    def handle_odom_line(self, line):
        self.raw_odom_pub.publish(line)

        # ODOM,<millis>,<leftDist>,<rightDist>,<theta_gyro>,<gyroZ>,
        #      <leftPulses>,<rightPulses>,
        #      <magX>,<magY>,<magZ>,
        #      <accelX>,<accelY>,<accelZ>,
        #      <imuReady>
        parts = line.split(",")

        if len(parts) == 15:
            try:
                arduino_ms = int(parts[1])
                left_dist = float(parts[2])
                right_dist = float(parts[3])
                theta = float(parts[4])
                gyro_z = float(parts[5])
                left_pulses = int(parts[6])
                right_pulses = int(parts[7])
                mag_x = float(parts[8])
                mag_y = float(parts[9])
                mag_z = float(parts[10])
                accel_x = float(parts[11])
                accel_y = float(parts[12])
                accel_z = float(parts[13])
                imu_ready = int(parts[14])
            except ValueError:
                rospy.logwarn_throttle(2.0, "Parse error in ODOM packet: %s", line)
                return
        elif len(parts) == 7:
            try:
                arduino_ms = int(parts[1])
                left_dist = float(parts[2])
                right_dist = float(parts[3])
                theta = float(parts[4])
                gyro_z = float(parts[5])
                imu_ready = int(parts[6])
            except ValueError:
                rospy.logwarn_throttle(2.0, "Parse error in legacy ODOM packet: %s", line)
                return

            left_pulses = 0
            right_pulses = 0
            mag_x = 0.0
            mag_y = 0.0
            mag_z = 0.0
            accel_x = 0.0
            accel_y = 0.0
            accel_z = 0.0
        else:
            rospy.logwarn_throttle(2.0, "Bad ODOM packet: %s", line)
            return

        debug_line = (
            "ms={ms} "
            "L={ld:.3f} R={rd:.3f} "
            "th={th:.3f} gz={gz:.3f} "
            "lp={lp} rp={rp} "
            "mx={mx:.6f} my={my:.6f} mz={mz:.6f} "
            "ax={ax:.3f} ay={ay:.3f} az={az:.3f} "
            "ready={ready}"
        ).format(
            ms=arduino_ms,
            ld=left_dist,
            rd=right_dist,
            th=theta,
            gz=gyro_z,
            lp=left_pulses,
            rp=right_pulses,
            mx=mag_x * 1e-6,
            my=mag_y * 1e-6,
            mz=mag_z * 1e-6,
            ax=accel_x,
            ay=accel_y,
            az=accel_z,
            ready=imu_ready
        )
        self.parsed_odom_pub.publish(debug_line)

        if self.prev_arduino_ms is not None and arduino_ms < self.prev_arduino_ms:
            rospy.logwarn("Arduino reset detected, clearing odom state")
            self.reset_odom_state()

        self.prev_arduino_ms = arduino_ms
        stamp = rospy.Time.now()

        if not self.have_prev:
            self.prev_left_dist = left_dist
            self.prev_right_dist = right_dist
            self.prev_theta = theta
            self.prev_stamp = stamp
            self.have_prev = True
            self.publish_imu(stamp, gyro_z, accel_x, accel_y, accel_z, imu_ready)
            self.publish_mag(stamp, mag_x, mag_y, mag_z, imu_ready)
            return

        dt = max((stamp - self.prev_stamp).to_sec(), 1e-6)
        d_left = left_dist - self.prev_left_dist
        d_right = right_dist - self.prev_right_dist
        ds = 0.5 * (d_left + d_right)

        dtheta = self.wrap_pi(theta - self.prev_theta)
        theta_mid = self.wrap_pi(self.prev_theta + 0.5 * dtheta)

        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)

        v = ds / dt
        w = dtheta / dt

        self.publish_odom(stamp, self.x, self.y, theta, v, w)
        self.publish_imu(stamp, gyro_z, accel_x, accel_y, accel_z, imu_ready)
        self.publish_mag(stamp, mag_x, mag_y, mag_z, imu_ready)

        self.prev_left_dist = left_dist
        self.prev_right_dist = right_dist
        self.prev_theta = theta
        self.prev_stamp = stamp

    def publish_odom(self, stamp, x, y, theta, linear_x, angular_z):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        q = quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(*q)

        msg.twist.twist.linear.x = linear_x
        msg.twist.twist.angular.z = angular_z

        msg.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.2
        ]

        msg.twist.covariance = [
            0.1,  0,    0,    0,    0,    0,
            0,    0.1,  0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.3
        ]

        self.odom_pub.publish(msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(*q)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_imu(self, stamp, gyro_z, accel_x, accel_y, accel_z, imu_ready):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame

        msg.orientation_covariance[0] = -1.0

        if imu_ready:
            msg.angular_velocity.z = gyro_z
            msg.angular_velocity_covariance = [
                1e6, 0,   0,
                0,   1e6, 0,
                0,   0,   0.02
            ]

            msg.linear_acceleration.x = accel_x
            msg.linear_acceleration.y = accel_y
            msg.linear_acceleration.z = accel_z
            msg.linear_acceleration_covariance = [
                0.2, 0,   0,
                0,   0.2, 0,
                0,   0,   0.2
            ]
        else:
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    @staticmethod
    def wrap_pi(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_mag(self, stamp, mag_x, mag_y, mag_z, imu_ready):
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame

        if imu_ready:
            msg.magnetic_field.x = mag_x * 1e-6
            msg.magnetic_field.y = mag_y * 1e-6
            msg.magnetic_field.z = mag_z * 1e-6
            msg.magnetic_field_covariance = [
                1e-6, 0,    0,
                0,    1e-6, 0,
                0,    0,    1e-6
            ]

        self.mag_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("arduino_bridge")
    ArduinoBridge()
    rospy.spin()
