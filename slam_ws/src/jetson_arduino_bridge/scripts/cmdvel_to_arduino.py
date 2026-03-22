#!/usr/bin/env python2
import rospy
import serial
from geometry_msgs.msg import Twist

class CmdVelToArduino(object):
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", 115200))
        self.wheel_base = float(rospy.get_param("~wheel_base", 0.20))  # meters
        self.max_pwm = int(rospy.get_param("~max_pwm", 200))           # <=255
        self.k_pwm = float(rospy.get_param("~k_pwm", 500.0))           # pwm per (m/s)
        self.timeout = float(rospy.get_param("~timeout", 0.4))         # seconds

        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.last_cmd = rospy.Time.now()

        rospy.Subscriber("/cmd_vel", Twist, self.cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.05), self.deadman)  # 20 Hz

        rospy.loginfo("[cmdvel_to_arduino] port=%s baud=%d wheel_base=%.3f max_pwm=%d k_pwm=%.1f timeout=%.2f",
                      port, baud, self.wheel_base, self.max_pwm, self.k_pwm, self.timeout)

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # differential drive
        vL = v - (w * self.wheel_base / 2.0)
        vR = v + (w * self.wheel_base / 2.0)

        L = int(round(vL * self.k_pwm))
        R = int(round(vR * self.k_pwm))

        L = self.clamp(L, -self.max_pwm, self.max_pwm)
        R = self.clamp(R, -self.max_pwm, self.max_pwm)

        line = "VEL,%d,%d\n" % (L, R)
        self.ser.write(line)  # python2: str is bytes
        self.last_cmd = rospy.Time.now()

    def deadman(self, _evt):
        if (rospy.Time.now() - self.last_cmd).to_sec() > self.timeout:
            self.ser.write("VEL,0,0\n")
            self.last_cmd = rospy.Time.now()

if __name__ == "__main__":
    rospy.init_node("cmdvel_to_arduino")
    CmdVelToArduino()
    rospy.spin()

