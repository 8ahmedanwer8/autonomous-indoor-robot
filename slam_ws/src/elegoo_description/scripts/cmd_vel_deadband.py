#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Twist


class CmdVelDeadband(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        # Linear velocity mapping
        self.in_min_x = abs(float(rospy.get_param("~in_min_x", 0.04)))
        self.in_max_x = abs(float(rospy.get_param("~in_max_x", 0.15)))
        self.out_min_x = abs(float(rospy.get_param("~out_min_x", 0.10)))
        self.out_max_x = abs(float(rospy.get_param("~out_max_x", 0.15)))

        # Rotation-in-place mapping
        self.in_min_theta = abs(
            float(rospy.get_param("~in_min_theta", 0.25))
        )
        self.in_max_theta = abs(
            float(rospy.get_param("~in_max_theta", 0.90))
        )
        self.out_min_theta = abs(
            float(rospy.get_param("~out_min_theta", 5.00))
        )
        self.out_max_theta = abs(
            float(rospy.get_param("~out_max_theta", 7.50))
        )

        # Rotation-while-driving mapping
        self.in_min_theta_driving = abs(
            float(rospy.get_param("~in_min_theta_driving", 0.10))
        )
        self.in_max_theta_driving = abs(
            float(rospy.get_param("~in_max_theta_driving", 0.45))
        )
        self.out_min_theta_driving = abs(
            float(rospy.get_param("~out_min_theta_driving", 4.50))
        )
        self.out_max_theta_driving = abs(
            float(rospy.get_param("~out_max_theta_driving", 5.50))
        )

        self.x_deadband = abs(
            float(rospy.get_param("~x_deadband", 0.005))
        )
        self.theta_deadband = abs(
            float(rospy.get_param("~theta_deadband", 0.02))
        )
        self.driving_x_threshold = abs(
            float(rospy.get_param("~driving_x_threshold", 0.02))
        )

        self.pub = rospy.Publisher(
            self.output_topic,
            Twist,
            queue_size=1
        )

        self.sub = rospy.Subscriber(
            self.input_topic,
            Twist,
            self.cmd_cb,
            queue_size=1
        )

        rospy.loginfo(
            "cmd_vel_deadband: %s -> %s",
            self.input_topic,
            self.output_topic
        )

        rospy.loginfo(
            "linear: input=[%.3f, %.3f], output=[%.3f, %.3f]",
            self.in_min_x,
            self.in_max_x,
            self.out_min_x,
            self.out_max_x
        )

        rospy.loginfo(
            "rotation in place: input=[%.3f, %.3f], "
            "output=[%.3f, %.3f]",
            self.in_min_theta,
            self.in_max_theta,
            self.out_min_theta,
            self.out_max_theta
        )

        rospy.loginfo(
            "rotation driving: input=[%.3f, %.3f], "
            "output=[%.3f, %.3f]",
            self.in_min_theta_driving,
            self.in_max_theta_driving,
            self.out_min_theta_driving,
            self.out_max_theta_driving
        )

    def cmd_cb(self, msg):
        out = Twist()

        out.linear.x = self.remap_deadband(
            msg.linear.x,
            self.in_min_x,
            self.in_max_x,
            self.out_min_x,
            self.out_max_x,
            self.x_deadband
        )

        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y

        if abs(msg.linear.x) > self.driving_x_threshold:
            out.angular.z = self.remap_deadband(
                msg.angular.z,
                self.in_min_theta_driving,
                self.in_max_theta_driving,
                self.out_min_theta_driving,
                self.out_max_theta_driving,
                self.theta_deadband
            )
        else:
            out.angular.z = self.remap_deadband(
                msg.angular.z,
                self.in_min_theta,
                self.in_max_theta,
                self.out_min_theta,
                self.out_max_theta,
                self.theta_deadband
            )

        self.pub.publish(out)

    @staticmethod
    def remap_deadband(
        value,
        input_minimum,
        input_maximum,
        output_minimum,
        output_maximum,
        deadband
    ):
        # Protect the motor controller from invalid commands.
        if math.isnan(value) or math.isinf(value):
            rospy.logerr_throttle(
                1.0,
                "cmd_vel_deadband received NaN or Inf"
            )
            return 0.0

        magnitude = abs(value)

        if magnitude <= deadband:
            return 0.0

        # Any meaningful command below the planner's expected minimum
        # receives the motor's minimum usable command.
        if magnitude <= input_minimum:
            mapped = output_minimum

        elif magnitude >= input_maximum:
            mapped = output_maximum

        else:
            ratio = (
                (magnitude - input_minimum) /
                (input_maximum - input_minimum)
            )

            mapped = output_minimum + ratio * (
                output_maximum - output_minimum
            )

        return math.copysign(mapped, value)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_deadband")
    CmdVelDeadband()
    rospy.spin()
