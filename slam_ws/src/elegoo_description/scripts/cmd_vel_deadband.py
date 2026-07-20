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

        # Pulsed in-place turn: duty-cycle rotation so the lidar/SLAM can
        # integrate clean scans between bursts. Reduces map smearing during
        # explore_lite / move_base in-place turns.
        self.pulsed_turn = bool(rospy.get_param("~pulsed_turn", False))
        self.turn_on_duration = float(
            rospy.get_param("~turn_on_duration", 0.5)
        )
        self.turn_off_duration = float(
            rospy.get_param("~turn_off_duration", 0.5)
        )

        # Output loop (only used when pulsed_turn is enabled). Decouples the
        # output rate from the planner's controller_frequency so the duty cycle
        # stays smooth even when move_base runs slowly (e.g. 3.5 Hz).
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.6))

        self.last_input = Twist()
        self.last_input_time = rospy.Time(0)

        # Pulsed-turn phase state ("on" = turn burst, "off" = pause).
        self.turn_phase = "off"
        self.turn_phase_start = None

        # Settle before turn: when transitioning from driving into an in-place
        # turn, stop for settle_duration so the camera/SLAM can settle before
        # the robot starts rotating. Only effective when pulsed_turn is on.
        self.settle_before_turn = bool(
            rospy.get_param("~settle_before_turn", False)
        )
        self.settle_duration = float(
            rospy.get_param("~settle_duration", 1.5)
        )
        self.motion_state = "idle"
        self.settle_start = None

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

        if self.pulsed_turn:
            rospy.loginfo(
                "pulsed turn ENABLED: on=%.2fs off=%.2fs @ %.1f Hz "
                "(cmd_timeout=%.2fs)",
                self.turn_on_duration,
                self.turn_off_duration,
                self.rate,
                self.cmd_timeout
            )
            self.timer = rospy.Timer(
                rospy.Duration(1.0 / self.rate), self.timer_cb
            )
            if self.settle_before_turn:
                rospy.loginfo(
                    "settle-before-turn ENABLED: %.2fs stop before turning",
                    self.settle_duration
                )
        else:
            rospy.loginfo("pulsed turn disabled (continuous output)")
            if self.settle_before_turn:
                rospy.logwarn(
                    "settle_before_turn is set but pulsed_turn is off; "
                    "settle has no effect without the output timer."
                )

    def cmd_cb(self, msg):
        if not self.pulsed_turn:
            # Original behaviour: remap and publish on every incoming command.
            self.pub.publish(self.remap(msg))
            return

        # Pulsed mode: hand the latest command to the output timer so the duty
        # cycle is driven at a fixed rate instead of the planner's rate.
        self.last_input = msg
        self.last_input_time = rospy.Time.now()

    def timer_cb(self, _event):
        now = rospy.Time.now()

        # Planner went silent (or never spoke): stop and reset all state.
        if (now - self.last_input_time).to_sec() > self.cmd_timeout:
            self.pub.publish(Twist())
            self.reset_pulse()
            self.motion_state = "idle"
            self.settle_start = None
            return

        out = self.remap(self.last_input)

        driving = abs(self.last_input.linear.x) > self.driving_x_threshold
        turning = (not driving) and abs(out.angular.z) > 0.0
        if driving:
            intended = "driving"
        elif turning:
            intended = "turning"
        else:
            intended = "idle"

        # Drive the motion-state machine. The only special transition is
        # driving -> turning: optionally stop and let the camera settle first.
        if self.motion_state == "driving":
            if intended == "turning":
                if self.settle_before_turn:
                    self.motion_state = "settling"
                    self.settle_start = now
                else:
                    self.motion_state = "turning"
                    self.reset_pulse()
            elif intended == "idle":
                self.motion_state = "idle"
            # intended == "driving": stay driving
        elif self.motion_state == "settling":
            if intended == "driving":
                self.motion_state = "driving"
                self.settle_start = None
            elif intended == "idle":
                self.motion_state = "idle"
                self.settle_start = None
            elif (now - self.settle_start).to_sec() >= self.settle_duration:
                self.motion_state = "turning"
                self.reset_pulse()
                self.settle_start = None
            # else keep settling (turn still intended)
        elif self.motion_state == "turning":
            if intended != "turning":
                self.motion_state = intended
                self.reset_pulse()
            # intended == "turning": keep pulsing
        else:  # idle
            if intended == "turning":
                self.motion_state = "turning"
                self.reset_pulse()
            elif intended == "driving":
                self.motion_state = "driving"
            # intended == "idle": stay idle

        # Emit output for the current state.
        if self.motion_state == "settling":
            self.pub.publish(Twist())
            return
        if self.motion_state == "turning":
            out.angular.z = self.pulse_turn(out.angular.z, now)
            self.pub.publish(out)
            return
        # driving or idle: pass the remapped command through (idle -> zeros).
        self.reset_pulse()
        self.pub.publish(out)

    def remap(self, msg):
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

        return out

    def pulse_turn(self, angular_z, now):
        # Start the first burst on demand.
        if self.turn_phase_start is None:
            self.turn_phase = "on"
            self.turn_phase_start = now

        elapsed = (now - self.turn_phase_start).to_sec()

        if self.turn_phase == "on":
            if elapsed >= self.turn_on_duration:
                self.turn_phase = "off"
                self.turn_phase_start = now
                return 0.0
            return angular_z

        # off phase: pause so SLAM can integrate a clean scan.
        if elapsed >= self.turn_off_duration:
            self.turn_phase = "on"
            self.turn_phase_start = now
            return angular_z
        return 0.0

    def reset_pulse(self):
        self.turn_phase = "off"
        self.turn_phase_start = None

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
