#!/usr/bin/env python

import rospy
from intera_core_msgs.msg import InteractionControlCommand, EndpointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray, Bool, Empty
import numpy as np


class CartesianImpedanceController:
    def __init__(self):
        rospy.init_node('cartesian_impedance_controller')

        # Publisher for interaction control
        self.pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                    InteractionControlCommand, queue_size=10)

        # Publisher for collision avoidance suppression
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                      Empty, queue_size=10)
        
        self.start_time = rospy.Time.now()

        # Default and high stiffness settings
        self.default_stiffness = [500.0, 500.0, 500.0, 30.0, 30.0, 30.0]
        self.high_stiffness = [1200.0, 1200.0, 1200.0, 30.0, 30.0, 30.0]

        # State variables
        self.error_waypoint = None
        self.stiffness_triggered = False
        self.trajectory_active = False
        # Add state variables
        self.state = "IDLE"  # Possible states: IDLE, WAITING_FOR_STIFFNESS_UPDATE, STIFFNESS_UPDATED
        self.last_update_time = rospy.Time.now()

        # Subscribe to topics
        rospy.Subscriber('/error_waypoint', Float64MultiArray, self.error_waypoint_callback)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_state_callback)
        rospy.Subscriber('/trajectory_progress', Bool, self.trajectory_progress_callback)

        # Register a shutdown handler
        rospy.on_shutdown(self.shutdown_handler)

    def error_waypoint_callback(self, msg):
        """
        Store the error waypoint position.
        """
        self.error_waypoint = msg.data
        rospy.loginfo("Received error waypoint: %s", self.error_waypoint)

    def endpoint_state_callback(self, msg):
        """
        Monitor robot's position and control stiffness based on real-time feedback.
        """
        if self.error_waypoint is None:
            return  # No error waypoint available

        # Ignore small deviations during initial execution
        elapsed_time_since_start = rospy.Time.now() - self.start_time
        if elapsed_time_since_start.to_sec() < 3.0:  # Allow 3 seconds for stabilization
            rospy.loginfo("Ignoring deviations during stabilization period.")
            return

        current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        distance = np.linalg.norm(np.array(self.error_waypoint) - np.array(current_position))
        velocity = np.linalg.norm([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

        # Monitor the robot's proximity to the error waypoint
        if self.state == "IDLE" and distance < 0.02 and velocity < 0.05:
            rospy.loginfo("Robot nearing error waypoint. Increasing stiffness.")
            self.state = "APPROACHING_ERROR"
            self.last_update_time = rospy.Time.now()

            # Publish high stiffness
            high_stiffness_msg = self.create_interaction_msg(self.high_stiffness)
            self.pub.publish(high_stiffness_msg)

        elif self.state == "APPROACHING_ERROR":
            elapsed_time = rospy.Time.now() - self.last_update_time
            if elapsed_time.to_sec() > 2.0:  # Wait for stiffness to stabilize
                rospy.loginfo("Stiffness increased. Monitoring for deviation.")
                self.state = "STIFFNESS_UPDATED"

        elif self.state == "STIFFNESS_UPDATED" and distance > 0.03:  # Example condition for deviation
            rospy.loginfo("Robot deviated. Resetting stiffness.")
            self.state = "RESET_STIFFNESS"

            # Publish default stiffness
            default_stiffness_msg = self.create_interaction_msg(self.default_stiffness)
            self.pub.publish(default_stiffness_msg)

        elif self.state == "RESET_STIFFNESS":
            rospy.loginfo("Stiffness reset. Returning to normal operation.")
            self.state = "IDLE"


    def trajectory_progress_callback(self, msg):
        """
        Update trajectory progress status.
        """
        self.trajectory_active = msg.data
        if self.trajectory_active:
            rospy.loginfo("Trajectory execution started.")
        else:
            rospy.loginfo("Trajectory execution completed.")

    def suppress_collision_avoidance(self):
        """
        Continuously suppress collision avoidance by publishing empty messages.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.collision_suppress_pub.publish(Empty())
            rate.sleep()

    def shutdown_handler(self):
        """
        Switch back to position control mode during shutdown.
        """
        rospy.loginfo("Shutting down. Switching back to position control mode...")
        position_control_msg = self.create_position_control_msg()
        for _ in range(5):  # Publish multiple times to ensure the mode switch is registered
            self.pub.publish(position_control_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Robot switched to position control mode.")

    def create_interaction_msg(self, stiffness):
        """
        Creates and configures an InteractionControlCommand message with the given stiffness.
        """
        from intera_motion_interface import InteractionOptions

        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_K_impedance(stiffness)  # Cartesian stiffness
        interaction_options.set_max_impedance([False, False, False, False, False, False])  # Not maximizing stiffness
        interaction_options.set_interaction_control_mode([1, 1, 1, 1, 1, 1])  # Impedance mode for all axes
        interaction_options.set_in_endpoint_frame(False)  # Base frame as reference
        interaction_options.set_force_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No external force commands
        interaction_options.set_K_nullspace([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # Nullspace stiffness
        interaction_options.set_disable_damping_in_force_control(False)  # Enable damping
        interaction_options.set_disable_reference_resetting(False)  # Allow smooth transitions

        return interaction_options.to_msg()

    def create_position_control_msg(self):
        """
        Creates a message to switch the robot back to position control mode.
        """
        msg = InteractionControlCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        msg.interaction_control_active = False  # Disable interaction control
        rospy.loginfo("Created message to switch back to position control mode.")
        return msg

    def publish_control(self):
        """
        Continuously publish control settings.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.state == "IDLE":
                # Publish default stiffness
                msg = self.create_interaction_msg(self.default_stiffness)
                self.pub.publish(msg)
            rate.sleep()

    def run(self):
        """
        Main loop for the controller.
        """
        rospy.loginfo("Running Cartesian Impedance Controller.")

        # Start collision avoidance suppression in a separate thread
        import threading
        collision_thread = threading.Thread(target=self.suppress_collision_avoidance)
        collision_thread.daemon = True  # Ensure thread exits with the main program
        collision_thread.start()

        self.publish_control()


if __name__ == '__main__':
    try:
        controller = CartesianImpedanceController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")
