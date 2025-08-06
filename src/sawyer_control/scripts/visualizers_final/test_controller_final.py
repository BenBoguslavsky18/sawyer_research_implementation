#!/usr/bin/env python

import rospy
from intera_core_msgs.msg import InteractionControlCommand, EndpointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray, Empty
import numpy as np

def create_interaction_msg(stiffness):
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

def create_position_control_msg():
    """
    Creates a message to switch the robot back to position control mode.
    """
    msg = InteractionControlCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base"
    msg.interaction_control_active = False  # Disable interaction control
    rospy.loginfo("Created message to switch back to position control mode.")
    return msg

class CartesianImpedanceController:
    def __init__(self):
        rospy.init_node('cartesian_impedance_controller')

        # Publisher for interaction control
        self.pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                    InteractionControlCommand, queue_size=10)

        # Publisher for collision avoidance suppression
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                      Empty, queue_size=10)

        # Default and high stiffness settings
        self.default_stiffness = [400.0, 400.0, 400.0, 27.0, 27.0, 27.0]
        self.high_stiffness = [1200.0, 1200.0, 1200.0, 30.0, 30.0, 30.0]

        # Subscribe to error waypoint and real-time endpoint state
        self.error_waypoint = None
        self.stiffness_triggered = False
        rospy.Subscriber('/error_waypoint', Float64MultiArray, self.error_waypoint_callback)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_state_callback)

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
        Monitor the robot's real-time y-position and trigger stiffness update.
        """
        if self.error_waypoint is None or self.stiffness_triggered:
            return  # No error waypoint set yet or stiffness already triggered

        # Extract the y-position of the error waypoint
        error_y_position = self.error_waypoint[1]
        current_y_position = msg.pose.position.y

        # Define a threshold buffer to trigger stiffness increase slightly earlier
        trigger_buffer = 0.04
        if error_y_position - trigger_buffer <= current_y_position <= error_y_position:
            rospy.loginfo("Approaching y-position of the error waypoint. Increasing stiffness.")
            self.stiffness_triggered = True  # Prevent repeated triggering

            # Publish high stiffness
            high_stiffness_msg = create_interaction_msg(self.high_stiffness)
            self.pub.publish(high_stiffness_msg)

            # Wait for 5 seconds before reverting stiffness
            rospy.sleep(4)

            rospy.loginfo("Reverting to default stiffness.")
            default_stiffness_msg = create_interaction_msg(self.default_stiffness)
            self.pub.publish(default_stiffness_msg)

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
        position_control_msg = create_position_control_msg()
        for _ in range(5):  # Publish multiple times to ensure the mode switch is registered
            self.pub.publish(position_control_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Robot switched to position control mode.")

    def publish_control(self):
        """
        Continuously publish Cartesian Impedance Control settings.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.stiffness_triggered:
                # Publish default stiffness if no override is active
                msg = create_interaction_msg(self.default_stiffness)
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
