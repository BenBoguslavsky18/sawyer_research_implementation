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
        self.default_stiffness = [500.0, 500.0, 500.0, 30.0, 30.0, 30.0]
        self.high_stiffness = [600.0, 1000.0, 1000.0, 30.0, 30.0, 30.0]

        # Subscribe to trajectory points, error waypoint, and real-time endpoint state
        self.trajectory_points = []
        self.error_waypoint = None
        rospy.Subscriber('/trajectory_points', Float64MultiArray, self.trajectory_callback)
        rospy.Subscriber('/error_waypoint', Float64MultiArray, self.error_waypoint_callback)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_state_callback)

        # Register a shutdown handler
        rospy.on_shutdown(self.shutdown_handler)

    def trajectory_callback(self, msg):
        """
        Store the trajectory points.
        """
        num_points = len(msg.data) // 3
        self.trajectory_points = [
            (msg.data[i], msg.data[i + num_points], msg.data[i + 2 * num_points])
            for i in range(num_points)
        ]
        rospy.loginfo("Received trajectory points.")

    def error_waypoint_callback(self, msg):
        """
        Store the error waypoint position.
        """
        self.error_waypoint = msg.data
        rospy.loginfo("Received error waypoint: %s", self.error_waypoint)

    def adjust_stiffness(self, current_position):
        """
        Adjust stiffness dynamically based on proximity to the error waypoint.
        """
        if not self.trajectory_points or self.error_waypoint is None:
            return  # No trajectory or error waypoint available

        error_index = 25  # Index of the error waypoint
        trigger_distance = 0.1  # Distance to start increasing stiffness

        # Iterate through trajectory points to adjust stiffness based on proximity
        for i, point in enumerate(self.trajectory_points):
            trajectory_position = np.array(point)
            distance = np.linalg.norm(trajectory_position - np.array(current_position))

            # Gradually increase stiffness as the robot approaches the error waypoint
            if distance < trigger_distance and i <= error_index:
                scale = (trigger_distance - distance) / trigger_distance
                stiffness = [
                    self.default_stiffness[j] + scale * (self.high_stiffness[j] - self.default_stiffness[j])
                    for j in range(6)
                ]
                rospy.loginfo("Adjusting stiffness dynamically: %s", stiffness)
                stiffness_msg = create_interaction_msg(stiffness)
                self.pub.publish(stiffness_msg)

            # Pause at error waypoint and maintain high stiffness
            if i == error_index and distance < 0.03:
                rospy.loginfo("Pausing at error waypoint...")
                high_stiffness_msg = create_interaction_msg(self.high_stiffness)
                self.pub.publish(high_stiffness_msg)
                rospy.sleep(5)  # Pause for 5 seconds
                break

    def endpoint_state_callback(self, msg):
        """
        Monitor the robot's real-time position and adjust stiffness dynamically.
        """
        # Get the robot's current position
        current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.adjust_stiffness(current_position)

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
