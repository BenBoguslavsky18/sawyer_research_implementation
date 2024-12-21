#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import InteractionOptions, InteractionPublisher
from std_msgs.msg import Bool, Empty, Float64MultiArray  # For receiving stiffness updates and error states

def setup_cartesian_impedance_control(stiffness, damping, max_stiffness, interaction_frame, control_mode, nullspace_stiffness):
    interaction_options = InteractionOptions()

    # Activate interaction control
    interaction_options.set_interaction_control_active(True)
    interaction_options.set_K_impedance(stiffness)
    interaction_options.set_K_nullspace(nullspace_stiffness)
    interaction_options.set_max_impedance(max_stiffness)
    interaction_options.set_interaction_control_mode(control_mode)
    interaction_options.set_in_endpoint_frame(True)
    interaction_options.set_interaction_frame(interaction_frame)
    interaction_options.set_disable_damping_in_force_control(False)
    interaction_options.set_disable_reference_resetting(False)
    interaction_options.set_rotations_for_constrained_zeroG(False)

    msg = interaction_options.to_msg()
    msg.D_impedance = damping

    return msg

def shutdown_handler(ic_pub):
    """
    Handler to switch back to position control mode upon shutdown.
    """
    rospy.loginfo("Shutting down and switching to position control mode.")
    ic_pub.send_position_mode_cmd()

class CartesianImpedanceController:
    def __init__(self):
        rospy.init_node('cartesian_impedance_controller')

        # Default stiffness
        self.default_stiffness = [1000.0, 20.0, 20.0, 30.0, 30.0, 30.0]
        self.current_stiffness = self.default_stiffness

        # Initialize other parameters
        self.damping = rospy.get_param("~damping", [8.0, 8.0, 8.0, 2.0, 2.0, 2.0])
        self.nullspace_stiffness = rospy.get_param("~nullspace_stiffness", [10.0] * 7)
        self.max_stiffness = rospy.get_param("~max_stiffness", [False, False, False, False, False, False])
        self.control_mode = rospy.get_param("~control_mode", [1, 1, 1, 1, 1, 1])

        # Define a neutral interaction frame (identity matrix)
        self.interaction_frame = Pose()
        self.interaction_frame.position.x = 0.0
        self.interaction_frame.position.y = 0.0
        self.interaction_frame.position.z = 0.0
        self.interaction_frame.orientation.w = 1.0
        self.interaction_frame.orientation.x = 0.0
        self.interaction_frame.orientation.y = 0.0
        self.interaction_frame.orientation.z = 0.0

        # Initialize the interaction publisher
        self.ic_pub = InteractionPublisher()

        # Initialize the collision avoidance suppression publisher
        self.collision_suppress_pub = rospy.Publisher(
            '/robot/limb/right/suppress_collision_avoidance', Empty, queue_size=10
        )

        # Subscribe to stiffness updates from the trajectory script
        rospy.Subscriber('/stiffness_update', Float64MultiArray, self.stiffness_update_callback)

        # Register the shutdown handler
        rospy.on_shutdown(lambda: shutdown_handler(self.ic_pub))

    def stiffness_update_callback(self, msg):
        """
        Callback to handle stiffness updates from the trajectory script.
        """
        rospy.loginfo("Received stiffness update: %s", msg.data)  # Log received stiffness
        self.current_stiffness = msg.data

    def publish_control(self):
        """
        Continuously publish Cartesian Impedance Control settings and suppress collision avoidance.
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                # Generate the interaction message
                interaction_msg = setup_cartesian_impedance_control(
                    self.current_stiffness,
                    self.damping,
                    self.max_stiffness,
                    self.interaction_frame,
                    self.control_mode,
                    self.nullspace_stiffness,
                )
                rospy.loginfo("Publishing stiffness: %s", self.current_stiffness)

                # Publish the interaction command
                self.ic_pub.send_command(interaction_msg, 10)

                # Publish an empty message to suppress collision avoidance
                self.collision_suppress_pub.publish(Empty())
            except Exception as e:
                rospy.logwarn("Failed to publish stiffness or suppress collision avoidance: %s", e)

            rate.sleep()

    def run(self):
        """
        Main loop for the controller.
        """
        self.publish_control()

if __name__ == '__main__':
    try:
        controller = CartesianImpedanceController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt detected, exiting...")
