#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import InteractionOptions, InteractionPublisher
from intera_motion_interface.utility_functions import int2bool

def setup_cartesian_impedance_control(stiffness, damping, max_stiffness, interaction_frame, control_mode, nullspace_stiffness):
    """
    Set up Cartesian impedance control for the Sawyer robot.
    """
    interaction_options = InteractionOptions()

    # Activate interaction control
    interaction_options.set_interaction_control_active(True)

    # Set Cartesian stiffness values
    interaction_options.set_K_impedance(stiffness)
    
     # Set nullspace stiffness
    interaction_options.set_K_nullspace(nullspace_stiffness)

    # Set max stiffness (if needed)
    interaction_options.set_max_impedance(max_stiffness)

    # Set interaction control mode (impedance)
    interaction_options.set_interaction_control_mode(control_mode)

    # Set interaction frame
    interaction_options.set_in_endpoint_frame(True)
    interaction_options.set_interaction_frame(interaction_frame)

    # Other options
    interaction_options.set_disable_damping_in_force_control(False)
    interaction_options.set_disable_reference_resetting(False)
    interaction_options.set_rotations_for_constrained_zeroG(False)

    # Generate ROS message
    msg = interaction_options.to_msg()

    # Manually add damping to the message
    msg.D_impedance = damping

    return msg

def main():
    """
    Main function to initialize the Cartesian impedance control node.
    """
    rospy.init_node('cartesian_impedance_controller')

    # Define the desired stiffness, damping, and max stiffness parameters
    stiffness = [1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]  # [N/m, Nm/rad]
    damping = [20.0, 20.0, 20.0, 5.0, 5.0, 5.0]  # [Ns/m, Nms/rad]
    nullspace_stiffness = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]  # [Nm/rad]
    max_stiffness = [True, True, True, True, True, True]
    # max_stiffness = [False, False, False, False, False, False]
    control_mode = [1, 1, 1, 1, 1, 1]  # Impedance mode for all axes

    # Define the interaction frame (identity matrix)
    interaction_frame = Pose()
    interaction_frame.position.x = 1.0
    interaction_frame.position.y = 1.0
    interaction_frame.position.z = 0.0
    interaction_frame.orientation.w = 1.0
    interaction_frame.orientation.x = 0.0
    interaction_frame.orientation.y = 0.0
    interaction_frame.orientation.z = 0.0

    # Setup interaction options and generate message
    interaction_msg = setup_cartesian_impedance_control(
        stiffness, damping, max_stiffness, interaction_frame, control_mode, nullspace_stiffness
    )

    # Initialize interaction publisher
    ic_pub = InteractionPublisher()

    # Log the configuration
    rospy.loginfo("Setting up Cartesian Impedance Control:")
    rospy.loginfo(interaction_msg)

    # Send the interaction command
    pub_rate = 10.  # Publish rate (Hz)
    rospy.on_shutdown(ic_pub.send_position_mode_cmd)
    ic_pub.send_command(interaction_msg, pub_rate)  # Add pub_rate argument

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt detected, exiting...")

