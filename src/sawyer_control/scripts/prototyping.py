#!/usr/bin/env python

# Small example to move the Sawyer EE position using the keyboard input

import argparse
import rospy
import intera_interface
import intera_external_devices
import tf
from intera_interface import CHECK_VERSION
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
# from sawyer_xr_teleop import ControllerData
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest,
)

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


# returns the current EE pose
def get_current_ee_pose(sawyer_limb, fksvc, ns):

    fkreq = SolvePositionFKRequest()
    print('FK REQ')
    print(fkreq)
    joints = JointState()
    print('JOINT STATE')
    print(joints)
    joints.name = sawyer_limb.joint_names()
    fkreq.tip_names.append('right_hand')

    # get current joints configuration
    for joint in joints.name:
        joints.position.append(sawyer_limb.joint_angle(joint))

    fkreq.configuration.append(joints)

    # compute FK
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = fksvc(fkreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid
    if resp.isValid[0]:
        current_pose = resp.pose_stamp[0].pose
    else:
        return False

    return current_pose


def compute_ik(desired_ee_pose, iksvc, ns):

    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    posed = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=desired_ee_pose.position.x,
                    y=desired_ee_pose.position.y,
                    z=desired_ee_pose.position.z,
                ),
                orientation=Quaternion(
                    x=desired_ee_pose.orientation.x,
                    y=desired_ee_pose.orientation.y,
                    z=desired_ee_pose.orientation.z,
                    w=desired_ee_pose.orientation.w,
                ),
            ),
        )

    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(posed)

    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')
    try:
        # rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    # Check if result valid, and type of seed ultimately used to get solution
    if resp.result_type[0] > 0:
        # rospy.loginfo("SUCCESS - Valid Joint Solution Found!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        return False

    return limb_joints


def move_ee(side):
    sawyer_arm = intera_interface.Limb(side)
    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True
        rospy.loginfo("The electric gripper is mounted on the robot.")

    # The gripper has to be calibrated before use
    if has_gripper:
        gripper.calibrate()
        if gripper.is_calibrated():
            rospy.loginfo("The gripper has been calibrated.")
        else:
            rospy.loginfo("[WARNING]: the gripper could not be calibrated correctly. Issues may occur.")

        # this returns the position of the gripper as how much it is open, in meters
        gripper_position = gripper.get_position()
        print("Gripper position: " + str(gripper_position))

    # Start FK service
    nsfk = "ExternalTools/" + side + "/PositionKinematicsNode/FKService"
    fksvc = rospy.ServiceProxy(nsfk, SolvePositionFK)

    # Start IK service
    nsik = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(nsik, SolvePositionIK)

    # Get initial EE pose
    current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc, nsfk)
    if not current_ee_pose:
        rospy.logerr("Simple FK call FAILED")
    else:
        print("Initial EE pose:\n" + str(current_ee_pose))

    # Initially set the new pose to the current pose
    new_ee_pose = current_ee_pose
    # Menu print
    print("- Press 0 to move the EE with the keyboard.")
    print("- Press 1 to move the EE to a desired position.")

    user_choice = input("Input your choice: ")

    # Move with keyboard
    if user_choice == '0':
        delta = input("Please choose a delta for the motion: ")
        delta = int(delta) #cast to allow math operations
        instructions = """
        Instructions:
         - 1: increase X
         - q: decrease X
         - 2: increase Y
         - w: decrease Y
         - 3: increase Z
         - e: decrease Z
        """
        print(instructions)

        # function to call IK and move the joints to the new obtained pose with raw position control
        def set_ee_pose(limb, ee_pose, iksvc, ns):
            new_limb_pose = compute_ik(ee_pose, iksvc, ns)
            if not new_limb_pose:
                print("Impossible to move due to failed solution.")
            else:
                limb.set_joint_positions(new_limb_pose)

        # Transform the quaternion in pose format into a quaternion format and transforms into euler angles
        def user_quaternion_to_euler(quaternion):
            new_quaternion = ( quaternion.x,
                               quaternion.y,
                               quaternion.z,
                               quaternion.w)
            return tf.transformations.euler_from_quaternion(new_quaternion)

        # Transform the euler angles into quaternion and then into pose format
        def user_euler_to_quaternion(euler, orientation):
            quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
            orientation.x = quaternion[0]
            orientation.y = quaternion[1]
            orientation.z = quaternion[2]
            orientation.w = quaternion[3]
            return orientation

        bindings = {
            '1': (set_ee_pose, "X increase"),
            'q': (set_ee_pose, "X decrease"),
            '2': (set_ee_pose, "Y increase"),
            'w': (set_ee_pose, "Y decrease"),
            '3': (set_ee_pose, "Z increase"),
            'e': (set_ee_pose, "Z decrease"),
            '4': (set_ee_pose, "X rot increase"),
            'r': (set_ee_pose, "X rot decrease"),
            '5': (set_ee_pose, "Y rot increase"),
            't': (set_ee_pose, "Y rot decrease"),
            '6': (set_ee_pose, "Z rot increase"),
            'y': (set_ee_pose, "Z rot decrease"),
        }

        done = False

        print("Controlling joints. Press ? for help, Esc to quit.")
        while not done and not rospy.is_shutdown():
            # get the keyboard input
            c = intera_external_devices.getch()
            if c:
                # catch Esc or ctrl-c
                if c in ['\x1b', '\x03']:
                    done = True
                    rospy.signal_shutdown("Terminating the program.")
                elif c in bindings:
                    cmd = bindings[c]
                    if c == '1':
                        new_ee_pose.position.x += delta
                    elif c == 'q':
                        new_ee_pose.position.x -= delta
                    if c == '2':
                        new_ee_pose.position.y += delta
                    elif c == 'w':
                        new_ee_pose.position.y -= delta
                    if c == '3':
                        new_ee_pose.position.z += delta
                    elif c == 'e':
                        new_ee_pose.position.z -= delta
                    if c == '4':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[0] += delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    elif c == 'r':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[0] -= delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    if c == '5':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[1] += delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    elif c == 't':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[1] -= delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    if c == '6':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[2] += delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    elif c == 'y':
                        euler_ang = list(user_quaternion_to_euler(new_ee_pose.orientation))
                        euler_ang[2] -= delta
                        new_ee_pose.orientation = user_euler_to_quaternion(euler_ang, new_ee_pose.orientation)
                    print(cmd[1])
                    cmd[0](sawyer_arm, new_ee_pose, iksvc, nsik)
                else:
                    print("key bindings: ")
                    print("  Esc: Quit")
                    print(instructions)

    # Move to typed desired pose
    elif user_choice == '1':
        new_ee_pose.position.x = input("New x position: ")
        new_ee_pose.position.y = input("New y position: ")
        new_ee_pose.position.z = input("New z position: ")
        new_limb_pose = compute_ik(new_ee_pose)
        if not new_limb_pose:
            rospy.logerr("Impossible to move due to failed solution.")
        else:
            print("New joint configuration:\n" + str(new_limb_pose))

        print("Moving to the new configuration")
        sawyer_arm.set_joint_positions(new_limb_pose)
        rospy.signal_shutdown("Terminating the program.")
    else:
        if (user_choice != '1') and ((user_choice != '0')):
            rospy.logerr(str(user_choice) + " is not an admissible option.")
        else:
            rospy.logerr("Nothing selected.")
        rospy.signal_shutdown("Terminating the program.")


def sawyer_shutdown():
    print("Exiting the test program. Goodbye.")


def main():
    """
    Use the keyboard to move the EE, use the arrows to change position, use other keys to change orientation.
    """
    robot_params = intera_interface.RobotParams()
    # get the name of the Sawyer arm to pass to intera sdk
    sawyer_arm_name = robot_params.get_limb_names()[0]
    # Initialize the ROS node
    print("Initializing node now...")
    rospy.init_node("sawyer_test_ee")
    rospy.Rate(100)
    # Initialize robot state
    print("Getting robot state now...")
    robot_state = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = robot_state.state().enabled

    print("Initial state: ")
    print(init_state)

    rospy.on_shutdown(sawyer_shutdown)

    # enable the robot
    print("Enabling robot now...")
    robot_state.enable()

    # call the function to move the EE
    move_ee(sawyer_arm_name)


if __name__ == '__main__':
    main()