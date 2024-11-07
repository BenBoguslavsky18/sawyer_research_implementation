#! /usr/bin/env python
 
"""
For motion of sawyer
"""

import rospy
from std_msgs.msg import Empty
import intera_interface
from intera_interface import CHECK_VERSION

from geometry_msgs.msg import Pose

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
 

# ---------------------
# Important global vars
# ---------------------
starting_x = -0.3
starting_y = -0.7


class Teleop(object):
 
    # initialization method
    def __init__(self, limb = "right"):
        # -------------------------------------
        # Saywer setup
        # -------------------------------------
        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
 
        # create right limb instance
        self._limb = intera_interface.Limb(limb)
        self._arm_joints = self._limb.joint_names()
 
        # initialize parameters
        self._start_angles = dict()
 
        # create cuff disable publisher / disabling cuff
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
 
        # verify robot is enabled
        print("Getting robot state...")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot...\n")
        self._rs.enable()
 
        # Setup IK for follower arm (Sawyer)
        # chain for follower arm
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        
        # Create a KDL joint array for follower arm
        self.num_joints = self.sawyerchain.getNrOfJoints()
        # Create the IK solver
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
 
        # Create an empty joint array to store the IK solution
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
 
        print("Running. Ctrl-c to quit")
 
 
    #extract the kinematic chain between base link and end effector
    def load_chain(self, filename, base_link, end_effector_link):
        # Load the URDF from a file
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
            
        # Load the URDF from the string
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
 
        # Ensure the URDF was successfully parsed
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
            
        # Extract the kinematic chain from base to end-effector
        return tree.getChain(base_link, end_effector_link)
 
 
    #do IK for given coordinates
    def _solve_ik(self, x, y, z, orientation): 
        # Perform IK on Sawyer
        # Convert position and orientation to PyKDL.Vector and PyKDL.Rotation
        position_vector = kdl.Vector(x, y, z)
        orientation_rotation = kdl.Rotation(orientation[0, 0], orientation[0, 1], orientation[0, 2],
                                            orientation[1, 0], orientation[1, 1], orientation[1, 2],
                                            orientation[2, 0], orientation[2, 1], orientation[2, 2])
    
 
        # Set the desired end-effector frame for IK
        target_frame = kdl.Frame(orientation_rotation, position_vector)
        # Perform inverse kinematics
        result = self.ik_solver.CartToJnt(self.ik_joint_positions_prev, target_frame, self.ik_joint_positions) #guess-start location, target, stored result
 
        for i in range(self.num_joints):
            self.ik_joint_positions_prev[i] = self.ik_joint_positions[i]
 
        return result
 
 
    #attempt to set robot joint positions based on result of IK
    def _map_poses(self, x, y, z, orientation):
        result = self._solve_ik(x, y, z, orientation)
        if result >= 0:
            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            i = 0
            for j in self._arm_joints:
                cur_pos[j] = self.ik_joint_positions[i]
                i = i+1
            self._limb.set_joint_positions(cur_pos)
        else:
            print("[ERROR] Could not solve IK")
 

    #moves the limb to neutral location
    def move_to_neutral(self):
        self._limb.move_to_neutral()
        cur_pos = self._limb.joint_angles()
        i = 0
        for j in self._arm_joints:
            self.ik_joint_positions_prev[i] = cur_pos[j]
            i = i+1
 
 
    def run_teleop(self, x_coordinate, y_coordinate):
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()
 
        # set control rate
        control_rate = rospy.Rate(self._rate)
 
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
 
        print("Done, ready to move.\n")

        # read the current end effector pose
        cur_ee_pose = self._limb.endpoint_pose()
        # x = -0.3
        # y = -0.7
        x = x_coordinate
        y = y_coordinate
        z = 0.2 # KEEP CONSTANT
        orientation = kdl.Rotation.Quaternion(cur_ee_pose["orientation"].x, cur_ee_pose["orientation"].y, cur_ee_pose["orientation"].z, cur_ee_pose["orientation"].w)
 
        # loop at specified rate commanding new joint torques
        # while not rospy.is_shutdown():
        while not (x - 0.001) < cur_ee_pose["position"].x < (x + 0.001) and not (y - 0.001) < cur_ee_pose["position"].y < (y + 0.001) and not (z - 0.001) < cur_ee_pose["position"].z < (z + 0.001):
            if not self._rs.state().enabled:
                rospy.logerr("Running the robot")
                break
            self._map_poses(x, y, z, orientation)
            cur_ee_pose = self._limb.endpoint_pose()
            print(cur_ee_pose["position"].x)
            control_rate.sleep()
 
 
    #exit cleanly
    def clean_shutdown(self):
        print("\nExiting example...")
        self._limb.exit_control_mode()
 
 
def startup():
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("sawyer_arm_teleop")
    teleop = Teleop()
    # register shutdown callback
    rospy.on_shutdown(teleop.clean_shutdown)
    teleop.move_to_neutral()
    
    # [Yue] print out the current end effector pose
    # print(teleop._limb.endpoint_pose())

    # [Yue] You need to call the run_teleop to run the control loop, otherwise it is doing nothing
    # teleop.run_teleop(-0.3, -0.7)
    teleop.run_teleop(starting_x, starting_y)






 
if __name__ == "__main__":
    startup()