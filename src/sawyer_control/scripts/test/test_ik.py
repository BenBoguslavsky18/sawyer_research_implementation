#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb
from std_msgs.msg import Empty

import intera_interface
from intera_interface import CHECK_VERSION

# ---------------------
# Important global vars
# ---------------------
starting_x = -0.4
starting_y = -0.8

teleop = None  # responsible for movement

class Teleop(object):
 
    # initialization method
    def __init__(self, limb="right"):
        # -------------------------------------
        # Sawyer setup
        # -------------------------------------
        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
 
        # create right limb instance
        self._limb = Limb(limb)
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

        # Initialize trajectory interface
        self._traj = MotionTrajectory(limb=self._limb)
        self._wpt_opts = MotionWaypointOptions(max_linear_speed=0.3, 
                                               max_linear_accel=0.3,
                                                max_rotational_speed=1.0, 
                                                max_rotational_accel=1.0)
        
        self._waypoint = MotionWaypoint(options=self._wpt_opts.to_msg(), limb=self._limb)
 
        print("Running. Ctrl-c to quit")

    def move_to_neutral(self):
        self._limb.move_to_neutral()
        
    def move_to_position(self, x, y, z, orientation):
        # Create PoseStamped message for target position
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        
        # Set the Cartesian pose (Impedance Control will handle the rest)
        self._waypoint.set_cartesian_pose(pose, "right_hand")
        self._traj.append_waypoint(self._waypoint.to_msg())

        # Send the trajectory to the robot
        result = self._traj.send_trajectory()
        if result is None:
            rospy.logerr("Trajectory FAILED to send")
        elif result.result:
            rospy.loginfo("Trajectory successfully executed!")
        else:
            rospy.logerr("Trajectory execution failed with error %s", result.errorId)

    def run_teleop(self, x_coordinate, y_coordinate):
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()
 
        # set control rate
        control_rate = rospy.Rate(self._rate)
 
        print("Done, ready to move.\n")

        # read the current end effector pose
        cur_ee_pose = self._limb.endpoint_pose()
        
        x = x_coordinate
        y = y_coordinate
        z = 0.3  # KEEP CONSTANT
        orientation = [
            cur_ee_pose["orientation"].x,
            cur_ee_pose["orientation"].y,
            cur_ee_pose["orientation"].z,
            cur_ee_pose["orientation"].w
        ]
        
        # Move to the desired Cartesian position using impedance control
        self.move_to_position(x, y, z, orientation)

        # Sleep to allow robot to reach position
        control_rate.sleep()

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

    # Run teleop with starting coordinates
    teleop.run_teleop(starting_x, starting_y)

if __name__ == "__main__":
    startup()
