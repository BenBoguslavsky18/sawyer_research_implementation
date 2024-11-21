#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_interface import Limb, CHECK_VERSION
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
 
constant_x = 0.6

class IKMotionWaypoint:
    def __init__(self, limb="right"):
        rospy.init_node('ik_motion_waypoint')
 
        # Limb and IK solver setup
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
        
        # Setup motion trajectory
        self.traj = MotionTrajectory(limb=self._limb)
        rospy.loginfo("IK and MotionWaypoint setup complete.")
 
    def load_chain(self, filename, base_link, end_effector_link):
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
        return tree.getChain(base_link, end_effector_link)
 
    def calculate_ik(self, pose):
        # Target position and orientation for IK
        pos = pose.position
        orientation = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w)
        frame = kdl.Frame(orientation, kdl.Vector(pos.x, pos.y, pos.z))
 
        # Perform IK calculation
        result = self.ik_solver.CartToJnt(self.ik_joint_positions_prev, frame, self.ik_joint_positions)
        if result < 0:
            rospy.logerr("IK solution not found")
            return None
        for i in range(self.num_joints):
            self.ik_joint_positions_prev[i] = self.ik_joint_positions[i]
        return [self.ik_joint_positions[i] for i in range(self.num_joints)]
 
    def add_waypoint(self, pose, limb_name="right_hand"):
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Skipping waypoint due to IK failure")
            return False
 
        # Set motion waypoint options
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.3,
            max_linear_accel=0.3,
            max_rotational_speed=0.3,
            max_rotational_accel=0.3,
            max_joint_speed_ratio=0.3
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.append_waypoint(waypoint.to_msg())
        rospy.loginfo("Waypoint added at: {}".format(pose.position))
        return True
 
    def execute_trajectory(self):
        # Define two main Cartesian waypoints using proper Pose format
        waypoints = [
            Pose(position=Point(x=constant_x, y=0.2, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
            Pose(position=Point(x=constant_x, y=-0.7, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
        ]
 
        for pose in waypoints:
            self.add_waypoint(pose)
 
        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Trajectory FAILED to send")
        elif result.result:
            rospy.loginfo("Trajectory successfully executed!")
        else:
            rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
 
if __name__ == '__main__':
    ik_motion = IKMotionWaypoint()
    ik_motion.execute_trajectory()