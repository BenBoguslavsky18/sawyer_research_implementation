#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb
 
def execute_trajectory():
    rospy.init_node('trajectory_execution')
    limb = Limb()
 
    # Set trajectory options for Cartesian interpolation
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)
 
    # Define waypoint options with suitable speed and acceleration limits
    wpt_opts = MotionWaypointOptions(max_linear_speed=0.5,
                                     max_linear_accel=0.1,
                                     max_rotational_speed=1.0,
                                     max_rotational_accel=0.1,
                                     max_joint_speed_ratio=1.0)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
 
    # Define two main Cartesian positions for back-and-forth movement in the XZ plane
    # Fixed y-axis value to constrain movement to the XZ plane
    fixed_z = 0.2
    # start_point = {"position": [-0.1, -0.7, fixed_z], "orientation": [0.0, 1.0, 0.0, 0.0]}
    # end_point = {"position": [0.6, 0.3, fixed_z], "orientation": [0.0, 1.0, 0.0, 0.0]}
    start_point = {"position": [0.0, 0.0, 0], "orientation": [0.0, 1.0, 0.0, 0.0]}
    end_point = {"position": [0.3, 0.0, 0], "orientation": [0.0, 1.0, 0.0, 0.0]}
 
    # Loop to create back-and-forth movement in the XY plane
    for i in range(2):  # Repeat at least 2 times
        rospy.loginfo(f"Starting back-and-forth movement cycle {i + 1}")
 
        # Append the start point
        pose = PoseStamped()
        pose.pose.position.x = start_point["position"][0]
        pose.pose.position.y = start_point["position"][1]
        pose.pose.position.z = fixed_z  # Fixed z-axis value for XY plane
        pose.pose.orientation.x = start_point["orientation"][0]
        pose.pose.orientation.y = start_point["orientation"][1]
        pose.pose.orientation.z = start_point["orientation"][2]
        pose.pose.orientation.w = start_point["orientation"][3]
 
        waypoint.set_cartesian_pose(pose, "right_hand")
        traj.append_waypoint(waypoint.to_msg())
 
        # Append the end point
        pose.pose.position.x = end_point["position"][0]
        pose.pose.position.y = end_point["position"][1]
        pose.pose.position.z = fixed_z  # Fixed z-axis value for XY plane
        pose.pose.orientation.x = end_point["orientation"][0]
        pose.pose.orientation.y = end_point["orientation"][1]
        pose.pose.orientation.z = end_point["orientation"][2]
        pose.pose.orientation.w = end_point["orientation"][3]
 
        waypoint.set_cartesian_pose(pose, "right_hand")
        traj.append_waypoint(waypoint.to_msg())
 
    # Send the trajectory to the robot
    result = traj.send_trajectory()
    if result is None:
        rospy.logerr("Trajectory FAILED to send")
    elif result.result:
        rospy.loginfo("Trajectory successfully executed!")
    else:
        rospy.logerr("Trajectory execution failed with error %s", result.errorId)
 
if __name__ == '__main__':
    try:
        execute_trajectory()
    except rospy.ROSInterruptException:
        pass