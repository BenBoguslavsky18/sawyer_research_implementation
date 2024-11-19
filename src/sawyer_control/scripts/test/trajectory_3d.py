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

    # Define waypoint options
    wpt_opts = MotionWaypointOptions(max_linear_speed=0.3,  # Set suitable speed
                                     max_linear_accel=0.3,
                                     max_rotational_speed=1.0,
                                     max_rotational_accel=1.0,
                                     max_joint_speed_ratio=1.0)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

    # Define two main Cartesian positions for back-and-forth movement
    start_point = {"position": [0.5, 0.0, 0.2], "orientation": [0.0, 1.0, 0.0, 0.0]}
    end_point = {"position": [0.8, -0.3, 0.4], "orientation": [0.0, 1.0, 0.0, 0.0]}

    # Loop to create back-and-forth movement
    for i in range(3):  # Repeat at least 3 times
        rospy.loginfo(f"Starting back-and-forth movement cycle {i + 1}")

        # Append the start point
        pose = PoseStamped()
        pose.pose.position.x = start_point["position"][0]
        pose.pose.position.y = start_point["position"][1]
        pose.pose.position.z = start_point["position"][2]
        pose.pose.orientation.x = start_point["orientation"][0]
        pose.pose.orientation.y = start_point["orientation"][1]
        pose.pose.orientation.z = start_point["orientation"][2]
        pose.pose.orientation.w = start_point["orientation"][3]

        waypoint.set_cartesian_pose(pose, "right_hand")
        traj.append_waypoint(waypoint.to_msg())

        # Append the end point
        pose.pose.position.x = end_point["position"][0]
        pose.pose.position.y = end_point["position"][1]
        pose.pose.position.z = end_point["position"][2]
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
