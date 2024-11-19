#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb

def incremental_trajectory_steps(increment=0.02, steps=5):
    """
    Moves the end effector incrementally by a specified distance.
    
    Parameters:
    - increment: Distance in meters for each incremental movement
    - steps: Number of incremental movements to perform
    """
    rospy.init_node('incremental_trajectory')
    limb = Limb()

    # Get the current pose of the end-effector
    current_pose = limb.tip_state('right_hand').pose

    # Set initial target position
    target_pose = PoseStamped()
    target_pose.pose.orientation = current_pose.orientation  # Maintain the current orientation
    
    for step in range(steps):
        # Update target position by adding incremental values
        target_pose.pose.position.x = current_pose.position.x + (step + 1) * increment
        target_pose.pose.position.y = current_pose.position.y + (step + 1) * increment
        target_pose.pose.position.z = current_pose.position.z + (step + 1) * increment

        rospy.loginfo(f"Step {step + 1}: Moving to Position(x: {target_pose.pose.position.x}, "
                      f"y: {target_pose.pose.position.y}, z: {target_pose.pose.position.z})")

        # Define trajectory options with JOINT interpolation type for testing
        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.JOINT
        traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)

        # Configure waypoint options with reduced speeds and accelerations
        wpt_opts = MotionWaypointOptions(max_linear_speed=0.1, max_linear_accel=0.1)
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

        # Set Cartesian waypoint with nullspace bias from current joint angles
        joint_angles = limb.joint_ordered_angles()
        waypoint.set_cartesian_pose(target_pose, joint_angles=joint_angles)

        # Append waypoint and send trajectory
        traj.append_waypoint(waypoint.to_msg())
        result = traj.send_trajectory()

        if result is None or not result.result:
            rospy.logerr(f"Trajectory execution failed at step {step + 1} with error {result.errorId}")
            break
        else:
            rospy.loginfo(f"Step {step + 1} executed successfully!")

        # Update current pose to the last target position
        current_pose = target_pose.pose

if __name__ == '__main__':
    try:
        incremental_trajectory_steps(increment=0.02, steps=5)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt detected, exiting...")
