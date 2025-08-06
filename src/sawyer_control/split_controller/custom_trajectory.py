#!/usr/bin/env python

"""
Custom trajectory following implementation for Sawyer Robot Challenge
Uses direct joint control instead of MotionTrajectory for better control
"""

import rospy
import threading
import time
import numpy as np
from intera_interface import Limb
from intera_core_msgs.msg import JointCommand
from config import LOW_STIFFNESS


class CustomTrajectoryFollower:
    """Custom trajectory follower using direct joint control"""
    
    def __init__(self, limb_name='right'):
        self.limb = Limb(limb_name)
        self.limb_name = limb_name
        
        # Joint command publisher
        self.joint_cmd_pub = rospy.Publisher(
            f'/robot/limb/{limb_name}/joint_command', 
            JointCommand, 
            queue_size=10
        )
        
        # Control flags
        self.trajectory_active = False
        self.current_target_joints = None
        self.trajectory_thread = None
        
        # Control parameters
        self.control_rate = 100  # Hz - much higher than MotionTrajectory
        self.position_tolerance = 0.01  # radians
        self.max_joint_velocity = 0.3  # rad/s
        
        rospy.loginfo("Custom trajectory follower initialized")
    
    def move_to_joint_positions(self, target_joints, max_time=10.0):
        """
        Move to target joint positions with smooth interpolation
        
        Args:
            target_joints: Dict of joint names to target positions
            max_time: Maximum time to reach target (seconds)
            
        Returns:
            True if successful, False if failed
        """
        if self.trajectory_active:
            rospy.logwarn("Trajectory already active, stopping current trajectory")
            self.stop_trajectory()
        
        # Get current joint positions
        current_joints = self.limb.joint_angles()
        
        # Calculate trajectory
        trajectory = self._plan_smooth_trajectory(current_joints, target_joints, max_time)
        
        # Execute trajectory
        return self._execute_trajectory(trajectory)
    
    def move_through_cartesian_waypoints(self, poses, ik_solver, max_time_per_segment=5.0):
        """
        Move through a series of Cartesian poses using IK
        
        Args:
            poses: List of Pose objects
            ik_solver: IK solver function that converts pose to joint angles
            max_time_per_segment: Maximum time per waypoint
            
        Returns:
            True if successful, False if failed
        """
        if len(poses) == 0:
            return True
        
        success = True
        current_joints = self.limb.joint_angles()
        
        for i, pose in enumerate(poses):
            if rospy.is_shutdown() or not self.trajectory_active:
                break
            
            # Calculate IK for this pose
            target_joints = ik_solver(pose)
            if target_joints is None:
                rospy.logerr(f"IK failed for waypoint {i}")
                continue
            
            # Convert to joint dict - fix the joint names access
            try:
                joint_names = self.limb.joint_names()
            except AttributeError:
                # Fallback if joint_names() method doesn't exist
                joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
            
            target_joint_dict = {joint_names[j]: target_joints[j] for j in range(len(joint_names))}
            
            rospy.loginfo(f"Moving to waypoint {i+1}/{len(poses)}")
            
            # Move to this waypoint
            if not self.move_to_joint_positions(target_joint_dict, max_time_per_segment):
                rospy.logerr(f"Failed to reach waypoint {i}")
                success = False
                break
            
            current_joints = target_joint_dict
        
        return success
    
    def start_continuous_tracking(self, poses, ik_solver, loop_time=30.0):
        """
        Start continuous tracking through waypoints for a specified time
        
        Args:
            poses: List of Pose objects to track through
            ik_solver: IK solver function
            loop_time: Total time to track (seconds)
        """
        if self.trajectory_active:
            self.stop_trajectory()
        
        self.trajectory_active = True
        
        def tracking_loop():
            start_time = time.time()
            pose_index = 0
            
            rospy.loginfo(f"Starting continuous tracking for {loop_time} seconds")
            
            while (time.time() - start_time < loop_time and 
                   not rospy.is_shutdown() and 
                   self.trajectory_active):
                
                # Get current pose to track
                current_pose = poses[pose_index % len(poses)]
                
                # Calculate IK
                target_joints = ik_solver(current_pose)
                if target_joints is not None:
                    joint_names = self.limb.joint_names()
                    target_joint_dict = {joint_names[j]: target_joints[j] for j in range(len(joint_names))}
                    
                    # Send smooth joint command
                    self._send_smooth_joint_command(target_joint_dict)
                
                # Move to next pose
                pose_index += 1
                
                # Control loop timing
                rospy.sleep(0.5)  # Move to next waypoint every 0.5 seconds
            
            rospy.loginfo("Continuous tracking completed")
            self.trajectory_active = False
        
        self.trajectory_thread = threading.Thread(target=tracking_loop)
        self.trajectory_thread.daemon = True
        self.trajectory_thread.start()
    
    def _plan_smooth_trajectory(self, start_joints, end_joints, duration):
        """Plan smooth trajectory between joint positions"""
        try:
            joint_names = self.limb.joint_names()
        except AttributeError:
            # Fallback if joint_names() method doesn't exist
            joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        
        trajectory = []
        
        # Number of trajectory points
        num_points = int(duration * self.control_rate)
        
        for i in range(num_points + 1):
            t = float(i) / num_points
            
            # Smooth interpolation (cubic ease-in-out)
            smooth_t = 3 * t * t - 2 * t * t * t
            
            trajectory_point = {}
            for joint_name in joint_names:
                start_pos = start_joints[joint_name]
                end_pos = end_joints[joint_name]
                
                # Interpolate position
                pos = start_pos + smooth_t * (end_pos - start_pos)
                trajectory_point[joint_name] = pos
            
            trajectory.append(trajectory_point)
        
        return trajectory
    
    def _execute_trajectory(self, trajectory):
        """Execute planned trajectory"""
        self.trajectory_active = True
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo(f"Executing trajectory with {len(trajectory)} points")
        
        for i, target_joints in enumerate(trajectory):
            if rospy.is_shutdown() or not self.trajectory_active:
                break
            
            # Send joint command
            self._send_joint_command(target_joints)
            
            # Progress feedback
            if i % (self.control_rate // 4) == 0:  # Every 0.25 seconds
                progress = 100.0 * i / len(trajectory)
                rospy.logdebug(f"Trajectory progress: {progress:.1f}%")
            
            rate.sleep()
        
        self.trajectory_active = False
        rospy.loginfo("Trajectory execution completed")
        return True
    
    def _send_joint_command(self, target_joints):
        """Send joint position command"""
        joint_cmd = JointCommand()
        joint_cmd.mode = JointCommand.POSITION_MODE
        joint_cmd.names = list(target_joints.keys())
        joint_cmd.position = list(target_joints.values())
        joint_cmd.velocity = [0.0] * len(target_joints)
        joint_cmd.effort = [0.0] * len(target_joints)
        
        self.joint_cmd_pub.publish(joint_cmd)
    
    def _send_smooth_joint_command(self, target_joints):
        """Send smooth joint command with velocity limiting"""
        current_joints = self.limb.joint_angles()
        
        # Calculate limited velocities
        limited_joints = {}
        dt = 1.0 / self.control_rate
        
        for joint_name in target_joints.keys():
            current_pos = current_joints[joint_name]
            target_pos = target_joints[joint_name]
            
            # Calculate desired velocity
            desired_vel = (target_pos - current_pos) / dt
            
            # Limit velocity
            if abs(desired_vel) > self.max_joint_velocity:
                desired_vel = np.sign(desired_vel) * self.max_joint_velocity
            
            # Calculate limited target position
            limited_target = current_pos + desired_vel * dt
            limited_joints[joint_name] = limited_target
        
        self._send_joint_command(limited_joints)
    
    def stop_trajectory(self):
        """Stop current trajectory execution"""
        self.trajectory_active = False
        if self.trajectory_thread and self.trajectory_thread.is_alive():
            self.trajectory_thread.join(timeout=1.0)
        
        # Send current position as target to stop motion
        current_joints = self.limb.joint_angles()
        self._send_joint_command(current_joints)
        
        rospy.loginfo("Trajectory stopped")
    
    def hold_current_position(self):
        """Hold current position actively"""
        current_joints = self.limb.joint_angles()
        
        def hold_position():
            rate = rospy.Rate(50)  # High frequency for holding
            while not rospy.is_shutdown() and self.trajectory_active:
                self._send_joint_command(current_joints)
                rate.sleep()
        
        if self.trajectory_active:
            self.stop_trajectory()
        
        self.trajectory_active = True
        hold_thread = threading.Thread(target=hold_position)
        hold_thread.daemon = True
        hold_thread.start()
        
        rospy.loginfo("Holding current position")


class HybridController:
    """Combines custom trajectory following with impedance control"""
    
    def __init__(self, ik_motion_waypoint, custom_follower):
        self.ik_motion = ik_motion_waypoint
        self.custom_follower = custom_follower
        
    def execute_improved_normal_mode(self, spline_waypoints):
        """Improved normal mode using custom trajectory following"""
        try:
            # Enable low impedance for manual guidance
            self.ik_motion.enable_low_impedance()
            rospy.sleep(1.0)
            
            # Use custom follower to move to end point smoothly
            final_pose = spline_waypoints[-1]
            final_joints = self.ik_motion.calculate_ik(final_pose)
            
            if final_joints is not None:
                joint_names = self.ik_motion._limb.joint_names()
                target_joint_dict = {joint_names[j]: final_joints[j] for j in range(len(joint_names))}
                
                rospy.loginfo("IMPROVED NORMAL MODE: Moving to target with custom control...")
                success = self.custom_follower.move_to_joint_positions(target_joint_dict, max_time=15.0)
                
                if success:
                    rospy.loginfo("Reached target, now holding position with low impedance")
                    # Switch to position holding while maintaining low impedance
                    self.custom_follower.hold_current_position()
                else:
                    rospy.logerr("Failed to reach target position")
            
        except Exception as e:
            rospy.logerr(f"Error in improved normal mode: {e}")
            self.custom_follower.stop_trajectory()
    
    def execute_improved_tracking_mode(self, spline_waypoints):
        """Improved tracking mode using custom trajectory following"""
        try:
            # Enable low impedance for manual guidance
            self.ik_motion.enable_low_impedance()
            rospy.sleep(1.0)
            
            rospy.loginfo("IMPROVED TRACKING MODE: Following complete spline path...")
            
            # Use custom follower for smooth tracking
            success = self.custom_follower.move_through_cartesian_waypoints(
                spline_waypoints, 
                self.ik_motion.calculate_ik,
                max_time_per_segment=2.0
            )
            
            if success:
                rospy.loginfo("Completed tracking, now holding final position")
                self.custom_follower.hold_current_position()
            else:
                rospy.logerr("Tracking mode encountered errors")
            
        except Exception as e:
            rospy.logerr(f"Error in improved tracking mode: {e}")
            self.custom_follower.stop_trajectory()