#!/usr/bin/env python

"""
Pure direct robot control - completely bypasses all SDK trajectory systems
Uses only raw ROS messages for maximum control
"""

import rospy
import threading
import time
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand
from intera_interface import Limb


class PureDirectController:
    """Completely direct robot control bypassing ALL SDK components"""
    
    def __init__(self, limb_name='right'):
        self.limb_name = limb_name
        self.limb = Limb(limb_name)
        
        # Raw joint command publisher - bypasses everything
        self.joint_cmd_pub = rospy.Publisher(
            f'/robot/limb/{limb_name}/joint_command',
            JointCommand,
            queue_size=1
        )
        
        # Joint state subscriber for feedback
        self.joint_state_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self._joint_state_callback
        )
        
        # Current joint states
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.joint_names = [f'{limb_name}_j{i}' for i in range(7)]
        
        # Control parameters
        self.control_active = False
        self.control_thread = None
        self.target_positions = {}
        self.max_velocity = 0.5  # rad/s
        self.position_tolerance = 0.01  # rad
        
        # Initialize current positions
        self._wait_for_joint_states()
        
        rospy.loginfo("Pure direct controller initialized - NO SDK trajectory system used!")
    
    def _wait_for_joint_states(self):
        """Wait for initial joint state message"""
        rospy.loginfo("Waiting for joint states...")
        timeout = time.time() + 5.0
        
        while not self.current_joint_positions and time.time() < timeout:
            rospy.sleep(0.1)
        
        if self.current_joint_positions:
            rospy.loginfo("Joint states received - ready for control")
        else:
            rospy.logwarn("No joint states received - using limb interface fallback")
            try:
                self.current_joint_positions = self.limb.joint_angles()
            except:
                rospy.logerr("Could not get joint positions")
    
    def _joint_state_callback(self, msg):
        """Update current joint states from ROS message"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]
                self.current_joint_velocities[name] = msg.velocity[i]
    
    def move_to_positions(self, target_positions, max_time=10.0):
        """
        Move to target joint positions using pure direct control
        
        Args:
            target_positions: Dict of {joint_name: angle}
            max_time: Maximum time to reach targets
            
        Returns:
            True if successful
        """
        rospy.loginfo(f"PURE DIRECT: Moving to target positions")
        
        # Store targets
        self.target_positions = target_positions.copy()
        
        # Start control loop
        if self.control_active:
            self.stop_control()
        
        self.control_active = True
        success = self._execute_direct_control(max_time)
        self.control_active = False
        
        return success
    
    def _execute_direct_control(self, max_time):
        """Execute direct control loop"""
        rate = rospy.Rate(200)  # Very high frequency - 200Hz
        start_time = time.time()
        
        rospy.loginfo("Starting pure direct control loop at 200Hz")
        
        while (self.control_active and 
               not rospy.is_shutdown() and 
               time.time() - start_time < max_time):
            
            # Calculate next command
            command_positions = self._calculate_next_positions()
            
            # Send direct command
            self._send_direct_command(command_positions)
            
            # Check if reached targets
            if self._at_targets():
                rospy.loginfo("PURE DIRECT: Reached all targets!")
                return True
            
            rate.sleep()
        
        rospy.loginfo("PURE DIRECT: Control loop ended")
        return False
    
    def _calculate_next_positions(self):
        """Calculate next joint positions for smooth motion"""
        command_positions = {}
        dt = 1.0 / 200.0  # 200Hz control
        
        for joint_name in self.joint_names:
            if joint_name not in self.current_joint_positions:
                continue
                
            current_pos = self.current_joint_positions[joint_name]
            target_pos = self.target_positions.get(joint_name, current_pos)
            
            # Calculate error
            error = target_pos - current_pos
            
            # Limit velocity
            max_step = self.max_velocity * dt
            if abs(error) > max_step:
                step = np.sign(error) * max_step
            else:
                step = error
            
            command_positions[joint_name] = current_pos + step
        
        return command_positions
    
    def _send_direct_command(self, positions):
        """Send pure direct joint command - no SDK interference"""
        cmd = JointCommand()
        cmd.mode = JointCommand.POSITION_MODE
        cmd.names = list(positions.keys())
        cmd.position = list(positions.values())
        cmd.velocity = [0.0] * len(positions)
        cmd.effort = [0.0] * len(positions)
        
        # Publish directly
        self.joint_cmd_pub.publish(cmd)
    
    def _at_targets(self):
        """Check if all joints are at target positions"""
        for joint_name in self.joint_names:
            if joint_name not in self.current_joint_positions:
                return False
            
            current = self.current_joint_positions[joint_name]
            target = self.target_positions.get(joint_name, current)
            
            if abs(current - target) > self.position_tolerance:
                return False
        
        return True
    
    def stop_control(self):
        """Stop all control and hold current position"""
        self.control_active = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        
        # Send current position as command to stop motion
        if self.current_joint_positions:
            self._send_direct_command(self.current_joint_positions)
        
        rospy.loginfo("Pure direct control stopped")
    
    def follow_trajectory(self, joint_trajectory, trajectory_time):
        """
        Follow a complete joint trajectory
        
        Args:
            joint_trajectory: List of joint position dicts
            trajectory_time: Total time for trajectory
        """
        if len(joint_trajectory) == 0:
            return True
        
        rospy.loginfo(f"PURE DIRECT: Following trajectory with {len(joint_trajectory)} points")
        
        self.control_active = True
        rate = rospy.Rate(200)  # 200Hz
        
        # Calculate timing
        points_per_second = len(joint_trajectory) / trajectory_time
        dt_between_points = 1.0 / points_per_second
        
        start_time = time.time()
        
        for i, target_positions in enumerate(joint_trajectory):
            if not self.control_active or rospy.is_shutdown():
                break
            
            # Calculate how long to spend on this waypoint
            waypoint_start = time.time()
            waypoint_duration = dt_between_points
            
            # Move towards this waypoint
            while (time.time() - waypoint_start < waypoint_duration and
                   self.control_active and not rospy.is_shutdown()):
                
                # Interpolate towards waypoint
                alpha = (time.time() - waypoint_start) / waypoint_duration
                alpha = min(1.0, alpha)
                
                interpolated_positions = {}
                for joint_name in self.joint_names:
                    if joint_name in self.current_joint_positions and joint_name in target_positions:
                        current = self.current_joint_positions[joint_name]
                        target = target_positions[joint_name]
                        interpolated_positions[joint_name] = current + alpha * (target - current)
                
                self._send_direct_command(interpolated_positions)
                rate.sleep()
            
            # Progress feedback
            if i % 10 == 0:
                progress = 100.0 * i / len(joint_trajectory)
                rospy.loginfo(f"Trajectory progress: {progress:.1f}%")
        
        self.control_active = False
        rospy.loginfo("Pure direct trajectory following completed")
        return True


class TrulyDirectRobotInterface:
    """
    Interface that completely bypasses SDK for robot control
    """
    
    def __init__(self, ik_solver):
        self.ik_solver = ik_solver
        self.direct_controller = PureDirectController()
        
        rospy.loginfo("Truly direct robot interface initialized")
    
    def execute_direct_normal_mode(self, spline_waypoints):
        """
        Normal mode using ONLY direct control - no SDK at all
        """
        rospy.loginfo("=== TRULY DIRECT NORMAL MODE ===")
        rospy.loginfo("NO impedance control, NO SDK trajectory, PURE direct joint control")
        
        try:
            # Get final target
            final_pose = spline_waypoints[-1]
            
            # Calculate IK
            joint_angles = self.ik_solver(final_pose)
            if joint_angles is None:
                rospy.logerr("IK failed for final pose")
                return
            
            # Convert to joint dict
            joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
            target_positions = {joint_names[i]: joint_angles[i] for i in range(len(joint_angles))}
            
            rospy.loginfo("Moving directly to final position - you can manually guide the robot!")
            
            # Move using pure direct control
            success = self.direct_controller.move_to_positions(target_positions, max_time=25.0)
            
            if success:
                rospy.loginfo("Direct normal mode completed successfully!")
            else:
                rospy.logwarn("Direct normal mode timed out, but continuing...")
            
        except Exception as e:
            rospy.logerr(f"Error in direct normal mode: {e}")
    
    def execute_direct_tracking_mode(self, spline_waypoints):
        """
        Tracking mode using ONLY direct control - no SDK at all
        """
        rospy.loginfo("=== TRULY DIRECT TRACKING MODE ===")
        rospy.loginfo("NO impedance control, NO SDK trajectory, PURE direct joint control")
        
        try:
            # Convert all waypoints to joint trajectories
            joint_trajectory = []
            
            for i, pose in enumerate(spline_waypoints):
                joint_angles = self.ik_solver(pose)
                if joint_angles is None:
                    rospy.logwarn(f"IK failed for waypoint {i}, skipping")
                    continue
                
                joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
                joint_positions = {joint_names[j]: joint_angles[j] for j in range(len(joint_angles))}
                joint_trajectory.append(joint_positions)
            
            if len(joint_trajectory) == 0:
                rospy.logerr("No valid waypoints for tracking")
                return
            
            rospy.loginfo(f"Following trajectory with {len(joint_trajectory)} waypoints")
            rospy.loginfo("Robot will guide you through the complete path - you can still influence it!")
            
            # Execute trajectory using pure direct control
            success = self.direct_controller.follow_trajectory(joint_trajectory, trajectory_time=20.0)
            
            if success:
                rospy.loginfo("Direct tracking mode completed successfully!")
            else:
                rospy.logwarn("Direct tracking mode timed out, but continuing...")
            
        except Exception as e:
            rospy.logerr(f"Error in direct tracking mode: {e}")
    
    def stop_all_motion(self):
        """Emergency stop - halt all motion immediately"""
        rospy.loginfo("EMERGENCY STOP: Halting all direct control")
        self.direct_controller.stop_control()