#!/usr/bin/env python

"""
Robot motion control for Sawyer Robot Challenge
Handles kinematics, trajectory execution, and impedance control
"""

import rospy
import threading
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions, InteractionOptions
from intera_interface import Limb
from intera_core_msgs.msg import InteractionControlCommand
from std_msgs.msg import Empty

from config import (LOW_STIFFNESS, NORMAL_MODE_SPEED, TRACKING_MODE_SPEED, 
                   HOLD_POSITION_SPEED, MOVE_TO_START_SPEED)
from spline_generator import SplineGenerator


class InteractionControl:
    """Handles impedance control and interaction control messages"""
    
    @staticmethod
    def create_interaction_msg(stiffness):
        """
        Creates and configures an InteractionControlCommand message with the given stiffness.
        
        Args:
            stiffness: List of stiffness values for each axis
            
        Returns:
            InteractionControlCommand message
        """
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_K_impedance(stiffness)  # Cartesian stiffness
        interaction_options.set_max_impedance([False, False, False, False, False, False])  # Not maximizing stiffness
        interaction_options.set_interaction_control_mode([1, 1, 1, 1, 1, 1])  # Impedance mode for all axes
        interaction_options.set_in_endpoint_frame(False)  # Base frame as reference
        interaction_options.set_force_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No external force commands
        interaction_options.set_K_nullspace([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # Nullspace stiffness
        interaction_options.set_disable_damping_in_force_control(False)  # Enable damping
        interaction_options.set_disable_reference_resetting(False)  # Allow smooth transitions

        return interaction_options.to_msg()

    @staticmethod
    def create_position_control_msg():
        """
        Creates a message to switch the robot back to position control mode.
        
        Returns:
            InteractionControlCommand message for position control
        """
        msg = InteractionControlCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        msg.interaction_control_active = False  # Disable interaction control
        return msg


class IKMotionWaypoint:
    """Handles inverse kinematics, trajectory planning, and motion execution"""
    
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self._load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
        self.traj = MotionTrajectory(limb=self._limb)
        
        # Impedance control setup
        self.impedance_pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                           InteractionControlCommand, queue_size=10)
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                    Empty, queue_size=10)
        
        # Spline generator
        self.spline_generator = SplineGenerator()
        
        # Thread control flag
        self.impedance_active = False
        
        rospy.loginfo("IK and Low Impedance Motion setup complete.")

    def _load_chain(self, filename, base_link, end_effector_link):
        """Load URDF chain for kinematics calculations"""
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
        return tree.getChain(base_link, end_effector_link)

    def calculate_ik(self, pose):
        """
        Calculate inverse kinematics for a given pose
        
        Args:
            pose: Target pose for end effector
            
        Returns:
            List of joint angles or None if IK fails
        """
        pos = pose.position
        orientation = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w)
        frame = kdl.Frame(orientation, kdl.Vector(pos.x, pos.y, pos.z))
        result = self.ik_solver.CartToJnt(self.ik_joint_positions_prev, frame, self.ik_joint_positions)
        if result < 0:
            rospy.logerr("IK solution not found")
            return None
        for i in range(self.num_joints):
            self.ik_joint_positions_prev[i] = self.ik_joint_positions[i]
        return [self.ik_joint_positions[i] for i in range(self.num_joints)]

    def move_to_pose(self, pose, limb_name="right_hand", speed_config=None):
        """
        Move robot to a specific pose
        
        Args:
            pose: Target pose
            limb_name: Name of the limb endpoint
            speed_config: Speed configuration dict (uses MOVE_TO_START_SPEED if None)
            
        Returns:
            Success status of the movement
        """
        if speed_config is None:
            speed_config = MOVE_TO_START_SPEED
            
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Failed to move to pose: IK solution not found")
            return False
            
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=speed_config['max_linear_speed'],
            max_linear_accel=speed_config['max_linear_accel'],
            max_rotational_speed=speed_config['max_rotational_speed'],
            max_rotational_accel=speed_config['max_rotational_accel'],
            max_joint_speed_ratio=speed_config['max_joint_speed_ratio']
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.clear_waypoints()
        self.traj.append_waypoint(waypoint.to_msg())
        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Failed to move to pose: Trajectory failed to send")
            return False
        elif result.result:
            rospy.loginfo("Successfully moved to pose!")
            return True
        else:
            rospy.logerr(f"Failed to move to pose with error {result.errorId}")
            return False

    def generate_spline_waypoints_custom(self, start_pose, end_pose, control_points, spline_type='spline', num_points=40):
        """Generate spline waypoints using the spline generator"""
        return self.spline_generator.generate_spline_waypoints_custom(
            start_pose, end_pose, control_points, spline_type, num_points
        )

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling low impedance mode...")
        
        # Initialize thread control flag
        self.impedance_active = True
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = InteractionControl.create_interaction_msg(LOW_STIFFNESS)
        self.impedance_pub.publish(low_impedance_msg)
        
        # Keep publishing to maintain low impedance
        def maintain_low_impedance():
            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown() and self.impedance_active:
                self.collision_suppress_pub.publish(Empty())
                low_impedance_msg = InteractionControl.create_interaction_msg(LOW_STIFFNESS)
                self.impedance_pub.publish(low_impedance_msg)
                rate.sleep()
        
        self.impedance_thread = threading.Thread(target=maintain_low_impedance)
        self.impedance_thread.daemon = True
        self.impedance_thread.start()
        
        rospy.loginfo("Low impedance mode enabled. Robot can be manually guided.")

    def disable_impedance_control(self):
        """Disable impedance control and return to position mode"""
        rospy.loginfo("Disabling impedance control...")
        
        # Stop the impedance maintenance thread
        self.impedance_active = False
        if hasattr(self, 'impedance_thread') and self.impedance_thread.is_alive():
            self.impedance_thread.join(timeout=1.0)
        
        position_control_msg = InteractionControl.create_position_control_msg()
        for _ in range(15):  # Publish more times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        
        # Also clear any existing trajectory to ensure clean state
        self.traj.clear_waypoints()
        rospy.loginfo("Cleared trajectory waypoints.")
        
        # Wait a bit more to ensure the control mode has fully switched
        rospy.sleep(0.5)
        rospy.loginfo("Returned to position control mode.")

    def ensure_position_control(self):
        """Ensure robot is in normal position control mode with standard motion parameters"""
        rospy.loginfo("Ensuring robot is in position control mode...")
        
        # Disable any impedance control
        self.disable_impedance_control()
        
        # Re-establish position control explicitly
        rospy.loginfo("Re-establishing position control...")
        position_control_msg = InteractionControl.create_position_control_msg()
        for _ in range(10):
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.1)
        
        # Clear trajectory and reset motion interface
        self.traj.clear_waypoints()
        rospy.sleep(1.0)  # Give time for mode to fully establish
        
        rospy.loginfo("Position control mode established. Ready for normal motion.")

    def _lock_robot_at_current_position(self):
        """Lock robot at current position to prevent movement"""
        try:
            current_joint_angles = self._limb.joint_angles()
            hold_wpt_opts = MotionWaypointOptions(
                max_linear_speed=HOLD_POSITION_SPEED['max_linear_speed'],
                max_linear_accel=HOLD_POSITION_SPEED['max_linear_accel'],
                max_rotational_speed=HOLD_POSITION_SPEED['max_rotational_speed'],
                max_rotational_accel=HOLD_POSITION_SPEED['max_rotational_accel'],
                max_joint_speed_ratio=HOLD_POSITION_SPEED['max_joint_speed_ratio']
            )
            hold_waypoint = MotionWaypoint(options=hold_wpt_opts.to_msg(), limb=self._limb)
            hold_waypoint.set_joint_angles(current_joint_angles, "right_hand")
            
            self.traj.clear_waypoints()
            self.traj.append_waypoint(hold_waypoint.to_msg())
            rospy.loginfo("Robot locked at current position")
            return True
        except Exception as e:
            rospy.logwarn(f"Could not lock robot position: {e}")
            return False

    def execute_low_impedance_trajectory(self, spline_waypoints):
        """
        NORMAL MODE (Run 1): Move directly to final waypoint with very low impedance and extremely slow speed.
        Fixed to prevent robot from shooting backwards when reaching target.
        """
        try:
            # Enable low impedance mode first
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Only use the final waypoint from the spline
            final_pose = spline_waypoints[-1]
            
            # Calculate IK for final position only
            joint_angles = self.calculate_ik(final_pose)
            if joint_angles is None:
                rospy.logerr("Failed to calculate IK for final position")
                return
            
            # Create single waypoint to final position
            wpt_opts = MotionWaypointOptions(
                max_linear_speed=NORMAL_MODE_SPEED['max_linear_speed'],
                max_linear_accel=NORMAL_MODE_SPEED['max_linear_accel'],
                max_rotational_speed=NORMAL_MODE_SPEED['max_rotational_speed'],
                max_rotational_accel=NORMAL_MODE_SPEED['max_rotational_accel'],
                max_joint_speed_ratio=NORMAL_MODE_SPEED['max_joint_speed_ratio']
            )
            waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
            waypoint.set_joint_angles(joint_angles, "right_hand")
            
            # Clear any existing trajectory and add only the final waypoint
            self.traj.clear_waypoints()
            self.traj.append_waypoint(waypoint.to_msg())
            
            rospy.loginfo("NORMAL MODE: Moving directly to final position with low impedance...")
            rospy.loginfo("You can now manually guide the robot arm during motion!")
            
            # Execute the trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed normal mode motion to final position!")
            else:
                rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
            
            # CRITICAL FIX: Immediately stop any motion and clear trajectory
            rospy.loginfo("Stopping robot motion to prevent backward shooting...")
            
            # Stop the robot by clearing trajectory immediately
            self.traj.clear_waypoints()  # Clear all waypoints
            rospy.sleep(0.2)
            
            # Lock robot at current position
            self._lock_robot_at_current_position()
            
            # Maintain low impedance but stop any active motion
            rospy.loginfo("Maintaining low impedance mode for manual guidance only")
            
        except Exception as e:
            rospy.logerr(f"Error during normal mode trajectory: {e}")
            # Safety cleanup on error
            self.traj.stop()
            self.traj.clear_waypoints()

    def execute_tracking_trajectory(self, spline_waypoints):
        """
        TRACKING MODE (Run 2): Execute trajectory through ALL spline waypoints with low impedance.
        Fixed to prevent robot from shooting backwards when reaching target.
        """
        try:
            # Enable low impedance mode first
            self.enable_low_impedance()
            rospy.sleep(1.0)  # Allow impedance mode to activate
            
            # Clear any existing trajectory
            self.traj.clear_waypoints()
            
            # Add ALL spline waypoints to the trajectory
            rospy.loginfo(f"TRACKING MODE: Adding {len(spline_waypoints)} waypoints to trajectory...")
            
            waypoints_added = 0
            for i, pose in enumerate(spline_waypoints):
                # Calculate IK for each waypoint
                joint_angles = self.calculate_ik(pose)
                if joint_angles is None:
                    rospy.logerr(f"Failed to calculate IK for waypoint {i}")
                    continue
                
                # Create waypoint with tracking mode speed
                wpt_opts = MotionWaypointOptions(
                    max_linear_speed=TRACKING_MODE_SPEED['max_linear_speed'],
                    max_linear_accel=TRACKING_MODE_SPEED['max_linear_accel'],
                    max_rotational_speed=TRACKING_MODE_SPEED['max_rotational_speed'],
                    max_rotational_accel=TRACKING_MODE_SPEED['max_rotational_accel'],
                    max_joint_speed_ratio=TRACKING_MODE_SPEED['max_joint_speed_ratio']
                )
                waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
                waypoint.set_joint_angles(joint_angles, "right_hand")
                
                # Add to trajectory
                self.traj.append_waypoint(waypoint.to_msg())
                waypoints_added += 1
            
            rospy.loginfo(f"TRACKING MODE: Successfully added {waypoints_added} waypoints to trajectory")
            rospy.loginfo("TRACKING MODE: Robot will now pull you through ALL spline points!")
            rospy.loginfo("You can still guide the robot, but it will follow the complete path!")
            
            # Execute the full trajectory
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Tracking trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed tracking mode through all spline points!")
            else:
                rospy.logerr(f"Tracking trajectory execution failed with error {result.errorId}")
            
            # CRITICAL FIX: Immediately stop any motion and clear trajectory
            rospy.loginfo("Stopping robot motion to prevent backward shooting...")
            
            # Stop the robot by clearing trajectory immediately
            self.traj.clear_waypoints()  # Clear all waypoints
            rospy.sleep(0.2)
            
            # Lock robot at current position
            self._lock_robot_at_current_position()
            
            # Maintain low impedance but stop any active motion
            rospy.loginfo("Maintaining low impedance mode for manual guidance only")
                
        except Exception as e:
            rospy.logerr(f"Error during tracking trajectory: {e}")
            # Safety cleanup on error
            self.traj.stop()
            self.traj.clear_waypoints()

    def emergency_stop(self):
        """Emergency stop method to immediately halt all robot motion"""
        rospy.loginfo("EMERGENCY STOP INITIATED")
        
        # Clear all waypoints multiple times
        for i in range(3):
            try:
                self.traj.clear_waypoints()
                rospy.sleep(0.1)
            except Exception as e:
                rospy.logwarn(f"Could not clear waypoints: {e}")
        
        # Try to lock robot at current position
        self._lock_robot_at_current_position()
        
        # Disable impedance and return to position control
        try:
            self.disable_impedance_control()
        except Exception as e:
            rospy.logwarn(f"Could not disable impedance control: {e}")
            
        rospy.loginfo("Emergency stop completed")