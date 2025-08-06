#!/usr/bin/env python

#  Sawyer path visualizer with simple distance-based impedance control

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions, InteractionOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState, InteractionControlCommand
from std_msgs.msg import Empty

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from scipy.interpolate import CubicSpline
from scipy.spatial.distance import cdist

import numpy as np
import pygame
import threading


'''IMPORTANT VARIABLES'''
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

start_pose = Pose(position=Point(x=0.75, y=-0.3, z=0.3),
                  orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.75, y=0.5, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


def create_interaction_msg(stiffness):
    """
    Creates and configures an InteractionControlCommand message with the given stiffness.
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


def create_position_control_msg():
    """
    Creates a message to switch the robot back to position control mode.
    """
    msg = InteractionControlCommand()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base"
    msg.interaction_control_active = False  # Disable interaction control
    return msg


'''Simple Distance-Based Impedance Controller'''
class SimpleDistanceImpedanceController:
    def __init__(self, spline_trajectory, update_rate=20):
        """
        Simple controller: low impedance when close to spline, high when far
        
        Args:
            spline_trajectory: List of [x, y, z] waypoints
            update_rate: Control frequency in Hz (20 Hz is good balance)
        """
        self.spline_trajectory = np.array(spline_trajectory)
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        # Simple distance thresholds
        self.close_distance = 0.01    # 1cm - very low impedance
        self.far_distance = 0.05      # 5cm - high impedance
        
        # Simple impedance levels
        self.low_impedance = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]      # Easy to move
        self.high_impedance = [200.0, 200.0, 200.0, 15.0, 15.0, 15.0]  # Harder to move
        
        # Current state
        self.current_position = None
        self.distance_from_spline = 0.0
        self.lock = threading.Lock()
        
        # ROS setup
        self.impedance_pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                           InteractionControlCommand, queue_size=10)
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                    Empty, queue_size=10)
        
        # Control state
        self.control_active = False
        self.control_thread = None
        
        rospy.loginfo(f"Simple distance impedance controller initialized at {update_rate} Hz")
    
    def update_robot_position(self, position):
        """Update robot position - called from main visualizer"""
        with self.lock:
            self.current_position = np.array(position)
            if len(self.spline_trajectory) > 0:
                self.distance_from_spline = self.find_distance_to_spline(self.current_position)
    
    def find_distance_to_spline(self, current_pos):
        """
        Find shortest Y-Z distance to spline (ignoring X-axis)
        Returns: distance in meters
        """
        if len(self.spline_trajectory) == 0:
            return float('inf')
        
        # Only consider Y-Z coordinates
        current_yz = current_pos[1:3]  # [y, z]
        spline_yz = self.spline_trajectory[:, 1:3]  # All [y, z] points
        
        # Simple distance calculation - find closest point
        distances = cdist([current_yz], spline_yz)[0]
        min_distance = np.min(distances)
        
        return min_distance
    
    def calculate_impedance(self, distance):
        """
        Simple linear interpolation between low and high impedance
        
        Args:
            distance: Distance from spline in meters
            
        Returns:
            List of 6 impedance values [x, y, z, rx, ry, rz]
        """
        if distance <= self.close_distance:
            # Very close - use low impedance
            return self.low_impedance.copy()
        elif distance >= self.far_distance:
            # Far away - use high impedance
            return self.high_impedance.copy()
        else:
            # In between - linear interpolation
            ratio = (distance - self.close_distance) / (self.far_distance - self.close_distance)
            impedance = []
            for i in range(6):
                value = self.low_impedance[i] + ratio * (self.high_impedance[i] - self.low_impedance[i])
                impedance.append(value)
            return impedance
    
    def control_loop(self):
        """Main control loop - much simpler than PID version"""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo(f"Starting simple distance-based impedance control at {self.update_rate} Hz")
        
        while self.control_active and not rospy.is_shutdown():
            # Suppress collision avoidance
            self.collision_suppress_pub.publish(Empty())
            
            with self.lock:
                if self.current_position is None:
                    rate.sleep()
                    continue
                
                current_distance = self.distance_from_spline
            
            # Calculate impedance based on distance (simple!)
            target_impedance = self.calculate_impedance(current_distance)
            
            # Publish impedance command
            impedance_msg = create_interaction_msg(target_impedance)
            self.impedance_pub.publish(impedance_msg)
            
            # Simple status logging
            if current_distance < self.close_distance:
                status = "CLOSE (low resistance)"
            elif current_distance > self.far_distance:
                status = "FAR (high resistance)"
            else:
                status = "MEDIUM (variable resistance)"
            
            rospy.loginfo_throttle(1.0, f"Distance: {current_distance:.3f}m, "
                                       f"Impedance: {target_impedance[0]:.0f}, "
                                       f"Status: {status}")
            
            rate.sleep()
    
    def start_control(self):
        """Start the impedance control"""
        if self.control_active:
            rospy.logwarn("Control already active!")
            return
        
        rospy.loginfo("Starting distance-based impedance control...")
        self.control_active = True
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
    
    def stop_control(self):
        """Stop the impedance control"""
        rospy.loginfo("Stopping impedance control...")
        self.control_active = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join()
        
        # Return to position control
        position_msg = create_position_control_msg()
        for _ in range(5):
            self.impedance_pub.publish(position_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Returned to position control mode.")


'''Visualizer Display with THREE MODES'''
class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('sawyer_impedance_modes', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Multi-Mode Controller")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        # Convert spline waypoints to robot coordinates
        robot_trajectory = []
        for waypoint in self.spline_waypoints:
            robot_trajectory.append([waypoint.position.x, waypoint.position.y, waypoint.position.z])
        
        # Initialize the simple distance-based impedance controller
        self.distance_controller = SimpleDistanceImpedanceController(robot_trajectory, update_rate=20)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)
        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = []

        self.icon_color = (9, 179, 54)
        self.icon_radius = 30
        self.x = 800
        self.y = 1000
        self.position_initialized = True
        self.scale_x = 1900
        self.scale_y = 1950
        self.lock = threading.Lock()

        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                           EndpointState,
                                           self.position_callback)

        # Control modes
        self.robot_motion_active = False
        self.robot_ready_to_move = False
        self.motion_thread = None
        self.spacebar_disabled = False
        self.trajectory_complete = False

        # Register shutdown handler
        rospy.on_shutdown(self.shutdown_handler)

    def convert_to_pixels_x(self, meter_values, m_min, m_max, p_min, p_max):
        return [p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min) for m in meter_values]

    def convert_to_pixels_y(self, meter_values, m_min, m_max, p_min, p_max):
        pixel_values = []
        for m in meter_values:
            p = p_min + ((m - m_min) / (m_max - m_min)) * (p_max - p_min)
            if m > 0.3:
                p = 600 - (p - 600)
            else:
                p = 600 + (600 - p)
            pixel_values.append(p)
        return pixel_values

    def position_callback(self, msg):
        with self.lock:
            screen_x = (msg.pose.position.y * self.scale_x) + 770
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x))
            self.y = max(0, min(1080 - self.icon_radius, screen_y))
            
            # Update distance controller with current robot position
            robot_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.distance_controller.update_robot_position(robot_position)

    def move_to_start_position(self):
        rospy.loginfo("Moving to start position...")
        if not self.robot_ready_to_move:
            self.start_position_event = threading.Event()
            
            def move_and_signal():
                self.ik_motion.move_to_pose(start_pose)
                self.start_position_event.set()
            
            self.start_position_thread = threading.Thread(target=move_and_signal)
            self.start_position_thread.start()
            
            def wait_for_move_completion():
                self.start_position_event.wait()
                if not rospy.is_shutdown():
                    self.robot_ready_to_move = True
                    self.spacebar_disabled = False
                    rospy.loginfo("Reached start position. Ready for adaptive impedance motion.")
                    rospy.loginfo("SPACEBAR = Start slow motion to end WITH adaptive impedance")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_robot_motion(self):
        """Slow automatic motion to final point with ADAPTIVE impedance"""
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            rospy.loginfo("Starting ADAPTIVE impedance motion to final point...")
            rospy.loginfo("Robot moves slowly to end + impedance changes based on distance from spline!")
            
            def motion_wrapper():
                # Pass the distance controller to the trajectory execution
                self.ik_motion.execute_adaptive_impedance_trajectory(self.spline_waypoints, self.distance_controller)
                # Mark trajectory as complete and stop the game
                self.trajectory_complete = True
                rospy.loginfo("Adaptive trajectory complete. Stopping visualization...")
            
            self.motion_thread = threading.Thread(target=motion_wrapper)
            self.motion_thread.start()

    def start_distance_mode(self):
        """REMOVED - Distance mode not working properly"""
        pass

    def stop_distance_mode(self):
        """REMOVED - Distance mode not working properly"""
        pass

    def shutdown_handler(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down...")

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown() and not self.trajectory_complete:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if not self.robot_ready_to_move and not self.spacebar_disabled:  # First press
                            self.spacebar_disabled = True
                            self.move_to_start_position()
                        elif not self.robot_motion_active:  # Second press
                            self.start_robot_motion()

            self.window.fill(self.background_color)
            
            # Draw spline path as guide
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            # Draw real-time trail
            if self.robot_motion_active and self.robot_ready_to_move:
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)
            
            # Display mode information
            if not self.robot_ready_to_move:
                mode_text = "Press SPACEBAR to move to start position"
            elif self.robot_motion_active:
                mode_text = "Mode: Slow motion + Adaptive impedance (close to path = easy, far = hard)"
            else:
                mode_text = "SPACEBAR = Start adaptive impedance motion to end"
            
            # Simple text rendering (you might want to add pygame font for better text)
            pygame.display.set_caption(f"Sawyer Multi-Mode Controller - {mode_text}")
            
            pygame.display.flip()
            clock.tick(60)

        # Cleanup
        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


'''ROBOT MOTION WITH IMPEDANCE CONTROL'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
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
        
        # Very low stiffness for manual guidance
        self.low_stiffness = [2.0, 2.0, 2.0, 5.0, 5.0, 5.0]  # Much lower than original
        
        rospy.loginfo("IK and Low Impedance Motion setup complete.")

    def load_chain(self, filename, base_link, end_effector_link):
        with open(filename, "r") as urdf_file:
            urdf_string = urdf_file.read()
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse the URDF model!")
        return tree.getChain(base_link, end_effector_link)

    def calculate_ik(self, pose):
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

    def move_to_pose(self, pose, limb_name="right_hand"):
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Failed to move to pose: IK solution not found")
            return
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.1,
            max_linear_accel=0.1,
            max_rotational_speed=0.1,
            max_rotational_accel=0.1,
            max_joint_speed_ratio=0.1
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.clear_waypoints()
        self.traj.append_waypoint(waypoint.to_msg())
        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Failed to move to pose: Trajectory failed to send")
        elif result.result:
            rospy.loginfo("Successfully moved to pose!")
        else:
            rospy.logerr(f"Failed to move to pose with error {result.errorId}")

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=40):
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            [0.75, -0.1, 0.2],
            [0.75, 0.1, 0.4],
            [0.75, 0.3, 0.2],
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])

        t = np.linspace(0, 1, len(control_points))
        t_spline = np.linspace(0, 1, num_points)
        x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
        y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
        z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

        t_spline_display = np.linspace(0, 1, 1000)
        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

        self.waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                          for x, y, z in zip(x_spline, y_spline, z_spline)]

        display_waypoints = list(zip(y_spline_display, z_spline_display))
        return control_points, self.waypoints, display_waypoints

    def enable_low_impedance(self):
        """Enable low impedance mode for manual guidance"""
        rospy.loginfo("Enabling low impedance mode...")
        
        # Suppress collision avoidance
        self.collision_suppress_pub.publish(Empty())
        
        # Set very low stiffness
        low_impedance_msg = create_interaction_msg(self.low_stiffness)
        self.impedance_pub.publish(low_impedance_msg)
        
        rospy.loginfo("Low impedance mode enabled. Robot can be manually guided.")

    def disable_impedance_control(self):
        """Disable impedance control and return to position mode"""
        rospy.loginfo("Disabling impedance control...")
        position_control_msg = create_position_control_msg()
        for _ in range(5):  # Publish multiple times to ensure mode switch
            self.impedance_pub.publish(position_control_msg)
            rospy.sleep(0.2)
        rospy.loginfo("Returned to position control mode.")

    def execute_adaptive_impedance_trajectory(self, spline_waypoints, distance_controller):
        """
        Move directly to final waypoint with ADAPTIVE impedance based on distance from spline.
        Robot moves slowly to end while impedance changes based on how close you are to the spline path.
        """
        try:
            rospy.loginfo("Starting adaptive impedance trajectory...")
            rospy.loginfo("Robot will move slowly to end point.")
            rospy.loginfo("Impedance will be LOW when close to red path, HIGH when far from path.")
            
            # Suppress collision avoidance
            self.collision_suppress_pub.publish(Empty())
            rospy.sleep(0.5)
            
            # Only use the final waypoint from the spline
            final_pose = spline_waypoints[-1]
            
            # Set EXTREMELY slow motion parameters
            wpt_opts = MotionWaypointOptions(
                max_linear_speed=0.002,      # 2mm/s - much slower than before
                max_linear_accel=0.001,      # Very gentle acceleration
                max_rotational_speed=0.005,  # Slower rotation
                max_rotational_accel=0.002,
                max_joint_speed_ratio=0.05   # Use minimum allowed (0.05 instead of 0.02)
            )
            
            # Calculate IK for final position only
            joint_angles = self.calculate_ik(final_pose)
            if joint_angles is None:
                rospy.logerr("Failed to calculate IK for final position")
                return
            
            # Create single waypoint to final position
            waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
            waypoint.set_joint_angles(joint_angles, "right_hand")
            
            # Clear any existing trajectory and add only the final waypoint
            self.traj.clear_waypoints()
            self.traj.append_waypoint(waypoint.to_msg())
            
            # Start the distance-based impedance controller BEFORE starting motion
            distance_controller.start_control()
            
            # Execute the trajectory (this runs in parallel with distance controller)
            result = self.traj.send_trajectory()
            
            if result is None:
                rospy.logerr("Trajectory FAILED to send")
            elif result.result:
                rospy.loginfo("Successfully completed adaptive impedance motion!")
            else:
                rospy.logerr(f"Trajectory execution failed with error {result.errorId}")
            
        except Exception as e:
            rospy.logerr(f"Error during adaptive impedance trajectory: {e}")
        
        finally:
            # Stop the distance controller and return to position control
            rospy.sleep(1.0)  # Allow motion to complete
            distance_controller.stop_control()


if __name__ == '__main__':
    try:
        sawyerVisualizer = SawyerVisualizer()
        sawyerVisualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt detected. Exiting...")