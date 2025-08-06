#!/usr/bin/env python

#  Sawyer path 2 (Spline) visualizer with PID spline tracking controller

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState
from intera_core_msgs.msg import InteractionControlCommand

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


'''ADDED: Spline Tracking PID Controller'''
class SplineTrackingController:
    def __init__(self, spline_trajectory):
        """
        spline_trajectory: List of (x, y, z) waypoints defining the desired spline path
        """
        self.spline_trajectory = np.array(spline_trajectory)
        
        # PID parameters for stiffness control based on distance deviation
        self.kp = 800.0   # Proportional gain - how much to react to current distance error
        self.ki = 100.0   # Integral gain - correct for persistent deviation
        self.kd = 200.0   # Derivative gain - smooth out rapid changes
        
        # Error tracking
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.dt = 0.1  # Control loop period (10 Hz)
        
        # Distance thresholds
        self.max_allowable_distance = 0.05  # 5cm max deviation before max stiffness
        self.good_tracking_distance = 0.01   # 1cm or less = good tracking
        
        # Stiffness bounds
        self.min_stiffness = [50.0, 50.0, 50.0, 25.0, 25.0, 25.0]    # Compliant when on-track
        self.max_stiffness = [1800.0, 1800.0, 1800.0, 60.0, 60.0, 60.0]  # Rigid when off-track
        self.base_stiffness = [300.0, 300.0, 300.0, 35.0, 35.0, 35.0]     # Normal operation
        
        # Current robot state
        self.current_position = None
        self.closest_spline_point = None
        self.distance_from_spline = 0.0
        self.lock = threading.Lock()
        
        # ROS setup for stiffness control
        self.stiffness_pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                           InteractionControlCommand, queue_size=10)
        
        # Control state
        self.control_active = False
        self.control_thread = None
        
        rospy.loginfo(f"Spline tracking controller initialized with {len(self.spline_trajectory)} waypoints")
    
    def update_robot_position(self, position):
        """Called by the main visualizer to update robot position"""
        with self.lock:
            self.current_position = np.array(position)
            if len(self.spline_trajectory) > 0:
                self.closest_spline_point, self.distance_from_spline = self.find_closest_spline_point(self.current_position)
    
    def find_closest_spline_point(self, current_pos):
        """Find the closest point on the spline trajectory to current position"""
        if len(self.spline_trajectory) == 0:
            return None, float('inf')
        
        # Calculate distances to all spline points
        distances = cdist([current_pos], self.spline_trajectory)[0]
        closest_idx = np.argmin(distances)
        closest_point = self.spline_trajectory[closest_idx]
        min_distance = distances[closest_idx]
        
        # For more accuracy, interpolate between adjacent points
        if len(self.spline_trajectory) > 1:
            # Check if we can interpolate with neighbors
            if closest_idx > 0:
                # Check interpolation with previous point
                prev_point = self.spline_trajectory[closest_idx - 1]
                interp_point, interp_dist = self.closest_point_on_line_segment(
                    current_pos, prev_point, closest_point)
                if interp_dist < min_distance:
                    closest_point = interp_point
                    min_distance = interp_dist
            
            if closest_idx < len(self.spline_trajectory) - 1:
                # Check interpolation with next point
                next_point = self.spline_trajectory[closest_idx + 1]
                interp_point, interp_dist = self.closest_point_on_line_segment(
                    current_pos, closest_point, next_point)
                if interp_dist < min_distance:
                    closest_point = interp_point
                    min_distance = interp_dist
        
        return closest_point, min_distance
    
    def closest_point_on_line_segment(self, point, line_start, line_end):
        """Find closest point on line segment to given point"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_len_sq = np.dot(line_vec, line_vec)
        if line_len_sq == 0:
            return line_start, np.linalg.norm(point - line_start)
        
        t = np.dot(point_vec, line_vec) / line_len_sq
        t = max(0, min(1, t))  # Clamp to line segment
        
        projection = line_start + t * line_vec
        distance = np.linalg.norm(point - projection)
        
        return projection, distance
    
    def calculate_deviation_error(self):
        """Calculate how far off the spline we are (normalized)"""
        if self.distance_from_spline is None:
            return 0.0
        normalized_error = min(self.distance_from_spline / self.max_allowable_distance, 1.0)
        return normalized_error
    
    def pid_control(self, error):
        """Calculate PID output for stiffness adjustment"""
        proportional = self.kp * error
        
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -1.0, 1.0)  # Prevent windup
        integral = self.ki * self.integral_error
        
        derivative = self.kd * (error - self.previous_error) / self.dt
        self.previous_error = error
        
        pid_output = proportional + integral + derivative
        return pid_output
    
    def calculate_adaptive_stiffness(self, deviation_error, pid_output):
        """Convert deviation error and PID output to stiffness values"""
        base_scale = 1.0 + (deviation_error * 3.0)  # Scale from 1.0 to 4.0
        
        pid_scale = 1.0 + (pid_output / 1000.0)
        pid_scale = np.clip(pid_scale, 0.2, 5.0)
        
        total_scale = base_scale * pid_scale
        
        adaptive_stiffness = []
        for i, base in enumerate(self.base_stiffness):
            new_stiffness = base * total_scale
            new_stiffness = np.clip(new_stiffness, 
                                  self.min_stiffness[i], 
                                  self.max_stiffness[i])
            adaptive_stiffness.append(new_stiffness)
        
        return adaptive_stiffness
    
    def create_stiffness_message(self, stiffness):
        """Create interaction control message"""
        from intera_motion_interface import InteractionOptions
        
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_K_impedance(stiffness)
        interaction_options.set_max_impedance([False] * 6)
        interaction_options.set_interaction_control_mode([1] * 6)  # Impedance mode
        interaction_options.set_in_endpoint_frame(False)
        interaction_options.set_force_command([0.0] * 6)
        interaction_options.set_K_nullspace([5.0] * 7)
        interaction_options.set_disable_damping_in_force_control(False)
        interaction_options.set_disable_reference_resetting(False)
        
        return interaction_options.to_msg()
    
    def control_loop(self):
        """Main PID control loop"""
        rate = rospy.Rate(1.0 / self.dt)  # Control frequency
        
        rospy.loginfo("Starting spline tracking control loop...")
        
        while self.control_active and not rospy.is_shutdown():
            with self.lock:
                if self.current_position is None:
                    rate.sleep()
                    continue
                
                distance_error = self.distance_from_spline
                closest_point = self.closest_spline_point.copy() if self.closest_spline_point is not None else None
            
            # Calculate normalized deviation error
            deviation_error = self.calculate_deviation_error()
            
            # PID control based on deviation
            pid_output = self.pid_control(deviation_error)
            
            # Calculate adaptive stiffness
            adaptive_stiffness = self.calculate_adaptive_stiffness(deviation_error, pid_output)
            
            # Publish stiffness command
            stiffness_msg = self.create_stiffness_message(adaptive_stiffness)
            self.stiffness_pub.publish(stiffness_msg)
            
            # Logging
            status = "ON_TRACK" if distance_error < self.good_tracking_distance else "OFF_TRACK"
            rospy.loginfo(f"Distance from spline: {distance_error:.4f}m, "
                         f"Deviation error: {deviation_error:.3f}, "
                         f"PID: {pid_output:.1f}, "
                         f"Stiffness: {adaptive_stiffness[0]:.0f}, "
                         f"Status: {status}")
            
            rate.sleep()
    
    def start_tracking(self):
        """Start spline deviation tracking"""
        if self.control_active:
            rospy.logwarn("Spline tracking already active!")
            return
        
        rospy.loginfo("Starting spline deviation tracking...")
        self.control_active = True
        
        # Reset PID state
        self.previous_error = 0.0
        self.integral_error = 0.0
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
    
    def stop_tracking(self):
        """Stop spline deviation tracking"""
        rospy.loginfo("Stopping spline deviation tracking...")
        self.control_active = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join()
        
        # Return to base stiffness
        base_msg = self.create_stiffness_message(self.base_stiffness)
        self.stiffness_pub.publish(base_msg)


'''MODIFIED: Visualizer Display with PID Integration'''
class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('positionSubscriber', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display with PID Control")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        # ADDED: Convert spline waypoints to robot coordinates for PID controller
        robot_trajectory = []
        for waypoint in self.spline_waypoints:
            robot_trajectory.append([waypoint.position.x, waypoint.position.y, waypoint.position.z])
        
        # ADDED: Initialize the PID spline tracking controller
        self.spline_controller = SplineTrackingController(robot_trajectory)

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

        self.robot_motion_active = False
        self.robot_ready_to_move = False
        self.motion_thread = None
        self.spacebar_disabled = False

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
            
            # ADDED: Update PID controller with current robot position
            robot_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.spline_controller.update_robot_position(robot_position)

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
                    rospy.loginfo("Reached start position.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            
            # Start the original trajectory execution
            self.motion_thread = threading.Thread(target=self.ik_motion.execute_trajectory, args=(self.spline_waypoints,))
            self.motion_thread.start()
            
            # ADDED: Start the PID spline tracking controller
            self.spline_controller.start_tracking()
            rospy.loginfo("Started both trajectory execution and PID spline tracking!")

    def run(self):
        clock = pygame.time.Clock()
        running = True
        guide_line_position_offset: int = 0

        while running and not rospy.is_shutdown():
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
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            if(self.robot_motion_active and self.robot_ready_to_move):
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)
                
                if(self.smooth_points[0][0] + guide_line_position_offset < self.smooth_points[-1][0]):
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] - 1000), (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] + 1000), 5)
                    guide_line_position_offset += 0.8
                else:
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[-1][0], self.smooth_points[0][1] - 1000), (self.smooth_points[-1][0], self.smooth_points[-1][1] + 1000), 5)
            else:
                pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0], self.smooth_points[0][1] - 1000), (self.smooth_points[0][0], self.smooth_points[0][1] + 1000), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)
            
            pygame.display.flip()
            clock.tick(60)

        # ADDED: Cleanup - stop PID controller when shutting down
        if hasattr(self, 'spline_controller'):
            self.spline_controller.stop_tracking()
        
        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


'''ROBOT MOTION - UNCHANGED'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
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
    
    def add_waypoint(self, pose, limb_name="right_hand"):
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logerr("Skipping waypoint due to IK failure")
            return False
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.1,
            max_linear_accel=0.1,
            max_rotational_speed=0.1,
            max_rotational_accel=0.1,
            max_joint_speed_ratio=0.1
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.append_waypoint(waypoint.to_msg())
        rospy.loginfo("Waypoint added at: {}".format(pose.position))
        return True

    def execute_trajectory(self, spline_waypoints):
        for pose in spline_waypoints:
            self.add_waypoint(pose)

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr("Trajectory FAILED to send")
        elif result.result:
            rospy.loginfo("Trajectory successfully executed!")
        else:
            rospy.logerr(f"Trajectory execution failed with error {result.errorId}")

if __name__ == '__main__':
    sawyerVisualizer = SawyerVisualizer()
    sawyerVisualizer.run()