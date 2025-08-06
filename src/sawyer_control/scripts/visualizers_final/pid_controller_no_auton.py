#!/usr/bin/env python

#  Sawyer path visualizer with guided spline following

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState

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


'''Spline Following Controller - Works WITH position control'''
class SplineFollowingController:
    def __init__(self, spline_trajectory, ik_motion):
        """
        spline_trajectory: List of (x, y, z) waypoints defining the desired spline path
        ik_motion: Reference to IKMotionWaypoint for sending position commands
        """
        self.spline_trajectory = np.array(spline_trajectory)
        self.ik_motion = ik_motion
        
        # Progress tracking
        self.current_spline_index = 0
        self.lookahead_distance = 0.03  # 3cm lookahead along spline
        self.position_tolerance = 0.02   # 2cm tolerance to advance to next point
        
        # Current robot state
        self.current_position = None
        self.target_spline_point = None
        self.distance_from_target = 0.0
        self.lock = threading.Lock()
        
        # Control state
        self.control_active = False
        self.control_thread = None
        self.last_command_time = 0
        self.command_rate_limit = 0.5  # Send new commands every 0.5 seconds max
        
        rospy.loginfo(f"Spline following controller initialized with {len(self.spline_trajectory)} waypoints")
    
    def update_robot_position(self, position):
        """Called by the main visualizer to update robot position"""
        with self.lock:
            self.current_position = np.array(position)
            if len(self.spline_trajectory) > 0:
                self.update_target_point()
    
    def update_target_point(self):
        """Find the next target point on the spline for the robot to move to"""
        if self.current_position is None:
            return
        
        # Find closest point on spline (Y-Z distance only)
        current_pos_yz = self.current_position[1:3]  # [y, z] only
        
        # Search around current progress
        search_start = max(0, self.current_spline_index - 2)
        search_end = min(len(self.spline_trajectory), self.current_spline_index + 10)
        
        search_trajectory = self.spline_trajectory[search_start:search_end]
        search_trajectory_yz = search_trajectory[:, 1:3]  # Y-Z only
        
        # Find closest point in search window
        distances = cdist([current_pos_yz], search_trajectory_yz)[0]
        local_closest_idx = np.argmin(distances)
        closest_global_idx = search_start + local_closest_idx
        self.distance_from_target = distances[local_closest_idx]
        
        # Update progress if we've moved forward
        if closest_global_idx >= self.current_spline_index:
            self.current_spline_index = closest_global_idx
        
        # Set target point with lookahead
        lookahead_idx = min(len(self.spline_trajectory) - 1, 
                           self.current_spline_index + 2)  # Look 2 points ahead
        
        self.target_spline_point = self.spline_trajectory[lookahead_idx]
    
    def should_send_new_command(self):
        """Check if enough time has passed to send a new position command"""
        current_time = rospy.get_time()
        if current_time - self.last_command_time > self.command_rate_limit:
            self.last_command_time = current_time
            return True
        return False
    
    def send_target_position(self):
        """Send the robot to the current target point on the spline"""
        if self.target_spline_point is None:
            return
        
        # Create pose for target point
        target_pose = Pose(
            position=Point(
                x=self.target_spline_point[0],
                y=self.target_spline_point[1],
                z=self.target_spline_point[2]
            ),
            orientation=start_pose.orientation  # Keep same orientation
        )
        
        # Send position command (non-blocking)
        try:
            self.ik_motion.move_to_pose_async(target_pose)
            rospy.loginfo(f"Sent target: [{self.target_spline_point[0]:.3f}, "
                         f"{self.target_spline_point[1]:.3f}, {self.target_spline_point[2]:.3f}]")
        except Exception as e:
            rospy.logwarn(f"Failed to send target position: {e}")
    
    def control_loop(self):
        """Main control loop - continuously update target position"""
        rate = rospy.Rate(5)  # 5 Hz control rate
        
        rospy.loginfo("Starting spline following control...")
        
        while self.control_active and not rospy.is_shutdown():
            with self.lock:
                if self.current_position is None or self.target_spline_point is None:
                    rate.sleep()
                    continue
                
                current_pos = self.current_position.copy()
                target_point = self.target_spline_point.copy()
                progress_percent = (self.current_spline_index / len(self.spline_trajectory)) * 100
                distance = self.distance_from_target
            
            # Send new position command if enough time has passed
            if self.should_send_new_command():
                self.send_target_position()
            
            # Logging
            rospy.loginfo(f"Progress: {progress_percent:.1f}% (index {self.current_spline_index}), "
                         f"Current: [{current_pos[1]:.3f}, {current_pos[2]:.3f}], "
                         f"Target: [{target_point[1]:.3f}, {target_point[2]:.3f}], "
                         f"Y-Z distance: {distance:.4f}m")
            
            # Check if we've completed the spline
            if self.current_spline_index >= len(self.spline_trajectory) - 3:
                rospy.loginfo("Spline following complete!")
                self.control_active = False
                break
            
            rate.sleep()
    
    def start_following(self):
        """Start spline following"""
        if self.control_active:
            rospy.logwarn("Spline following already active!")
            return
        
        rospy.loginfo("Starting spline following...")
        self.control_active = True
        
        # Reset progress
        self.current_spline_index = 0
        self.last_command_time = 0
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
    
    def stop_following(self):
        """Stop spline following"""
        rospy.loginfo("Stopping spline following...")
        self.control_active = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join()


'''MODIFIED: Visualizer for Spline Following'''
class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('positionSubscriber', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Spline Following")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        # Convert spline waypoints to robot coordinates for following controller
        robot_trajectory = []
        for waypoint in self.spline_waypoints:
            robot_trajectory.append([waypoint.position.x, waypoint.position.y, waypoint.position.z])
        
        # Initialize the spline following controller
        self.spline_controller = SplineFollowingController(robot_trajectory, self.ik_motion)

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

        self.spline_following_active = False
        self.robot_ready_to_move = False
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
            
            # Update spline controller with current robot position
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
                    rospy.loginfo("Reached start position. Ready to follow spline!")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))

    def start_spline_following(self):
        """Start spline following (robot guides itself along the path)"""
        if self.robot_ready_to_move and not self.spline_following_active:
            self.spline_following_active = True
            
            # Start spline following - robot will continuously move to next spline points
            self.spline_controller.start_following()
            rospy.loginfo("Spline following started! Robot will guide itself along the path.")

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if not self.robot_ready_to_move and not self.spacebar_disabled:  # First press
                            self.spacebar_disabled = True
                            self.move_to_start_position()
                        elif not self.spline_following_active:  # Second press
                            self.start_spline_following()

            self.window.fill(self.background_color)
            
            # Draw the spline path
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            # Draw robot trail
            if self.spline_following_active and self.robot_ready_to_move:
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)
                
                # Show start line
                pygame.draw.line(self.window, (235, 168, 52), 
                               (self.smooth_points[0][0], self.smooth_points[0][1] - 1000), 
                               (self.smooth_points[0][0], self.smooth_points[0][1] + 1000), 5)
            else:
                # Show start line
                pygame.draw.line(self.window, (235, 168, 52), 
                               (self.smooth_points[0][0], self.smooth_points[0][1] - 1000), 
                               (self.smooth_points[0][0], self.smooth_points[0][1] + 1000), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)
            
            pygame.display.flip()
            clock.tick(60)

        # Cleanup - stop spline controller when shutting down
        if hasattr(self, 'spline_controller'):
            self.spline_controller.stop_following()
        
        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


'''ROBOT MOTION - Enhanced with Async Commands'''
class IKMotionWaypoint:
    def __init__(self, limb="right"):
        self._limb = Limb(limb)
        self.sawyerchain = self.load_chain("sawyer.urdf", "base", "right_hand")
        self.num_joints = self.sawyerchain.getNrOfJoints()
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.sawyerchain)
        self.ik_joint_positions = kdl.JntArray(self.num_joints)
        self.ik_joint_positions_prev = kdl.JntArray(self.num_joints)
        self.traj = MotionTrajectory(limb=self._limb)
        rospy.loginfo("IK and Motion setup complete.")

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
        """Blocking move to pose - waits for completion"""
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

    def move_to_pose_async(self, pose, limb_name="right_hand"):
        """Non-blocking move to pose - sends command and returns immediately"""
        joint_angles = self.calculate_ik(pose)
        if joint_angles is None:
            rospy.logwarn("Failed to send pose: IK solution not found")
            return
        
        wpt_opts = MotionWaypointOptions(
            max_linear_speed=0.15,  # Slightly faster for continuous following
            max_linear_accel=0.15,
            max_rotational_speed=0.15,
            max_rotational_accel=0.15,
            max_joint_speed_ratio=0.15
        )
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=self._limb)
        waypoint.set_joint_angles(joint_angles, limb_name)
        self.traj.clear_waypoints()
        self.traj.append_waypoint(waypoint.to_msg())
        
        # Send trajectory without waiting for result
        self.traj.send_trajectory(timeout=0.1)  # Short timeout for async operation

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=40):
        """Generate spline for visualization and following reference"""
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

if __name__ == '__main__':
    sawyerVisualizer = SawyerVisualizer()
    sawyerVisualizer.run()