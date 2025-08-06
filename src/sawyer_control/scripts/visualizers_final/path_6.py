#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
from intera_interface import Limb
from intera_core_msgs.msg import EndpointState
import std_msgs.msg

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

import numpy as np
import pygame
import threading
import time
import math


'''IMPORTANT VARIABLES'''
x_screen_limit_left: int = 200
x_screen_limit_right: int = 1720
y_screen_limit_top: int = 200
y_screen_limit_bottom: int = 1000

start_pose = Pose(position=Point(x=0.75, y=-0.3, z=0.3),
                  orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.75, y=0.5, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('positionSubscriber', anonymous=True)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display")

        # Initialize font for text display
        pygame.font.init()
        self.font_large = pygame.font.Font(None, 72)
        self.font_medium = pygame.font.Font(None, 48)
        self.font_small = pygame.font.Font(None, 36)

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.3, 0.5, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)

        self.smooth_points = list(zip(x_coords, y_coords))
        self.trail_points = [] #list of points to store the path moves by the robot when going along the trajectory

        self.icon_color = (9, 179, 54) # the real-time display position icon
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

        # Timer and tracking variables
        self.countdown_time = 0
        self.countdown_active = False
        self.motion_start_time = None
        self.elapsed_time = 0
        
        # Path tracking variables
        self.coverage_percentage = 0.0
        self.covered_spline_points = set()  # Track which spline points have been covered
        
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
            screen_x = (msg.pose.position.y * self.scale_x) + 770 #original 1340
            screen_y = (-msg.pose.position.z * self.scale_y) + 1180
            self.x = max(0, min(1920 - self.icon_radius, screen_x)) # the position on the screen
            self.y = max(0, min(1080 - self.icon_radius, screen_y)) # the position on the screen

    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def update_coverage_tracking(self):
        """Update which parts of the spline have been covered by the robot trail"""
        if not self.trail_points:
            return
            
        coverage_threshold = 5  # pixels - how close trail needs to be to "cover" a spline point
        
        # Check which spline points are covered by ANY trail point
        for i, spline_point in enumerate(self.smooth_points):
            if i not in self.covered_spline_points:
                # Check if any black trail point is close enough to this red spline point
                for trail_point in self.trail_points:
                    distance = self.calculate_distance(trail_point, spline_point)
                    if distance <= coverage_threshold:
                        self.covered_spline_points.add(i)
                        break  # Once covered, no need to check other trail points
        
        # Calculate coverage percentage - how much of red spline is covered by black trail
        if len(self.smooth_points) > 0:
            self.coverage_percentage = (len(self.covered_spline_points) / len(self.smooth_points)) * 100

    def start_countdown(self, duration=5):
        """Start a countdown timer"""
        self.countdown_time = duration
        self.countdown_active = True
        
        def countdown_thread():
            while self.countdown_time > 0 and not rospy.is_shutdown():
                time.sleep(1)
                self.countdown_time -= 1
            self.countdown_active = False
            
        threading.Thread(target=countdown_thread, daemon=True).start()

    def move_to_start_position(self):
        rospy.loginfo("Moving to start position...")
        if not self.robot_ready_to_move:
            # Start countdown
            self.start_countdown(5)
            
            # Create a threading Event to signal when the move is complete
            self.start_position_event = threading.Event()
            
            def move_and_signal():
                self.ik_motion.move_to_pose(start_pose)
                # Set the event to signal the move is complete
                self.start_position_event.set()
            
            # Start the thread for moving to start position
            self.start_position_thread = threading.Thread(target=move_and_signal)
            self.start_position_thread.start()
            
            # Start another thread to wait for completion without blocking the main thread
            def wait_for_move_completion():
                self.start_position_event.wait()
                # Use rospy.is_shutdown() to check if ROS is still running
                if not rospy.is_shutdown():
                    self.robot_ready_to_move = True
                    self.spacebar_disabled = False  # Re-enable spacebar
                    rospy.loginfo("Reached start position.")
            
            self.wait_thread = threading.Thread(target=wait_for_move_completion)
            self.wait_thread.start()

    def record_realtime_position(self):
        self.trail_points.append((int(self.x), int(self.y)))
        # Update coverage tracking every time we record a new position
        self.update_coverage_tracking()

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            self.motion_start_time = time.time()
            self.covered_spline_points.clear()  # Reset coverage tracking
            self.coverage_percentage = 0.0
            
            # Start countdown for motion
            self.start_countdown(3)
            
            self.motion_thread = threading.Thread(target=self.ik_motion.execute_trajectory, args=(self.spline_waypoints,))
            self.motion_thread.start()

    def draw_text_with_background(self, text, font, text_color, bg_color, position):
        """Draw text with a background rectangle"""
        text_surface = font.render(text, True, text_color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = position
        
        # Create background rectangle with padding
        padding = 10
        bg_rect = pygame.Rect(text_rect.x - padding, text_rect.y - padding, 
                             text_rect.width + 2*padding, text_rect.height + 2*padding)
        
        pygame.draw.rect(self.window, bg_color, bg_rect)
        pygame.draw.rect(self.window, (0, 0, 0), bg_rect, 2)  # Black border
        self.window.blit(text_surface, text_rect)

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

            # Update elapsed time during motion
            if self.robot_motion_active and self.motion_start_time:
                self.elapsed_time = time.time() - self.motion_start_time

            self.window.fill(self.background_color)
            
            # Draw the red spline path
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), ((int(point[0]), point[1])), 5)
            
            if(self.robot_motion_active and self.robot_ready_to_move): #draw real time trail when following trajectory
                self.record_realtime_position()
                for point in self.trail_points:
                    pygame.draw.circle(self.window, (0, 0, 0), (point[0], point[1]), 5)

                #draw pace line here
                if(self.smooth_points[0][0] + guide_line_position_offset < self.smooth_points[-1][0]):
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] - 1000), (self.smooth_points[0][0] + guide_line_position_offset, self.smooth_points[0][1] + 1000), 5)
                    guide_line_position_offset += 0.75
                else:
                    pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[-1][0], self.smooth_points[0][1] - 1000), (self.smooth_points[-1][0], self.smooth_points[-1][1] + 1000), 5)
            else:
                pygame.draw.line(self.window, (235, 168, 52),  (self.smooth_points[0][0], self.smooth_points[0][1] - 1000), (self.smooth_points[0][0], self.smooth_points[0][1] + 1000), 5)

            with self.lock:
                pygame.draw.circle(self.window, self.icon_color, (int(self.x), int(self.y)), self.icon_radius)

            # Draw countdown timer
            if self.countdown_active and self.countdown_time > 0:
                countdown_text = f"Starting in: {self.countdown_time}"
                self.draw_text_with_background(countdown_text, self.font_large, (255, 255, 255), (255, 0, 0), (50, 50))

            # Draw elapsed time during motion
            if self.robot_motion_active:
                elapsed_text = f"Time: {self.elapsed_time:.1f}s"
                self.draw_text_with_background(elapsed_text, self.font_medium, (255, 255, 255), (0, 150, 0), (50, 150))

            # Draw coverage percentage
            coverage_text = f"Path Coverage: {self.coverage_percentage:.1f}%"
            color = (0, 255, 0) if self.coverage_percentage > 80 else (255, 165, 0) if self.coverage_percentage > 50 else (255, 0, 0)
            self.draw_text_with_background(coverage_text, self.font_medium, (255, 255, 255), color, (50, 250))

            # Draw instructions
            if not self.robot_ready_to_move and not self.spacebar_disabled:
                instruction_text = "Press SPACEBAR to move to start position"
                self.draw_text_with_background(instruction_text, self.font_small, (255, 255, 255), (100, 100, 100), (50, 350))
            elif self.robot_ready_to_move and not self.robot_motion_active:
                instruction_text = "Press SPACEBAR to start trajectory"
                self.draw_text_with_background(instruction_text, self.font_small, (255, 255, 255), (0, 100, 200), (50, 350))

            # Draw statistics
            stats_y = 450
            if self.trail_points:
                trail_length_text = f"Trail Points: {len(self.trail_points)}"
                self.draw_text_with_background(trail_length_text, self.font_small, (255, 255, 255), (50, 50, 50), (50, stats_y))
                
                covered_points_text = f"Covered Points: {len(self.covered_spline_points)}/{len(self.smooth_points)}"
                self.draw_text_with_background(covered_points_text, self.font_small, (255, 255, 255), (50, 50, 50), (50, stats_y + 50))
            
            pygame.display.flip()
            clock.tick(60)

        pygame.quit()
        rospy.signal_shutdown("PyGame window closed")


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

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=8):
        """
        Creates a simple maze path with 90-degree turns and zigzags from left to right.
        """
        
        # Simple maze: left to right with 90-degree turns and zigzags
        control_points = np.array([
            # Start position (left side)
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            
            # Move up (90-degree turn)
            [0.75, -0.2, 0.4],
            
            # Move right (90-degree turn)  
            [0.75, 0.0, 0.4],
            
            # Move down (90-degree turn)
            [0.75, 0.1, 0.2],
            
            # Start zigzag pattern
            [0.75, 0.2, 0.35],  # up
            [0.75, 0.3, 0.2],   # down
            [0.75, 0.4, 0.35],  # up
            
            # End position (right side)
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])

        # Create waypoints for actual robot motion
        x_maze_path = control_points[:, 0]
        y_maze_path = control_points[:, 1] 
        z_maze_path = control_points[:, 2]

        self.waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                        for x, y, z in zip(x_maze_path, y_maze_path, z_maze_path)]

        # Interpolation for display - simpler approach
        y_maze_path_display = []
        z_maze_path_display = []
        
        for i in range(len(y_maze_path) - 1):
            y_interpolated = np.linspace(y_maze_path[i], y_maze_path[i+1], 100, False)
            z_interpolated = np.linspace(z_maze_path[i], z_maze_path[i+1], 100, False)
            
            y_maze_path_display.extend(y_interpolated)
            z_maze_path_display.extend(z_interpolated)

        display_waypoints = list(zip(y_maze_path_display, z_maze_path_display))

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
        """
        Executes a trajectory based on the given spline waypoints.
        """
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