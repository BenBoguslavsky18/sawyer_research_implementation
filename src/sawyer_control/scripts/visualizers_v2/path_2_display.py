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
from scipy.interpolate import CubicSpline

import numpy as np
import random
import pygame
import threading


'''IMPORTANT VARIABLES'''
start_pose = Pose(position=Point(x=0.6, y=-0.6, z=0.3),
                  orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
end_pose = Pose(position=Point(x=0.6, y=0.2, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


class SawyerVisualizer:
    def __init__(self):
        rospy.init_node('positionSubscriber', anonymous=True)
        self.stiffness_pub = rospy.Publisher('/stiffness_update', std_msgs.msg.String, queue_size=10)

        pygame.init()
        self.window = pygame.display.set_mode((1920, 1420))
        self.background_color = (2, 114, 212)
        pygame.display.set_caption("Sawyer Real-Time Display")

        self.ik_motion = IKMotionWaypoint()
        self.spline_controlpoints, self.spline_waypoints, self.display_waypoints, self.spline_waypoints_false = self.ik_motion.generate_spline_waypoints(start_pose, end_pose)

        x_coords = [p[0] for p in self.display_waypoints]
        y_coords = [p[1] for p in self.display_waypoints]
        x_coords = self.convert_to_pixels_x(x_coords, -0.6, 0.2, 200, 1720)
        y_coords = self.convert_to_pixels_y(y_coords, 0.1, 0.5, 200, 1000)

        self.smooth_points = list(zip(x_coords, y_coords))

        self.rect_color = (9, 179, 54)
        self.rect_width = 50
        self.rect_height = 50
        self.x = 800
        self.y = 1000
        self.scale_x = 1900
        self.scale_y = 1950
        self.lock = threading.Lock()

        self.subscriber = rospy.Subscriber("/robot/limb/right/endpoint_state",
                                           EndpointState,
                                           self.position_callback)

        self.robot_motion_active = False
        self.robot_ready_to_move = False  # New flag for readiness
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
            screen_x = (msg.pose.position.y * self.scale_x) + 1325
            screen_y = (-msg.pose.position.z * self.scale_y) + 1150
            self.x = max(0, min(1920 - self.rect_width, screen_x))
            self.y = max(0, min(1080 - self.rect_height, screen_y))

    def move_to_start_position(self):
        rospy.loginfo("Moving to start position...")
        self.ik_motion.move_to_pose(start_pose)  # Move to the start pose
        rospy.loginfo("Reached start position.")
        self.robot_ready_to_move = True  # Set readiness flag

    def start_robot_motion(self):
        if self.robot_ready_to_move and not self.robot_motion_active:
            self.robot_motion_active = True
            self.motion_thread = threading.Thread(target=self.execute_with_stiffness_updates, daemon=True)
            self.motion_thread.start()

    def execute_with_stiffness_updates(self):
        rospy.loginfo("Starting spline trajectory...")
        for i, pose in enumerate(self.ik_motion.waypoints_false):
            if self.is_false_trajectory(i):
                self.update_stiffness([1000.0, 1000.0, 1000.0, 50.0, 50.0, 50.0])
            else:
                self.update_stiffness([500.0, 30.0, 30.0, 30.0, 30.0, 30.0])
            self.ik_motion.add_waypoint(pose)
        self.ik_motion.execute_trajectory()
        self.robot_motion_active = False
        rospy.loginfo("Spline trajectory completed.")

    def update_stiffness(self, stiffness):
        stiffness_msg = std_msgs.msg.String()
        stiffness_msg.data = str(stiffness)
        self.stiffness_pub.publish(stiffness_msg)
        rospy.loginfo(f"Stiffness updated: {stiffness}")

    def is_false_trajectory(self, index):
        return 1 <= index <= 3

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and not self.robot_motion_active:
                        if not self.robot_ready_to_move:  # First press: move to start
                            self.move_to_start_position()
                        else:  # Second press: start motion
                            self.start_robot_motion()

            self.window.fill(self.background_color)
            for point in self.smooth_points:
                pygame.draw.circle(self.window, (255, 0, 0), (int(point[0]), int(point[1])), 5)
            with self.lock:
                pygame.draw.rect(self.window,
                                 self.rect_color,
                                 (int(self.x), int(self.y),
                                  self.rect_width, self.rect_height))
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

    def generate_spline_waypoints(self, start_pose, end_pose, num_points=10):
        control_points = np.array([
            [start_pose.position.x, start_pose.position.y, start_pose.position.z],
            [0.6, -0.4, random.uniform(0.1, 0.5)],
            [0.6, -0.2, random.uniform(0.1, 0.5)],
            [0.6, 0.0, random.uniform(0.1, 0.5)],
            [end_pose.position.x, end_pose.position.y, end_pose.position.z]
        ])

        modified_control_point_index = random.randint(1, 3)
        modified_control_point_value = control_points[modified_control_point_index, 2]
        coordinate_change = random.uniform(0.1, 0.2)

        false_control_points = control_points.copy()
        if modified_control_point_value <= 0.3:
            false_control_points[modified_control_point_index, 2] = modified_control_point_value + coordinate_change
        else:
            false_control_points[modified_control_point_index, 2] = modified_control_point_value - coordinate_change

        t = np.linspace(0, 1, len(control_points))
        t_spline = np.linspace(0, 1, num_points)
        t_spline_display = np.linspace(0, 1, 1000)

        x_spline = CubicSpline(t, control_points[:, 0])(t_spline)
        y_spline = CubicSpline(t, control_points[:, 1])(t_spline)
        z_spline = CubicSpline(t, control_points[:, 2])(t_spline)

        y_spline_display = CubicSpline(t, control_points[:, 1])(t_spline_display)
        z_spline_display = CubicSpline(t, control_points[:, 2])(t_spline_display)

        t_false = np.linspace(0, 1, len(false_control_points))
        x_spline_false = CubicSpline(t_false, false_control_points[:, 0])(t_spline)
        y_spline_false = CubicSpline(t_false, false_control_points[:, 1])(t_spline)
        z_spline_false = CubicSpline(t_false, false_control_points[:, 2])(t_spline)

        self.waypoints = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                          for x, y, z in zip(x_spline, y_spline, z_spline)]

        self.waypoints_false = [Pose(position=Point(x=x, y=y, z=z), orientation=start_pose.orientation)
                                for x, y, z in zip(x_spline_false, y_spline_false, z_spline_false)]

        display_waypoints = list(zip(y_spline_display, z_spline_display))
        return control_points, self.waypoints, display_waypoints, self.waypoints_false

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

    def execute_trajectory(self):
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
