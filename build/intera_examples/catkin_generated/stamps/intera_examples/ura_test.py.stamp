#!/usr/bin/env python
 
import rospy
import copy
import math
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray, Empty
from intera_interface import Limb
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_core_msgs.msg import InteractionControlCommand, EndpointState
 
class SawyerStiffnessManager:
    def __init__(self, trigger_y):
        self.trigger_y = trigger_y
        self.stiffness_triggered = False
        self.pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                    InteractionControlCommand, queue_size=10)
        self.collision_suppress_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance',
                                                      Empty, queue_size=10)
 
        # Default and high stiffness values
        self.default_stiffness = [1.0, 1.0, 1.0, 27.0, 27.0, 27.0]
        self.high_stiffness = [1200.0, 1200.0, 1200.0, 30.0, 30.0, 30.0]
        # rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_state_callback)
 
    def create_interaction_msg(self, stiffness):
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_K_impedance(stiffness)
        interaction_options.set_max_impedance([False]*6)
        interaction_options.set_interaction_control_mode([1]*6)
        interaction_options.set_in_endpoint_frame(False)
        interaction_options.set_force_command([0.0]*6)
        interaction_options.set_K_nullspace([5.0]*7)
        interaction_options.set_disable_damping_in_force_control(False)
        interaction_options.set_disable_reference_resetting(False)
        return interaction_options.to_msg()
 
    # def endpoint_state_callback(self, msg):
    #     if self.stiffness_triggered:
    #         return
    #     current_y = msg.pose.position.y
    #     buffer = 0.04
    #     if self.trigger_y - buffer <= current_y <= self.trigger_y:
    #         rospy.loginfo("Approaching contact zone, increasing stiffness.")
    #         self.pub.publish(self.create_interaction_msg(self.high_stiffness))
    #         rospy.sleep(3.0)
    #         self.pub.publish(self.create_interaction_msg(self.default_stiffness))
    #         self.stiffness_triggered = True
 
    def start(self):
        threading.Thread(target=self._suppress_collision_loop, daemon=True).start()
        # threading.Thread(target=self._send_default_loop, daemon=True).start()
 
    def _suppress_collision_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.collision_suppress_pub.publish(Empty())
            rate.sleep()
 
    def _send_default_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.stiffness_triggered:
            self.pub.publish(self.create_interaction_msg(self.default_stiffness))
            rate.sleep()
 
def move_to_home(limb, timeout=15.0, speed=0.1):
    try:
        neutral_pose = rospy.get_param("named_poses/{0}/poses/neutral".format(limb.name))
    except KeyError:
        rospy.logerr(("Get neutral pose failed, arm: \"{0}\".").format(limb.name))
        return
    angles = dict(list(zip(limb.joint_names(), neutral_pose)))
    angles.update(right_j0 = 165/180*math.pi,
                  right_j5 = angles['right_j5'] - math.pi/2,
                  right_j6 = angles['right_j6'] - math.pi)
    limb.set_joint_position_speed(speed)
    return limb.move_to_joint_positions(angles, timeout)
 
def interpolate_poses(start_pose, end_pose, steps):
    interpolated = []
    for i in range(1, steps + 1):
        pose = copy.deepcopy(start_pose)
        alpha = float(i) / steps
        pose.position.x = (1 - alpha) * start_pose.position.x + alpha * end_pose.position.x
        pose.position.y = (1 - alpha) * start_pose.position.y + alpha * end_pose.position.y
        pose.position.z = (1 - alpha) * start_pose.position.z + alpha * end_pose.position.z
        interpolated.append(pose)
    return interpolated
 
def main():
    rospy.init_node('impedance_cartesian_trajectory')
 
    limb = Limb()
    tip_name = "right_hand"
 
    rospy.loginfo("Moving to neutral...")
    move_to_home(limb=limb)
    rospy.sleep(1.0)
 
    endpoint_state = limb.tip_state(tip_name)
    start_pose = endpoint_state.pose
    end_pose = copy.deepcopy(start_pose)
    end_pose.position.z -= 0.29
    
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(True) # Enable interaction controller

    # Soft compliance mostly in Z direction (downward)
    interaction_options.set_K_impedance([100.0, 100.0, 100.0, 40.0, 40.0, 40.0]) # [N/m, Nm/rad]

    # Optional: allow free movement in certain axes (disable max impedance)
    interaction_options.set_max_impedance([False, False, False, False, False, False])

    # Mode: 1 = Impedance (or try 3 = Impedance + force limit)
    interaction_options.set_interaction_control_mode([1, 1, 1, 1, 1, 1])
    # interaction_options.set_max_force_threshold([20.0]*6)
    # interaction_options.set_max_torque_threshold([10.0]*6)

    # interaction_options.set_interaction_control_mode([3, 3, 3, 1, 1, 1])
    # interaction_options.set_force_command([100.0, 100.0, 100.0, 20.0, 20.0, 20.0])

    # # Optional: set endpoint frame (if needed)
    # interaction_options.set_endpoint_name("right_hand")
    # interaction_options.set_in_endpoint_frame(True)

    # Optional: prevent sudden jerk when applying new settings
    interaction_options.set_disable_reference_resetting(True)  
    interaction_options.set_disable_damping_in_force_control(True)

    # Start impedance controller monitoring
    # stiffness_manager = SawyerStiffnessManager(trigger_y=start_pose.position.y)
    # stiffness_manager.start()
 
    # Interpolate poses
    waypoints = interpolate_poses(start_pose, end_pose, steps=30)
 
    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj_options.interaction_params = interaction_options.to_msg()
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)
 
    wpt_opts = MotionWaypointOptions(
        max_linear_speed=0.1,
        max_linear_accel=0.1,
        max_rotational_speed=0.1,
        max_rotational_accel=0.1,
        max_joint_speed_ratio=0.1,
    )
 
    rospy.loginfo("Moving down with impedance control...")
    for pose in waypoints:
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        joint_angles = limb.joint_ordered_angles()
        waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
        waypoint.set_cartesian_pose(pose_stamped, tip_name, joint_angles)
        traj.append_waypoint(waypoint.to_msg())
 
    #result = traj.send_trajectory(timeout=15.0)
    # if result and result.result:
    #     rospy.loginfo("Descent complete.")
    # else:
    #     rospy.logwarn("Descent failed or timed out.")
 
    rospy.sleep(5.0)
 
    rospy.loginfo("Returning to home...")
    move_to_home(limb=limb)
    rospy.sleep(2.0)
 
if __name__ == '__main__':
    main()

 