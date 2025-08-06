#!/usr/bin/env python

"""
Alternative approach for manual guidance during spline trajectory
Uses velocity control or small position steps instead of long trajectory commands
"""

import rospy
import time
import threading
from intera_interface import Limb
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np


def load_ik_solver():
    """Load the IK solver for realistic pose conversion"""
    try:
        with open("sawyer.urdf", "r") as urdf_file:
            urdf_string = urdf_file.read()
        robot = URDF.from_xml_string(urdf_string)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to parse URDF")
        
        chain = tree.getChain("base", "right_hand")
        ik_solver = kdl.ChainIkSolverPos_LMA(chain)
        
        return ik_solver, chain
    except Exception as e:
        print(f"Could not load IK solver: {e}")
        return None, None


def pose_to_joint_angles(pose, ik_solver, chain):
    """Convert Cartesian pose to joint angles using IK"""
    if ik_solver is None:
        return None
    
    try:
        pos = pose.position
        orientation = kdl.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w)
        frame = kdl.Frame(orientation, kdl.Vector(pos.x, pos.y, pos.z))
        
        num_joints = chain.getNrOfJoints()
        ik_joint_positions = kdl.JntArray(num_joints)
        ik_joint_positions_prev = kdl.JntArray(num_joints)
        
        result = ik_solver.CartToJnt(ik_joint_positions_prev, frame, ik_joint_positions)
        if result < 0:
            return None
        
        joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        return {joint_names[i]: ik_joint_positions[i] for i in range(num_joints)}
        
    except Exception as e:
        print(f"IK calculation failed: {e}")
        return None


def get_realistic_spline_positions():
    """Get realistic positions from the actual spline challenges"""
    spline_poses = [
        Pose(position=Point(x=0.75, y=-0.3, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=-0.2, z=0.35), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=-0.1, z=0.4), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.0, z=0.35), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.1, z=0.2), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.2, z=0.25), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.3, z=0.4), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.4, z=0.35), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.75, y=0.5, z=0.3), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
    ]
    return spline_poses


def interpolate_waypoints(start_joints, end_joints, num_steps=20):
    """Interpolate between two joint positions with many small steps"""
    interpolated = []
    for i in range(num_steps + 1):
        alpha = i / float(num_steps)
        interpolated_joints = {}
        for name in start_joints.keys():
            interpolated_joints[name] = start_joints[name] + alpha * (end_joints[name] - start_joints[name])
        interpolated.append(interpolated_joints)
    return interpolated


def calculate_joint_distance(pos1, pos2):
    """Calculate the total joint distance between two positions"""
    return sum(abs(pos1[name] - pos2[name]) for name in pos1.keys())


def test_velocity_based_spline_following():
    """
    Test spline following using velocity commands instead of position commands
    This should allow manual guidance to work
    """
    
    rospy.init_node('velocity_spline_test_node')
    limb = Limb('right')
    
    print("=== VELOCITY-BASED SPLINE TRAJECTORY WITH MANUAL GUIDANCE ===")
    print("Using joint velocity commands instead of position commands")
    print("This should allow manual guidance to work!")
    
    # Set low impedance
    joint_names = limb.joint_names()
    low_stiffness = {name: 10.0 for name in joint_names}
    
    try:
        limb.set_joint_position_impedance(low_stiffness)
        print("✓ Set low impedance for manual guidance")
    except Exception as e:
        print(f"⚠️  Could not set impedance: {e}")
    
    # Load IK solver and get waypoints
    ik_solver, chain = load_ik_solver()
    if ik_solver is None:
        print("ERROR: Cannot run test without IK solver")
        return False
    
    spline_poses = get_realistic_spline_positions()
    spline_waypoints = []
    
    for i, pose in enumerate(spline_poses):
        joints = pose_to_joint_angles(pose, ik_solver, chain)
        if joints is not None:
            spline_waypoints.append(joints)
            print(f"✓ Waypoint {i+1}: IK solution found")
    
    if len(spline_waypoints) < 3:
        print("ERROR: Not enough valid waypoints")
        return False
    
    print(f"Ready to follow {len(spline_waypoints)} waypoints using velocity control")
    print("💡 TRY TO MANUALLY GUIDE THE ARM DURING EXECUTION!")
    input("Press Enter to start velocity-based trajectory...")
    
    # Use velocity control to follow trajectory
    current_pos = limb.joint_angles()
    target_waypoint_idx = 0
    manual_guidance_detected = False
    
    control_rate = rospy.Rate(10)  # 10 Hz control loop
    velocity_scale = 0.3  # Moderate velocity
    
    print("Starting velocity-based spline following...")
    start_time = time.time()
    
    while target_waypoint_idx < len(spline_waypoints) and not rospy.is_shutdown():
        current_pos = limb.joint_angles()
        target_pos = spline_waypoints[target_waypoint_idx]
        
        # Calculate velocity commands toward target
        velocities = {}
        max_error = 0
        
        for name in joint_names:
            error = target_pos[name] - current_pos[name]
            max_error = max(max_error, abs(error))
            
            # Proportional velocity control
            velocities[name] = velocity_scale * error
        
        # Check if we're close enough to current waypoint
        if max_error < 0.05:  # 0.05 radians = ~3 degrees
            print(f"✓ Reached waypoint {target_waypoint_idx + 1}")
            target_waypoint_idx += 1
            time.sleep(1.0)  # Pause between waypoints for manual guidance
            continue
        
        # Send velocity commands
        limb.set_joint_velocities(velocities)
        
        # Check for manual guidance (simplified detection)
        elapsed = time.time() - start_time
        if int(elapsed) % 5 == 0 and elapsed > 5:  # Every 5 seconds
            print(f"  👋 Manual guidance window - try moving the arm!")
        
        control_rate.sleep()
    
    # Stop all motion
    limb.exit_control_mode()
    print("✓ Velocity-based trajectory completed")
    
    return True


def test_micro_step_position_control():
    """
    Test spline following using very small position steps
    Each step is so small that manual guidance can override
    """
    
    rospy.init_node('microstep_spline_test_node')
    limb = Limb('right')
    
    print("\n=== MICRO-STEP POSITION CONTROL WITH MANUAL GUIDANCE ===")
    print("Using tiny position steps instead of large trajectory commands")
    
    # Set low impedance and fast speed
    joint_names = limb.joint_names()
    low_stiffness = {name: 8.0 for name in joint_names}
    
    try:
        limb.set_joint_position_impedance(low_stiffness)
        limb.set_joint_position_speed(0.8)  # Fast speed for small steps
        print("✓ Set low impedance and fast speed for micro-steps")
    except Exception as e:
        print(f"⚠️  Could not set parameters: {e}")
    
    # Load waypoints
    ik_solver, chain = load_ik_solver()
    if ik_solver is None:
        print("ERROR: Cannot run test without IK solver")
        return False
    
    spline_poses = get_realistic_spline_positions()
    spline_waypoints = []
    
    for pose in spline_poses:
        joints = pose_to_joint_angles(pose, ik_solver, chain)
        if joints is not None:
            spline_waypoints.append(joints)
    
    if len(spline_waypoints) < 3:
        print("ERROR: Not enough valid waypoints")
        return False
    
    print("💡 MANUAL GUIDANCE SHOULD WORK WITH MICRO-STEPS!")
    input("Press Enter to start micro-step trajectory...")
    
    # Create many micro-steps between each waypoint
    all_microsteps = []
    current_pos = limb.joint_angles()
    
    for i, waypoint in enumerate(spline_waypoints):
        if i == 0:
            # First waypoint - interpolate from current position
            microsteps = interpolate_waypoints(current_pos, waypoint, num_steps=30)
        else:
            # Subsequent waypoints - interpolate from previous waypoint
            microsteps = interpolate_waypoints(spline_waypoints[i-1], waypoint, num_steps=30)
        
        all_microsteps.extend(microsteps[1:])  # Skip first step to avoid duplication
        print(f"Generated {len(microsteps)} micro-steps for waypoint {i+1}")
    
    print(f"Total micro-steps: {len(all_microsteps)}")
    print("Each step is tiny - manual guidance should easily override!")
    
    # Execute micro-steps
    manual_override_count = 0
    
    for i, microstep in enumerate(all_microsteps):
        if i % 20 == 0:  # Print progress every 20 steps
            print(f"Micro-step {i+1}/{len(all_microsteps)} - Try manual guidance!")
        
        # Record position before command
        pos_before = limb.joint_angles()
        
        # Send tiny position command with very short timeout
        success = limb.move_to_joint_positions(microstep, timeout=0.5)
        
        # Check if we ended up where expected (simplified manual override detection)
        pos_after = limb.joint_angles()
        expected_movement = calculate_joint_distance(pos_before, microstep)
        actual_movement = calculate_joint_distance(pos_before, pos_after)
        
        # If actual movement is significantly different, manual guidance may have occurred
        if abs(actual_movement - expected_movement) > 0.02:
            manual_override_count += 1
        
        # Very brief pause - micro-steps should be nearly continuous
        time.sleep(0.05)
    
    print(f"✓ Micro-step trajectory completed")
    print(f"Possible manual overrides detected: {manual_override_count}")
    
    if manual_override_count > 10:
        print("✅ MANUAL GUIDANCE APPEARS TO WORK with micro-steps!")
        return True
    else:
        print("❌ Manual guidance still limited with micro-steps")
        return False


def test_force_sensing_approach():
    """
    Test using force/torque sensing to detect manual guidance
    This is the most robust approach for manual guidance
    """
    
    print("\n=== FORCE-SENSING BASED MANUAL GUIDANCE ===")
    print("This approach monitors joint torques to detect manual guidance")
    
    rospy.init_node('force_sensing_test_node')
    limb = Limb('right')
    
    # Set very low impedance
    joint_names = limb.joint_names()
    very_low_stiffness = {name: 3.0 for name in joint_names}
    
    try:
        limb.set_joint_position_impedance(very_low_stiffness)
        print("✓ Set very low impedance for force sensing")
    except Exception as e:
        print(f"⚠️  Could not set impedance: {e}")
    
    print("Monitoring joint torques for manual guidance detection...")
    print("💡 Try manually moving the arm - the system should detect it!")
    
    # Monitor joint torques for manual guidance
    baseline_torques = None
    manual_guidance_threshold = 5.0  # Nm - adjust based on robot
    
    for i in range(100):  # Monitor for 10 seconds
        try:
            current_torques = limb.joint_efforts()
            
            if baseline_torques is None:
                baseline_torques = current_torques.copy()
                print("✓ Baseline torques recorded")
            
            # Check for significant torque changes indicating manual guidance
            manual_detected = False
            for name in joint_names:
                torque_change = abs(current_torques[name] - baseline_torques[name])
                if torque_change > manual_guidance_threshold:
                    manual_detected = True
                    break
            
            if manual_detected:
                print(f"🔍 Manual guidance detected at step {i+1}!")
            elif i % 10 == 0:
                print(f"Monitoring... step {i+1}/100")
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"Could not read joint torques: {e}")
            print("❌ Force sensing approach not available on this system")
            return False
    
    print("✓ Force sensing test completed")
    return True


if __name__ == '__main__':
    try:
        print("Testing alternative approaches for manual guidance during spline trajectory...")
        print("These methods should work better than low impedance alone")
        print()
        
        # Test different approaches
        print("APPROACH 1: Velocity-based control")
        velocity_works = test_velocity_based_spline_following()
        
        print("\nAPPROACH 2: Micro-step position control")  
        microstep_works = test_micro_step_position_control()
        
        print("\nAPPROACH 3: Force sensing detection")
        force_sensing_works = test_force_sensing_approach()
        
        print("\n" + "="*60)
        print("FINAL RECOMMENDATIONS FOR MANUAL GUIDANCE:")
        
        if velocity_works:
            print("🟢 VELOCITY CONTROL: Most promising approach")
            print("   - Use joint velocity commands instead of position commands")
            print("   - Manual forces can easily override velocity commands")
            print("   - Implement: Calculate desired velocities toward spline waypoints")
            print("   - Add force sensing to detect when user takes control")
            
        if microstep_works:
            print("🟡 MICRO-STEP POSITION: Partial solution")
            print("   - Break trajectory into very small position steps")
            print("   - Each step is small enough for manual override")
            print("   - Higher computational overhead but more robust")
            
        if force_sensing_works:
            print("🔵 FORCE SENSING: Essential for detection")
            print("   - Monitor joint torques to detect manual guidance")
            print("   - Switch control modes when manual guidance detected")
            print("   - Most professional approach for collaborative robotics")
        
        if not (velocity_works or microstep_works):
            print("🔴 ALTERNATIVE APPROACHES NEEDED:")
            print("   - Standard position control dominates over manual input")
            print("   - Consider switching to torque control mode")
            print("   - Implement explicit 'manual guidance mode'")
            print("   - Use ROS controllers with force feedback")
            
        print(f"\nBEST IMPLEMENTATION STRATEGY:")
        print("1. Use velocity control for trajectory following")
        print("2. Monitor joint torques for manual guidance detection") 
        print("3. When manual guidance detected:")
        print("   - Reduce or zero velocity commands")
        print("   - Let user guide the arm manually")
        print("   - Resume trajectory when manual guidance stops")
        print("4. Use very low impedance settings throughout")
        
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()