#!/usr/bin/env python
# Ben's test file for Sawyer and Learning
# Created July 2, 2024


# Import the necessary Python modules
import rospy  # ROS Python API
import intera_interface  # Sawyer Python API


# Initialize our ROS node, registering it with the Master
rospy.init_node('Test_Sawyer')


# Create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right')

# Get the right limb's current joint angles
angles = limb.joint_angles()

# # Print the current joint angles
# print('\n---Current Angles---')
# print(angles)
# rospy.sleep(3)


#---MOVING TO NEUTRAL POSE---
print('\n---MOVING TO NEUTRAL POSITION---')
limb.move_to_neutral()

# Get the right limb's current joint angles now that it is in neutral
angles = limb.joint_angles()

# Print the current joint angles again
print('---Current Angles---')
print(angles)
rospy.sleep(3)


#---MOVING TO CUSTOM POSE---
# Reassign new joint angles (all zeros) which we will later command to the limb
angles['right_j0'] = 0.0
angles['right_j1'] = 0.0
angles['right_j2'] = 0.0
angles['right_j3'] = 0.0
angles['right_j4'] = 0.0
angles['right_j5'] = 0.0
angles['right_j6'] = 0.0

# Move the right arm to those joint angles
print('\n---MOVING TO ALL 0 POSITION---')
limb.move_to_joint_positions(angles)

# Print the joint angle command
print('---Current Angles---')
print(angles)
rospy.sleep(1)


#---WAVING---
# Sawyer wants to say hello, let's wave the arm
# Store the first wave position
wave_1 = {
    'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126,
    'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259
}
# Store the second wave position
wave_2 = {
    'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103,
    'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281
}

print('\n---Waving---')
# Wave three times
for _move in range(3):
    limb.move_to_joint_positions(wave_1)
    rospy.sleep(0.1)
    limb.move_to_joint_positions(wave_2)
    rospy.sleep(0.1)

limb.move_to_neutral()