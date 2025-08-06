#!/usr/bin/env python

import rospy
import csv
import os
from datetime import datetime
from intera_core_msgs.msg import EndpointState
from std_msgs.msg import Float64MultiArray

# Define CSV file path
save_dir = os.path.expanduser("~/sawyer_effort_logs")
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

file_path = os.path.join(save_dir, f"sawyer_effort_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv")

# Open CSV file and write headers
with open(file_path, mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Joint_Torques"])

print(f"Logging effort values to: {file_path}")

def endpoint_callback(msg):
    """ Callback function for end-effector force/torque data """
    timestamp = rospy.Time.now().to_sec()
    forces = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
    torques = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    with open(file_path, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp] + forces + torques + ["-"])  # Placeholder for joint torques

def torque_callback(msg):
    """ Callback function for joint torque data """
    timestamp = rospy.Time.now().to_sec()
    torques = list(msg.data)

    with open(file_path, mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp] + ["-"] * 6 + [torques])  # Placeholder for end-effector forces

def main():
    rospy.init_node("sawyer_effort_recorder", anonymous=True)
    
    # Subscribe to topics
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, endpoint_callback)
    rospy.Subscriber("/robot/limb/right/torque_data", Float64MultiArray, torque_callback)

    rospy.loginfo("Recording effort values... Press CTRL+C to stop.")
    rospy.spin()

if __name__ == "__main__":
    main()

