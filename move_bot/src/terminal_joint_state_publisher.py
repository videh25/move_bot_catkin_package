#!/usr/bin/env python3

import threading
import rospy
from sensor_msgs.msg import JointState
import joint_state_publisher

# A general publisher for changing values of ANY urdf model loaded on rviz from terminal
# Uses joint_state_publisher

def inputstr_to_floatarray(input_str):
    return [float(x) for x in input_str.split()]

def terminal_joint_val_publisher():
    rospy.init_node('terminal_joint_state_publisher')
    jsp = joint_state_publisher.JointStatePublisher()

    threading.Thread(target=jsp.loop).start()

    while not rospy.is_shutdown():
        msg = JointState()
        msg.name = [x for x in jsp.free_joints.keys()]

        print("The free joints are " + str([x for x in jsp.free_joints.keys()]))
        print("Print enter to pass all values as 0s.")
        print("Enter 0 in any one prompt to simply close the publisher.")
        print()        
        position_input = inputstr_to_floatarray(input("Enter the positions of " + str(len(jsp.free_joints)) +" free joints: "))
        if len(position_input) == 1 and position_input[0] == 0:
            print("Press Ctrl+C to close the publisher")
            break
        elif len(position_input) == 0:
            position_input = len(jsp.free_joints) * [0.0]
        elif len(position_input) != len(jsp.free_joints):
            print("Enter exactly "+ str(len(jsp.free_joints)) + " values.")
            print()
            print("----------------------------------------------")
            print()
            continue

        velocity_input = inputstr_to_floatarray(input("Enter the velocities of " + str(len(jsp.free_joints)) +" free joints: "))
        if len(velocity_input) == 1 and velocity_input[0] == 0:
            print("Press Ctrl+C to close the publisher")
            break
        elif len(velocity_input) == 0:
            velocity_input = len(jsp.free_joints) * [0.0]
        elif len(velocity_input) != len(jsp.free_joints):
            print("Enter exactly "+ str(len(jsp.free_joints)) + " values.")
            print("Enter exactly "+ str(len(jsp.free_joints)) + " values.")
            print()
            print("----------------------------------------------")
            print()
            continue

            
        effort_input = inputstr_to_floatarray(input("Enter the efforts of " + str(len(jsp.free_joints)) +" free joints: "))
        if len(effort_input) == 1 and effort_input[0] == 0:
            print("Press Ctrl+C to close the publisher")
            break
        elif len(effort_input) == 0:
            effort_input = len(jsp.free_joints) * [0.0]
        elif len(effort_input) != len(jsp.free_joints):
            print("Enter exactly "+ str(len(jsp.free_joints)) + " values.")
            print("Enter exactly "+ str(len(jsp.free_joints)) + " values.")
            print()
            print("----------------------------------------------")
            print()
            continue

        print()
        print("----------------------------------------------")
        print()
        
        msg.position = position_input
        msg.velocity = velocity_input
        msg.effort = effort_input

        jsp.source_cb(msg)

    return None

if __name__ == "__main__":
    terminal_joint_val_publisher()