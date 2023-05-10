#!/usr/bin/env python

import rospy
import threading
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from ambf_client import Client
import time
import matplotlib.pyplot as plt

# Create a instance of the client
_client = Client()

# Connect the client 
_client.connect()


# Get the objects
basey_obj = _client.get_obj_handle('/ambf/env/404xr100_21_1')
tip_obj = _client.get_obj_handle('/ambf/env/IRIS')
# basex_obj = _client.get_obj_handle('/ambf/env/404xr100_21_2')
# zstage_obj = _client.get_obj_handle('/ambf/env/z_stage')
# rstage_obj = _client.get_obj_handle('/ambf/env/rotary_stage')
# pusher_obj = _client.get_obj_handle('/ambf/env/SHER_pusher')
time.sleep(1.0)

# Get first location of tip 
tip_0 = tip_obj.get_pos()

# J_inv
J = np.zeros((6, 5))
J[0,0] = 1
J[1,1] = 1
J[2,2] = 1
J[4,3] = 1
J[3,4] = 1
J_inv = np.linalg.pinv(J)

# Adjoint
F_omni2base = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

p = np.array([1, 2, 3])
R = F_omni2base[:3,:3]

Adj = np.zeros((6,6))
Adj[:3,:3] = R
p_skew = np.array([[0, -p[2], p[1]],
                   [p[2], 0, -p[0]],
                   [-p[1], p[0], 0]])
Adj[3:,:3] = np.dot(p_skew, R)
Adj[3:,3:] = R

# Initialize the lists to store the data for plotting
time_list = []
spacenav_x_list = []
spacenav_y_list = []
spacenav_z_list = []
tip_x_list = []
tip_y_list = []
tip_z_list = []

def robot_FK(q1,q2,q3,q4,q5):
    T_y = np.array([[1, 0, 0, 0],
                    [0, 1, 0, q1],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    
    T_x = np.array([[1, 0, 0, q2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    
    T_z = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, q3],
                    [0, 0, 0, 1]])
    
    R_y = np.array([[np.cos(q4), 0, np.sin(q4), 0],
                    [0, 1, 0, 0],
                    [-np.sin(q4), 0, np.cos(q4), 0],
                    [0, 0, 0, 1]])
    
    R_x = np.array([[1, 0, 0, 0],
                    [0, np.cos(q5), -np.sin(q5), 0],
                    [0, np.sin(q5), np.cos(q5), 0],
                    [0, 0, 0, 1]])
    
    # T = T_y @ T_x @ T_z @ R_y @ R_x
    T = np.dot(np.dot(np.dot(np.dot(T_y, T_x), T_z), R_y), R_x)
    return T

def read_spacenav():
    
    # Initialize the spacenav data and the inverse Jacobian matrix
    spacenav_data = np.zeros(6)
    
    def callback(data):
            spacenav_data[:] = data.axes[:6]
            spacenav_data[:] *= 0.1
    rospy.Subscriber("/spacenav/joy", Joy, callback)  # Subscribe to the spacenav/joy topic

    # Set the desired rate to 100Hz (0.01s interval)
    rate = rospy.Rate(100)

    # Enter the ROS event loop
    while not rospy.is_shutdown():
        # print("Spacenav data: %s" % spacenav_data)  # Print the spacenav data
        # Transfer [v w] of haptic tip to SHER base
        spatial_velocity = np.dot(Adj,spacenav_data)
        # Compute the joint velocities using the spacenav data and the inverse Jacobian
        joint_velocities = np.dot(J_inv, spatial_velocity)
        # print("Joint_v: %s" % joint_velocities)
        spacenav_x_list.append(joint_velocities[1])
        tip_position = tip_obj.get_pos()
        tip_x_list.append(tip_position.x)
        time_list.append(rospy.get_time())
        # joint_positions += joint_velocities * 0.01  # Update joint positions
        basey_obj.set_joint_vel(0,joint_velocities[0]) # q1
        basey_obj.set_joint_vel(1,joint_velocities[1]) # q2
        basey_obj.set_joint_vel(5,joint_velocities[2]) # q3
        basey_obj.set_joint_vel(3,joint_velocities[3]) # q4
        basey_obj.set_joint_vel(2,joint_velocities[4]) # q5

        rate.sleep()



if __name__ == '__main__':
    # Initialize the ROS node
    # rospy.init_node('spacenav_listener')

    # Create a publisher for the robot state
    # robot_state_pub = rospy.Publisher('/ambf/env/robot/command', ObjectState, queue_size=10)

    # Start a separate thread to read spacenav data
    spacenav_thread = threading.Thread(target=read_spacenav)
    spacenav_thread.start()

    # Plot spacenav_x_list and tip_x_list over time_list
    # tip_x_vel = np.diff(tip_x_list) / 0.01
    # Enter the ROS event loop
    rospy.spin()
    # Calculate the derivative of tip_x
    tip_x_dot_list = []
    for i in range(1, len(tip_x_list)):
        delta_x = tip_x_list[i] - tip_x_list[i-1]
        delta_t = time_list[i] - time_list[i-1]
        tip_x_dot = delta_x / delta_t if delta_t != 0 else 0
        tip_x_dot_list.append(tip_x_dot)
    
    tip_x_dot_list.append(0)

    plt.plot(time_list, spacenav_x_list, label='Spacenav X')
    plt.plot(time_list, tip_x_dot_list, label='Tip X')

    # Set the labels and title
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Comparison of Spacenav and Tip X Velocity')

    # Show the legend
    plt.legend()

    # Show the plot
    plt.show()

 
    # Wait
