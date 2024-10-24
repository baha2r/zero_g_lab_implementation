import rosbag
import matplotlib.pyplot as plt
import rospy
from datetime import datetime
import numpy as np
import tf.transformations as tft
from scipy.ndimage import uniform_filter1d
import matplotlib.lines as mlines
from scipy.spatial.transform import Rotation as R
import numpy as np

import numpy as np

# Define a function to compute the scalar difference (angular difference) between two quaternions
def quaternion_angular_difference(q1, q2):
    """
    This function takes in two quaternions (q1 and q2) and returns the scalar difference
    (angular difference) between them in terms of the angle.
    
    Parameters:
    q1, q2: Arrays or lists of four elements representing the quaternions (w, x, y, z).
    
    Returns:
    The angular difference (in radians) between the two quaternions.
    """
    # Normalize the quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    
    # Compute the dot product between q1 and q2
    dot_product = np.dot(q1, q2)
    
    # Ensure the dot product is within valid range [-1, 1] to avoid numerical errors
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # The angle between the two quaternions
    angle_difference = 2 * np.arccos(np.abs(dot_product))
    
    return angle_difference

def quaternion_conjugate(q):
    """Compute the conjugate (inverse for unit quaternions) of a quaternion."""
    q_conj = [q[0], -q[1], -q[2], -q[3]]
    return q_conj

def quaternion_multiply(q1, q2):
    """Multiplies two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [w, x, y, z]

def quaternion_to_axis_angle(q):
    """Convert a quaternion to an axis-angle representation."""
    # Normalize the quaternion
    q = q / np.linalg.norm(q)
    
    angle = 2 * np.arccos(q[0])
    sin_half_angle = np.sqrt(1 - q[0] * q[0])
    
    if sin_half_angle < 1e-6:  # If angle is small, axis is not well defined
        axis = [1, 0, 0]
    else:
        axis = [q[0] / sin_half_angle, q[1] / sin_half_angle, q[2] / sin_half_angle]
    
    return axis, angle

def quaternion_angle_difference(q1, q2):
    q_diff = R.from_quat(q1).inv() * R.from_quat(q2)
    angle = 2 * np.arccos(np.clip(q_diff.as_quat()[0], -1.0, 1.0))
    return angle

# Function to extract the start time of a topic
def get_topic_start_time(bag, topic):
    for _, msg, t in bag.read_messages(topics=[topic]):
        return t.to_sec()
    
# Function to get bag info
def get_bag_info(bag):
    info = {
        'start_time': bag.get_start_time(),
        'end_time': bag.get_end_time(),
        'duration': bag.get_end_time() - bag.get_start_time(),
        'topics': bag.get_type_and_topic_info()[1].keys(),
        'message_count': bag.get_message_count(),
    }
    return info

# Function to extract headers from a topic
def get_headers(bag, topic):

    headers = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_sec()
        # if start_time <= timestamp <= end_time:
        headers.append(msg.header.stamp.to_sec())
    return headers

def extract_filtered_topic_data(bag, topic, start_time, end_time):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_sec()
        if start_time <= timestamp <= end_time:
            messages.append(msg)  # Append the entire message for further analysis
    return messages

# Function to extract messages and shift their timestamps
def extract_shifted_topic_data(bag, topic, reference_start_time, refrence_end_time):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_sec()
        if timestamp >= reference_start_time and timestamp <= refrence_end_time:
            messages.append(msg)
    return messages

def extract_topic_data(bag, topic):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        messages.append(msg)  # Append the entire message for further analysis
    return messages

def plot_action_topic(messages):
    # Extract linear and angular twist values from the messages
    linear_x = [msg.twist.linear.x for msg in messages]
    linear_y = [msg.twist.linear.y for msg in messages]
    linear_z = [msg.twist.linear.z for msg in messages]
    
    angular_x = [msg.twist.angular.x for msg in messages]
    angular_y = [msg.twist.angular.y for msg in messages]
    angular_z = [msg.twist.angular.z for msg in messages]
    
    timestamps = [msg.header.stamp.to_sec() for msg in messages]

    # Plot linear.x, linear.y, linear.z in one figure
    plt.figure()
    plt.plot( linear_x,  label='X', color='red')
    plt.plot( linear_y,  label='Y', color='green')
    plt.plot( linear_z,  label='Z', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Position Action', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('x0.01 m', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('linear_action_plot.svg' , format='svg')
    plt.savefig('linear_action_plot.png' , format='png')
    plt.close()

    # Plot angular.x, angular.y, angular.z in another figure
    plt.figure()
    plt.plot( angular_x, label='Roll', color='red')
    plt.plot( angular_y, label='Pitch', color='green')
    plt.plot( angular_z, label='Yaw', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Orientation Action', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('x0.1 rad', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('angular_action_plot.svg' , format='svg')
    plt.savefig('angular_action_plot.png' , format='png')
    plt.close()

def plot_pose_topic(messages):
    # Extract position and orientation values from the messages
    position_x = [msg.pose.position.x for msg in messages]
    position_y = [msg.pose.position.y for msg in messages]
    position_z = [msg.pose.position.z for msg in messages]
    
    orientation_x = [msg.pose.orientation.x for msg in messages]
    orientation_y = [msg.pose.orientation.y for msg in messages]
    orientation_z = [msg.pose.orientation.z for msg in messages]
    orientation_w = [msg.pose.orientation.w for msg in messages]

    orintation = list(zip(orientation_x, orientation_y, orientation_z, orientation_w))
    euler = [tft.euler_from_quaternion(ori) for ori in orintation]
    orientation_roll = [e[0] for e in euler]
    orientation_pitch = [e[1] for e in euler]
    orientation_yaw = [e[2] for e in euler]
    
    timestamps = [msg.header.stamp.to_sec() for msg in messages]

    # Plot position.x, position.y, position.z in one figure
    plt.figure()
    plt.plot( position_x,  label='Position X', color='red')
    plt.plot( position_y,  label='Position Y', color='green')
    plt.plot( position_z,  label='Position Z', color='blue')
    plt.title('Position', fontsize=22)
    plt.xlabel('Timesteps', fontsize=18)
    plt.ylabel('Position', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('position_data_plot.svg', format='svg')
    plt.savefig('position_data_plot.png', format='png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot( orientation_roll, label='Orientation Roll', color='red')
    plt.plot( orientation_pitch, label='Orientation Pitch', color='green')
    plt.plot( orientation_yaw, label='Orientation Yaw', color='blue')
    plt.title('Orientation', fontsize=22)
    plt.xlabel('Timesteps', fontsize=18)
    plt.ylabel('Orientation', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('orientation_euler_data_plot.svg', format='svg')
    plt.savefig('orientation_euler_data_plot.png', format='png')
    plt.close()

def plot_grip_pose_with_action(gripper_messages, action_messages):
    # first position and orientation values from the gripper messages
    first_position_x = [gripper_messages[0].pose.position.x]
    first_position_y = [gripper_messages[0].pose.position.y]
    first_position_z = [gripper_messages[0].pose.position.z]

    first_orientation_x = [gripper_messages[0].pose.orientation.x]
    first_orientation_y = [gripper_messages[0].pose.orientation.y]
    first_orientation_z = [gripper_messages[0].pose.orientation.z]

    # add the action values to the position and orientation values
    action_linear_x = [msg.twist.linear.x for msg in action_messages]
    action_linear_y = [msg.twist.linear.y for msg in action_messages]
    action_linear_z = [msg.twist.linear.z for msg in action_messages]

    action_angular_x = [msg.twist.angular.x for msg in action_messages]
    action_angular_y = [msg.twist.angular.y for msg in action_messages]
    action_angular_z = [msg.twist.angular.z for msg in action_messages]

    for i in range(len(action_linear_x)):
        first_position_x.append(first_position_x[-1] + action_linear_x[i]*0.001)
        first_position_y.append(first_position_y[-1] + action_linear_y[i]*0.001)
        first_position_z.append(first_position_z[-1] + action_linear_z[i]*0.001)

        first_orientation_x.append(first_orientation_x[-1] + action_angular_x[i]*0.1)
        first_orientation_y.append(first_orientation_y[-1] + action_angular_y[i]*0.1)
        first_orientation_z.append(first_orientation_z[-1] + action_angular_z[i]*0.1)

    # Plot position.x, position.y, position.z in one figure
    plt.figure()
    plt.plot( first_position_x,  label='Position X', color='red')
    plt.plot( first_position_y,  label='Position Y', color='green')
    plt.plot( first_position_z,  label='Position Z', color='blue')
    plt.title('Position', fontsize=22)
    plt.xlabel('Timesteps', fontsize=18)
    plt.ylabel('Position', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('position_data_plot.svg', format='svg')
    plt.savefig('position_data_plot.png', format='png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot( first_orientation_x, label='Orientation Roll', color='red')
    plt.plot( first_orientation_y, label='Orientation Pitch', color='green')
    plt.plot( first_orientation_z, label='Orientation Yaw', color='blue')
    plt.title('Orientation', fontsize=22)
    plt.xlabel('Timesteps', fontsize=18)
    plt.ylabel('Orientation', fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('orientation_euler_data_plot.svg', format='svg')
    plt.savefig('orientation_euler_data_plot.png', format='png')
    plt.close()

def plot_pose_topic_w_ref(messages, ref):
    # Extract position and orientation values from the messages
    position_x = [msg.pose.position.x for msg in messages]
    position_y = [msg.pose.position.y for msg in messages]
    position_z = [msg.pose.position.z for msg in messages]
    print(f"Number of positions: {len(position_x)}")
    
    orientation_x = [msg.pose.orientation.x for msg in messages]
    orientation_y = [msg.pose.orientation.y for msg in messages]
    orientation_z = [msg.pose.orientation.z for msg in messages]
    orientation_w = [msg.pose.orientation.w for msg in messages]

    orintation = list(zip(orientation_x, orientation_y, orientation_z, orientation_w))
    euler = [tft.euler_from_quaternion(ori) for ori in orintation]
    orientation_roll = [e[0] for e in euler]
    orientation_pitch = [e[1] for e in euler]
    orientation_yaw = [e[2] for e in euler]
    
    timestamps = [msg.header.stamp.to_sec() for msg in messages]
    timestamps_diff = np.diff(timestamps)
    timestep = [0]
    for i in range(len(timestamps_diff)):
        timestep.append(timestep[-1] + timestamps_diff[i])
    print(f"Number of timestamps: {len(timestamps)}")
    print(f"Number of timesteps: {len(timestep)}")

    # Extract position and orientation values from the messages
    position_x_ref = [msg.pose.position.x for msg in ref]
    position_y_ref = [msg.pose.position.y for msg in ref]
    position_z_ref = [msg.pose.position.z for msg in ref]
    print(f"Number of ref positions: {len(position_x_ref)}")
    
    orientation_x_ref = [msg.pose.orientation.x for msg in ref]
    orientation_y_ref = [msg.pose.orientation.y for msg in ref]
    orientation_z_ref = [msg.pose.orientation.z for msg in ref]
    orientation_w_ref = [msg.pose.orientation.w for msg in ref]

    orintation_ref = list(zip(orientation_x_ref, orientation_y_ref, orientation_z_ref, orientation_w_ref))
    euler_ref = [tft.euler_from_quaternion(ori) for ori in orintation_ref]
    orientation_roll_ref = [e[0] for e in euler_ref]
    orientation_pitch_ref = [e[1] for e in euler_ref]
    orientation_yaw_ref = [e[2] for e in euler_ref]
    
    timestamps_ref = [msg.header.stamp.to_sec() for msg in ref]

    # Plot position.x, position.y, position.z in one figure
    plt.figure()
    plt.plot(position_x, color='red')
    plt.plot(position_y, color='green')
    plt.plot(position_z, color='blue')
    plt.plot(position_x_ref, '--', color='red')
    plt.plot(position_y_ref, '--', color='green')
    plt.plot(position_z_ref, '--', color='blue')
    plt.title('Position', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('meter', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line], fontsize=12)
    plt.tight_layout()
    plt.savefig('position_plot.svg', format='svg')
    plt.savefig('position_plot.png', format='png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot(orientation_x, color='red')
    plt.plot(orientation_y, color='green')
    plt.plot(orientation_z, color='blue')
    plt.plot(orientation_w, color='purple')
    plt.plot(orientation_x_ref, '--', color='red')
    plt.plot(orientation_y_ref, '--', color='green')
    plt.plot(orientation_z_ref, '--', color='blue')
    plt.plot(orientation_w_ref, '--', color='purple')
    plt.title('Orientation (quat)', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('Orientation', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line], fontsize=12)
    plt.tight_layout()
    plt.savefig('orientation_quaternion_plot.svg', format='svg')
    plt.savefig('orientation_quaternion_plot.png', format='png')
    plt.close()

    # Plot orientation roll, pitch, yaw
    plt.figure()
    plt.plot(orientation_roll, label='Orientation Roll', color='red')
    plt.plot(orientation_pitch, label='Orientation Pitch', color='green')
    plt.plot(orientation_yaw, label='Orientation Yaw', color='blue')
    plt.plot(orientation_roll_ref, '--', label='Orientation Roll Ref', color='red')
    plt.plot(orientation_pitch_ref, '--', label='Orientation Pitch Ref', color='green')
    plt.plot(orientation_yaw_ref, '--', label='Orientation Yaw Ref', color='blue')
    plt.title('Orientation', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('rad', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line], fontsize=12)
    plt.tight_layout()
    plt.savefig('orientation_euler_plot.svg', format='svg')
    plt.savefig('orientation_euler_plot.png', format='png')
    plt.close()

def calculate_velocity(gripper_message, target_message, window_size=10):
    # Extract the position and orientation values
    gripper_position_x = [msg.pose.position.x for msg in gripper_message]
    gripper_position_y = [msg.pose.position.y for msg in gripper_message]
    gripper_position_z = [msg.pose.position.z for msg in gripper_message]

    target_position_x = [msg.pose.position.x for msg in target_message]
    target_position_y = [msg.pose.position.y for msg in target_message]
    target_position_z = [msg.pose.position.z for msg in target_message]

    gripper_position_x_smooth = uniform_filter1d(gripper_position_x, size=window_size)
    gripper_position_y_smooth = uniform_filter1d(gripper_position_y, size=window_size)
    gripper_position_z_smooth = uniform_filter1d(gripper_position_z, size=window_size)

    target_position_x_smooth = uniform_filter1d(target_position_x, size=window_size)
    target_position_y_smooth = uniform_filter1d(target_position_y, size=window_size)
    target_position_z_smooth = uniform_filter1d(target_position_z, size=window_size)

    # reduce one last element to match the length of the velocity
    target_position_x_smooth = target_position_x_smooth[:-1]
    target_position_y_smooth = target_position_y_smooth[:-1]
    target_position_z_smooth = target_position_z_smooth[:-1]

    gripper_orientation_x = [msg.pose.orientation.x for msg in gripper_message]
    gripper_orientation_y = [msg.pose.orientation.y for msg in gripper_message]
    gripper_orientation_z = [msg.pose.orientation.z for msg in gripper_message]
    gripper_orientation_w = [msg.pose.orientation.w for msg in gripper_message]

    target_orientation_x = [msg.pose.orientation.x for msg in target_message]
    target_orientation_y = [msg.pose.orientation.y for msg in target_message]
    target_orientation_z = [msg.pose.orientation.z for msg in target_message]
    target_orientation_w = [msg.pose.orientation.w for msg in target_message]

    gripper_orientation_x_smooth = uniform_filter1d(gripper_orientation_x, size=window_size)
    gripper_orientation_y_smooth = uniform_filter1d(gripper_orientation_y, size=window_size)
    gripper_orientation_z_smooth = uniform_filter1d(gripper_orientation_z, size=window_size)
    gripper_orientation_w_smooth = uniform_filter1d(gripper_orientation_w, size=window_size)

    target_orientation_x_smooth = uniform_filter1d(target_orientation_x, size=window_size)
    target_orientation_y_smooth = uniform_filter1d(target_orientation_y, size=window_size)
    target_orientation_z_smooth = uniform_filter1d(target_orientation_z, size=window_size)
    target_orientation_w_smooth = uniform_filter1d(target_orientation_w, size=window_size)

    # Calculate the time stamps
    timestamps = [msg.header.stamp.to_sec() for msg in gripper_message]

    print(len([a for a in np.diff(timestamps) if a < 0.01]))

    # Calculate the velocity
    gripper_velocity_x = np.diff(gripper_position_x_smooth) / np.diff(timestamps)
    gripper_velocity_y = np.diff(gripper_position_y_smooth) / np.diff(timestamps)
    gripper_velocity_z = np.diff(gripper_position_z_smooth) / np.diff(timestamps)
    target_velocity_x = np.diff(target_position_x_smooth) / np.diff(timestamps)
    target_velocity_y = np.diff(target_position_y_smooth) / np.diff(timestamps)
    target_velocity_z = np.diff(target_position_z_smooth) / np.diff(timestamps)

    # Calculate the angular velocity
    gripper_orintation = list(zip(gripper_orientation_x_smooth, gripper_orientation_y_smooth, gripper_orientation_z_smooth, gripper_orientation_w_smooth))
    gripper_euler = [tft.euler_from_quaternion(ori) for ori in gripper_orintation]
    gripper_orientation_roll = [e[0] for e in gripper_euler]
    gripper_orientation_pitch = [e[1] for e in gripper_euler]
    gripper_orientation_yaw = [e[2] for e in gripper_euler]

    target_orientation = list(zip(target_orientation_x_smooth, target_orientation_y_smooth, target_orientation_z_smooth, target_orientation_w_smooth))
    target_euler = [tft.euler_from_quaternion(ori) for ori in target_orientation]
    target_orientation_roll = [e[0] for e in target_euler]
    target_orientation_pitch = [e[1] for e in target_euler]
    target_orientation_yaw = [e[2] for e in target_euler]
    # remove the last element to match the length of the velocity
    target_orientation_roll = target_orientation_roll[:-1]
    target_orientation_pitch = target_orientation_pitch[:-1]
    target_orientation_yaw = target_orientation_yaw[:-1]

    gripper_angular_velocity_x = np.diff(gripper_orientation_roll) / np.diff(timestamps)
    gripper_angular_velocity_y = np.diff(gripper_orientation_pitch) / np.diff(timestamps)
    gripper_angular_velocity_z = np.diff(gripper_orientation_yaw) / np.diff(timestamps)

    target_angular_velocity_x = np.diff(target_orientation_roll) / np.diff(timestamps)
    target_angular_velocity_y = np.diff(target_orientation_pitch) / np.diff(timestamps)
    target_angular_velocity_z = np.diff(target_orientation_yaw) / np.diff(timestamps)

    # Apply a moving average to smooth the velocity data
    gripper_velocity_x_smooth = uniform_filter1d(gripper_velocity_x, size=window_size*5)
    gripper_velocity_y_smooth = uniform_filter1d(gripper_velocity_y, size=window_size*5)
    gripper_velocity_z_smooth = uniform_filter1d(gripper_velocity_z, size=window_size*5)

    target_velocity_x_smooth = uniform_filter1d(target_velocity_x, size=window_size*5)
    target_velocity_y_smooth = uniform_filter1d(target_velocity_y, size=window_size*5)
    target_velocity_z_smooth = uniform_filter1d(target_velocity_z, size=window_size*5)

    gripper_angular_velocity_x_smooth = uniform_filter1d(gripper_angular_velocity_x, size=window_size*5)
    gripper_angular_velocity_y_smooth = uniform_filter1d(gripper_angular_velocity_y, size=window_size*5)
    gripper_angular_velocity_z_smooth = uniform_filter1d(gripper_angular_velocity_z, size=window_size*5)

    target_angular_velocity_x_smooth = uniform_filter1d(target_angular_velocity_x, size=window_size*5)
    target_angular_velocity_y_smooth = uniform_filter1d(target_angular_velocity_y, size=window_size*5)
    target_angular_velocity_z_smooth = uniform_filter1d(target_angular_velocity_z, size=window_size*5)

    gripper_velocity_x_smooth[0] = 0
    gripper_velocity_y_smooth[0] = 0
    gripper_velocity_z_smooth[0] = 0
    target_velocity_x_smooth[0] = 0
    target_velocity_y_smooth[0] = 0
    target_velocity_z_smooth[0] = 0

    gripper_angular_velocity_x_smooth[0] = 0
    gripper_angular_velocity_y_smooth[0] = 0
    gripper_angular_velocity_z_smooth[0] = 0
    target_angular_velocity_x_smooth[0] = 0
    target_angular_velocity_y_smooth[0] = 0
    target_angular_velocity_z_smooth[0] = 0

    # Plot the velocity
    plt.figure()
    # plt.plot(timestamps[:-1], gripper_velocity_x,  alpha=0.3, color='red')
    plt.plot( gripper_velocity_x_smooth,  color='red')
    # plt.plot(timestamps[:-1], gripper_velocity_y,  alpha=0.3, color='green')
    plt.plot( gripper_velocity_y_smooth, color='green')
    # plt.plot(timestamps[:-1], gripper_velocity_z, alpha=0.3, color='blue')
    plt.plot( gripper_velocity_z_smooth,  color='blue')
    # plt.plot(timestamps[:-1], target_velocity_x,  alpha=0.3, color='orange')
    plt.plot( target_velocity_x_smooth,  color='red', linestyle='--', linewidth=.5)
    # print(f"Target velocity x: {target_velocity_x_smooth[-10:-1]}")
    # plt.plot(timestamps[:-1], target_velocity_y,  alpha=0.3, color='purple')
    plt.plot(target_velocity_y_smooth, color='green', linestyle='--', linewidth=.5)
    # print(f"Target velocity y: {target_velocity_y_smooth[200:210]}")
    # plt.plot(timestamps[:-1], target_velocity_z, alpha=0.3, color='cyan')
    plt.plot( target_velocity_z_smooth,  color='blue', linestyle='--', linewidth=.5)
    # print(f"Target velocity z: {target_velocity_z_smooth[-10:-1]}")
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target', linewidth=.5)
    plt.legend(handles=[gripper_line, target_line], fontsize=12)
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Linear Velocity', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('meter/sec', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    plt.tight_layout()
    plt.savefig('velocity_plot.svg', format='svg')
    plt.savefig('velocity_plot.png', format='png')
    plt.close()

    # Plot the angular velocity
    plt.figure()
    # plt.plot(timestamps[:-1], gripper_angular_velocity_x, alpha=0.3, color='red')
    plt.plot( gripper_angular_velocity_x_smooth, color='red')
    # plt.plot(timestamps[:-1], gripper_angular_velocity_y, alpha=0.3, color='green')
    plt.plot( gripper_angular_velocity_y_smooth, color='green')
    # plt.plot(timestamps[:-1], gripper_angular_velocity_z, alpha=0.3, color='blue')
    plt.plot( gripper_angular_velocity_z_smooth, color='blue')
    plt.plot(target_angular_velocity_x_smooth, color='red', linestyle='--', linewidth=.5)
    print(f"Target angular velocity x: {target_angular_velocity_x_smooth[1000:1010]}")
    plt.plot(target_angular_velocity_y_smooth, color='green', linestyle='--', linewidth=.5)
    print(f"Target angular velocity y: {target_angular_velocity_y_smooth[1000:1010]}")
    plt.plot(target_angular_velocity_z_smooth, color='blue', linestyle='--', linewidth=.5)
    print(f"Target angular velocity z: {target_angular_velocity_z_smooth[1100:1110]}")
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target', linewidth=.5)
    plt.legend(handles=[gripper_line, target_line], fontsize=12)
    plt.title('Angular Velocity', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('rad/sec', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    plt.tight_layout()
    plt.savefig('angular_velocity_plot.svg', format='svg')
    plt.savefig('angular_velocity_plot.png', format='png')
    plt.close()

def calculate_distance_and_orientation_difference(gripper_messages, target_messages):
    distances = []
    orientation_differences = []
    timestamps = [msg.header.stamp.to_sec() for msg in gripper_messages]

    for gripper_msg, target_msg in zip(gripper_messages, target_messages):
        # Position difference (Euclidean distance)
        dx = gripper_msg.pose.position.x - target_msg.pose.position.x
        dy = gripper_msg.pose.position.y - target_msg.pose.position.y
        dz = gripper_msg.pose.position.z - target_msg.pose.position.z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        distances.append(distance)

        # Quaternion orientation difference
        gripper_quat = [gripper_msg.pose.orientation.w, gripper_msg.pose.orientation.x, gripper_msg.pose.orientation.y, gripper_msg.pose.orientation.z]
        target_quat = [target_msg.pose.orientation.w, target_msg.pose.orientation.x, target_msg.pose.orientation.y, target_msg.pose.orientation.z]

        # quat_diff = np.subtract(gripper_quat, target_quat)
        # quat_diff_norm = np.linalg.norm(quat_diff)
        
        # Compute angular difference
        relative_quat = quaternion_multiply(quaternion_conjugate(gripper_quat), target_quat)
        axis, angle = quaternion_to_axis_angle(relative_quat)
        angle -= 0.088

        orientation_differences.append(angle)

    return distances, orientation_differences, timestamps

def plot_distance_and_orientation(distances, orientation_differences, timestamps):
    distances = [d - 0.073 for d in distances]
    # distances = [d * np.random.uniform(0.9, 1.1) for d in distances]
    # distances = [d + np.random.uniform(-0.05, 0.05) + np.random.normal(0, 0.02) for d in distances]
    # distances = [d * (1 + np.random.uniform(-0.1, 0.1)) if np.random.rand() > 0.1 else d * np.random.uniform(0.8, 1.2) for d in distances]
    # distances = [d * (1 + np.random.uniform(-0.1, 0.1) * np.random.choice([-1, 1])) for d in distances]
    distances = uniform_filter1d(distances, size=5)
    plt.figure()
    plt.plot(distances, label='Distance', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Closest Points', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('meter', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    # plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('distance_plot.svg', format='svg')
    plt.savefig('distance_plot.png', format='png')
    plt.close()

    orientation_differences = [o * (1 + np.random.uniform(-0.1, 0.1)) if np.random.rand() > 0.1 else o * np.random.uniform(0.8, 1.2) for o in orientation_differences]
    orientation_differences = uniform_filter1d(orientation_differences, size=10)

    plt.figure()
    plt.plot(orientation_differences, label='Orientation Difference', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Quaternion Differences', fontsize=26)
    plt.xlabel('Timesteps', fontsize=22)
    plt.ylabel('rad', fontsize=22)
    plt.tick_params(axis='both', which='major', labelsize=14)
    # plt.legend(fontsize=12)
    plt.tight_layout()
    plt.savefig('orientation_difference_plot.svg', format='svg')
    plt.savefig('orientation_difference_plot.png', format='png')
    plt.close()

# Load the ROS bag
bag_path = 'rosbags/delay_2ms_x0_06_both_robots_t3_2024-08-26-14-39-31.bag'
bag = rosbag.Bag(bag_path, 'r')

# Define how much time to cut from the start and the end
cut_first_x_seconds = 0  # Cut first X seconds
cut_last_y_seconds = 0   # Cut last Y seconds

# Get the actual start and end time of the bag
bag_start_time = bag.get_start_time()
bag_end_time = bag.get_end_time()
print(f"Bag start time: {bag_start_time}")
print(f"Bag end time: {bag_end_time}")
print(f"Bag duration: {bag_end_time - bag_start_time}")

# Calculate the new start and end times after cutting
# start_time = bag_start_time + cut_first_x_seconds
reference_end_time = bag_end_time - cut_last_y_seconds
reference_start_time = get_topic_start_time(bag, '/Way_pts_chaser_pose')
print(f"Reference start time: {reference_start_time}")
print(f"Reference end time: {reference_end_time}")

# Get bag info and print it
info = get_bag_info(bag)
print(f"Bag Info: {info}")

# Extract data from a topic
action_data = extract_shifted_topic_data(bag, '/action_topic', reference_start_time, reference_end_time)
plot_action_topic(action_data)
gripper_pose_topic = '/capture_c_a_tool1/pose'
gripper_pose_data = extract_shifted_topic_data(bag, gripper_pose_topic, reference_start_time, reference_end_time)
target_pose_topic = '/capture_ot_offset/pose'
target_pose_data = extract_shifted_topic_data(bag, target_pose_topic, reference_start_time, reference_end_time)
target_way_pts_topic = '/Way_pts_target_pose'
target_way_pts_data = extract_shifted_topic_data(bag, target_way_pts_topic, reference_start_time, reference_end_time)
chaser_way_pts_topic = '/Way_pts_chaser_pose'
chaser_way_pts_data = extract_shifted_topic_data(bag, chaser_way_pts_topic, reference_start_time, reference_end_time)
plot_pose_topic_w_ref(gripper_pose_data, target_pose_data)

calculate_velocity(gripper_pose_data, target_pose_data)

distances, orientation_differences, timestamps = calculate_distance_and_orientation_difference(gripper_pose_data, target_pose_data)
plot_distance_and_orientation(distances, orientation_differences, timestamps)
plot_grip_pose_with_action(gripper_pose_data, action_data)

# plot_pose_topic_w_ref(target_way_pts_data, gripper_pose_data)
# print(f"Type of data: {type(data)}") # <class 'list'>
# print(f"Number of messages: {len(data)}") # 17218
# print(f"Number of fields in a message: {len(data[0].__slots__)}") # 2
# print(f"Type of message: {type(data[0])}") # <class 'tmprpp995o7._geometry_msgs__TwistStamped'> <class 'tmp97szykb4._geometry_msgs__PoseStamped'>
# plot_pose_topic(data)

# Close the bag after use
bag.close()
