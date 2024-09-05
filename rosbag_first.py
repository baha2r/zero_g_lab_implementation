import rosbag
import matplotlib.pyplot as plt
import rospy
from datetime import datetime
import numpy as np
import tf.transformations as tft
from scipy.ndimage import uniform_filter1d

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

# Function to plot the headers' timestamps
def plot_headers(headers):
    plt.figure()
    plt.plot(headers, 'o-')
    plt.title('Timestamps from Message Headers')
    plt.xlabel('Message Index')
    plt.ylabel('Time (s)')
    # save the plot
    plt.savefig('timestamps.png')
    # plt.show()

def extract_filtered_topic_data(bag, topic, start_time, end_time):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_sec()
        if start_time <= timestamp <= end_time:
            messages.append(msg)  # Append the entire message for further analysis
    return messages

def extract_topic_data(bag, topic):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        messages.append(msg)  # Append the entire message for further analysis
    return messages

# Function to print details of the extracted messages
def print_action_topic_messages(messages):
    for i, msg in enumerate(messages):
        # Print some details for each message
        # Assuming msg has a 'header' and other fields you are interested in
        print(f"Message {i}:")
        print(f"  Timestamp: {msg.header.stamp.to_sec()}")
        print(f"  Data: {msg.twist.linear.x}, {msg.twist.angular.z}")

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
    plt.plot(timestamps, linear_x,  label='Linear X')
    plt.plot(timestamps, linear_y,  label='Linear Y')
    plt.plot(timestamps, linear_z,  label='Linear Z')
    plt.title('Linear Twist in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('linear_twist_plot.png')
    plt.close()

    # Plot angular.x, angular.y, angular.z in another figure
    plt.figure()
    plt.plot(timestamps, angular_x, label='Angular X')
    plt.plot(timestamps, angular_y, label='Angular Y')
    plt.plot(timestamps, angular_z, label='Angular Z')
    plt.title('Angular Twist in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('angular_twist_plot.png')
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
    plt.plot(timestamps, position_x,  label='Position X')
    plt.plot(timestamps, position_y,  label='Position Y')
    plt.plot(timestamps, position_z,  label='Position Z')
    plt.title('Position in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.tight_layout()
    plt.savefig('position_data_plot.png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot(timestamps, orientation_roll, label='Orientation Roll')
    plt.plot(timestamps, orientation_pitch, label='Orientation Pitch')
    plt.plot(timestamps, orientation_yaw, label='Orientation Yaw')
    plt.title('Orientation in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation')
    plt.legend()
    plt.tight_layout()
    plt.savefig('orientation_euler_data_plot.png')
    plt.close()

def plot_pose_topic_w_ref(messages, ref):
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

    # Extract position and orientation values from the messages
    position_x_ref = [msg.pose.position.x for msg in ref]
    position_y_ref = [msg.pose.position.y for msg in ref]
    position_z_ref = [msg.pose.position.z for msg in ref]
    
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
    plt.plot(timestamps, position_x,  label='Position X')
    plt.plot(timestamps, position_y,  label='Position Y')
    plt.plot(timestamps, position_z,  label='Position Z')
    plt.plot(timestamps_ref, position_x_ref, '--', label='Position X Ref')
    plt.plot(timestamps_ref, position_y_ref, '--', label='Position Y Ref')
    plt.plot(timestamps_ref, position_z_ref, '--', label='Position Z Ref')
    plt.title('Position in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.tight_layout()
    plt.savefig('position_plot.png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot(timestamps, orientation_roll, label='Orientation Roll')
    plt.plot(timestamps, orientation_pitch, label='Orientation Pitch')
    plt.plot(timestamps, orientation_yaw, label='Orientation Yaw')
    plt.plot(timestamps_ref, orientation_roll_ref, '--', label='Orientation Roll Ref')
    plt.plot(timestamps_ref, orientation_pitch_ref, '--', label='Orientation Pitch Ref')
    plt.plot(timestamps_ref, orientation_yaw_ref, '--', label='Orientation Yaw Ref')
    plt.title('Orientation in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation')
    plt.legend()
    plt.tight_layout()
    plt.savefig('orientation_euler_plot.png')
    plt.close()

import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tft
from scipy.ndimage import uniform_filter1d

def calculate_velocity(message, window_size=10):
    # Extract the position and orientation values
    position_x = [msg.pose.position.x for msg in message]
    position_y = [msg.pose.position.y for msg in message]
    position_z = [msg.pose.position.z for msg in message]

    orientation_x = [msg.pose.orientation.x for msg in message]
    orientation_y = [msg.pose.orientation.y for msg in message]
    orientation_z = [msg.pose.orientation.z for msg in message]
    orientation_w = [msg.pose.orientation.w for msg in message]

    # Calculate the time stamps
    timestamps = [msg.header.stamp.to_sec() for msg in message]

    # Calculate the velocity
    velocity_x = np.diff(position_x) / np.diff(timestamps)
    velocity_y = np.diff(position_y) / np.diff(timestamps)
    velocity_z = np.diff(position_z) / np.diff(timestamps)

    # Calculate the angular velocity
    orintation = list(zip(orientation_x, orientation_y, orientation_z, orientation_w))
    euler = [tft.euler_from_quaternion(ori) for ori in orintation]
    orientation_roll = [e[0] for e in euler]
    orientation_pitch = [e[1] for e in euler]
    orientation_yaw = [e[2] for e in euler]

    angular_velocity_x = np.diff(orientation_roll) / np.diff(timestamps)
    angular_velocity_y = np.diff(orientation_pitch) / np.diff(timestamps)
    angular_velocity_z = np.diff(orientation_yaw) / np.diff(timestamps)

    # Apply a moving average to smooth the velocity data
    velocity_x_smooth = uniform_filter1d(velocity_x, size=window_size)
    velocity_y_smooth = uniform_filter1d(velocity_y, size=window_size)
    velocity_z_smooth = uniform_filter1d(velocity_z, size=window_size)

    angular_velocity_x_smooth = uniform_filter1d(angular_velocity_x, size=window_size)
    angular_velocity_y_smooth = uniform_filter1d(angular_velocity_y, size=window_size)
    angular_velocity_z_smooth = uniform_filter1d(angular_velocity_z, size=window_size)

    # Plot the velocity
    plt.figure()
    plt.plot(timestamps[:-1], velocity_x, label='Velocity X (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], velocity_x_smooth, label='Velocity X (smoothed)')
    plt.plot(timestamps[:-1], velocity_y, label='Velocity Y (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], velocity_y_smooth, label='Velocity Y (smoothed)')
    plt.plot(timestamps[:-1], velocity_z, label='Velocity Z (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], velocity_z_smooth, label='Velocity Z (smoothed)')
    plt.title('Velocity in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('velocity_plot.png')
    plt.close()

    # Plot the angular velocity
    plt.figure()
    plt.plot(timestamps[:-1], angular_velocity_x, label='Angular Velocity X (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], angular_velocity_x_smooth, label='Angular Velocity X (smoothed)')
    plt.plot(timestamps[:-1], angular_velocity_y, label='Angular Velocity Y (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], angular_velocity_y_smooth, label='Angular Velocity Y (smoothed)')
    plt.plot(timestamps[:-1], angular_velocity_z, label='Angular Velocity Z (raw)', alpha=0.3)
    plt.plot(timestamps[:-1], angular_velocity_z_smooth, label='Angular Velocity Z (smoothed)')
    plt.title('Angular Velocity in X, Y, Z Directions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('angular_velocity_plot.png')
    plt.close()

    return velocity_x_smooth, velocity_y_smooth, velocity_z_smooth, angular_velocity_x_smooth, angular_velocity_y_smooth, angular_velocity_z_smooth

# Load the ROS bag
bag_path = 'rosbags/delay_2ms_x0.06_both_robots_all50Hz_ry_3.5_t_16_2024-08-26-17-32-08.bag'
bag = rosbag.Bag(bag_path, 'r')

# Define how much time to cut from the start and the end
cut_first_x_seconds = 10  # Cut first X seconds
cut_last_y_seconds = 30   # Cut last Y seconds

# Get the actual start and end time of the bag
bag_start_time = bag.get_start_time()
bag_end_time = bag.get_end_time()
print(f"Bag start time: {bag_start_time}")
print(f"Bag end time: {bag_end_time}")

# Calculate the new start and end times after cutting
start_time = bag_start_time + cut_first_x_seconds
end_time = bag_end_time - cut_last_y_seconds

# Get bag info and print it
info = get_bag_info(bag)
# print(f"Bag Info: {info}")

# Extract data from a topic
action_data = extract_filtered_topic_data(bag, '/action_topic', start_time, end_time)
plot_action_topic(action_data)
gripper_pose_topic = '/capture_c_a_tool1/pose'
gripper_pose_data = extract_filtered_topic_data(bag, gripper_pose_topic, start_time, end_time)
velocity_x, velocity_y, velocity_z, angular_velocity_x, angular_velocity_y, angular_velocity_z = calculate_velocity(gripper_pose_data)
# print(f"Velocity X: {velocity_x}")
target_pose_topic = '/capture_ot_offset/pose'
target_pose_data = extract_filtered_topic_data(bag, target_pose_topic, start_time, end_time)
plot_pose_topic_w_ref(gripper_pose_data, target_pose_data)

topic = '/target_frame_pose'
data = extract_filtered_topic_data(bag, topic, start_time, end_time)
print(f"Type of data: {type(data)}") # <class 'list'>
print(f"Number of messages: {len(data)}") # 17218
print(f"Number of fields in a message: {len(data[0].__slots__)}") # 2
print(f"Type of message: {type(data[0])}") # <class 'tmprpp995o7._geometry_msgs__TwistStamped'> <class 'tmp97szykb4._geometry_msgs__PoseStamped'>
plot_pose_topic(data)

# Choose a topic to extract headers from
# topic = '/action_topic'
# data = bag.read_messages(topics=[topic])
# print("type of data: ", type(data))

# Get and plot headers
# headers = get_headers(bag, topic)
# print(f"Headers: {headers}")
# plot_headers(headers)

# Close the bag after use
bag.close()
