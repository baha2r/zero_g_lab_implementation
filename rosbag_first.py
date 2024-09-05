import rosbag
import matplotlib.pyplot as plt
import rospy
from datetime import datetime
import numpy as np
import tf.transformations as tft
from scipy.ndimage import uniform_filter1d
import matplotlib.lines as mlines

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

# Function to extract messages and shift their timestamps
def extract_shifted_topic_data(bag, topic, reference_start_time):
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamp = t.to_sec()
        if timestamp >= reference_start_time:
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
    plt.plot(timestamps, linear_x,  label='X', color='red')
    plt.plot(timestamps, linear_y,  label='Y', color='green')
    plt.plot(timestamps, linear_z,  label='Z', color='blue')
    plt.title('Displacement Action')
    plt.xlabel('Time (s)')
    plt.ylabel('x0.01 m')
    plt.legend()
    plt.tight_layout()
    plt.savefig('linear_action_plot.png')
    plt.close()

    # Plot angular.x, angular.y, angular.z in another figure
    plt.figure()
    plt.plot(timestamps, angular_x, label='Roll', color='red')
    plt.plot(timestamps, angular_y, label='Pitch', color='green')
    plt.plot(timestamps, angular_z, label='Yaw', color='blue')
    plt.title('Rotation Action')
    plt.xlabel('Time (s)')
    plt.ylabel('x0.1 rad')
    plt.legend()
    plt.tight_layout()
    plt.savefig('angular_action_plot.png')
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
    plt.plot(timestamps, position_x,  label='Position X', color='red')
    plt.plot(timestamps, position_y,  label='Position Y', color='green')
    plt.plot(timestamps, position_z,  label='Position Z', color='blue')
    plt.title('Position over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.tight_layout()
    plt.savefig('position_data_plot.png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot(timestamps, orientation_roll, label='Orientation Roll', color='red')
    plt.plot(timestamps, orientation_pitch, label='Orientation Pitch', color='green')
    plt.plot(timestamps, orientation_yaw, label='Orientation Yaw', color='blue')
    plt.title('Orientation over Time')
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
    plt.plot(timestamps, position_x, color='red')
    plt.plot(timestamps, position_y, color='green')
    plt.plot(timestamps, position_z, color='blue')
    plt.plot(timestamps_ref, position_x_ref, '--', color='red')
    plt.plot(timestamps_ref, position_y_ref, '--', color='green')
    plt.plot(timestamps_ref, position_z_ref, '--', color='blue')
    plt.title('Position over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line])
    plt.tight_layout()
    plt.savefig('position_plot.png')
    plt.close()

    # Plot orientation.x, orientation.y, orientation.z, orientation.w in another figure
    plt.figure()
    plt.plot(timestamps, orientation_x, color='red')
    plt.plot(timestamps, orientation_y, color='green')
    plt.plot(timestamps, orientation_z, color='blue')
    plt.plot(timestamps, orientation_w, color='purple')
    plt.plot(timestamps_ref, orientation_x_ref, '--', color='red')
    plt.plot(timestamps_ref, orientation_y_ref, '--', color='green')
    plt.plot(timestamps_ref, orientation_z_ref, '--', color='blue')
    plt.plot(timestamps_ref, orientation_w_ref, '--', color='purple')
    plt.title('Orientation (quat) over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation')
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line])
    plt.tight_layout()
    plt.savefig('orientation_quaternion_plot.png')
    plt.close()

    # Plot orientation roll, pitch, yaw
    plt.figure()
    plt.plot(timestamps, orientation_roll, label='Orientation Roll', color='red')
    plt.plot(timestamps, orientation_pitch, label='Orientation Pitch', color='green')
    plt.plot(timestamps, orientation_yaw, label='Orientation Yaw', color='blue')
    plt.plot(timestamps_ref, orientation_roll_ref, '--', label='Orientation Roll Ref', color='red')
    plt.plot(timestamps_ref, orientation_pitch_ref, '--', label='Orientation Pitch Ref', color='green')
    plt.plot(timestamps_ref, orientation_yaw_ref, '--', label='Orientation Yaw Ref', color='blue')
    plt.title('Orientation (euler) over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation')
    gripper_line = mlines.Line2D([], [], color='black', linestyle='-', label='Gripper')
    target_line = mlines.Line2D([], [], color='black', linestyle='--', label='Target')
    plt.legend(handles=[gripper_line, target_line])
    plt.tight_layout()
    plt.savefig('orientation_euler_plot.png')
    plt.close()

def calculate_velocity(message, window_size=10):
    # Extract the position and orientation values
    position_x = [msg.pose.position.x for msg in message]
    position_y = [msg.pose.position.y for msg in message]
    position_z = [msg.pose.position.z for msg in message]

    position_x_smooth = uniform_filter1d(position_x, size=window_size)
    position_y_smooth = uniform_filter1d(position_y, size=window_size)
    position_z_smooth = uniform_filter1d(position_z, size=window_size)

    orientation_x = [msg.pose.orientation.x for msg in message]
    orientation_y = [msg.pose.orientation.y for msg in message]
    orientation_z = [msg.pose.orientation.z for msg in message]
    orientation_w = [msg.pose.orientation.w for msg in message]

    orientation_x_smooth = uniform_filter1d(orientation_x, size=window_size)
    orientation_y_smooth = uniform_filter1d(orientation_y, size=window_size)
    orientation_z_smooth = uniform_filter1d(orientation_z, size=window_size)
    orientation_w_smooth = uniform_filter1d(orientation_w, size=window_size)

    # Calculate the time stamps
    timestamps = [msg.header.stamp.to_sec() for msg in message]

    # Calculate the velocity
    velocity_x = np.diff(position_x_smooth) / np.diff(timestamps)
    velocity_y = np.diff(position_y_smooth) / np.diff(timestamps)
    velocity_z = np.diff(position_z_smooth) / np.diff(timestamps)

    # Calculate the angular velocity
    orintation = list(zip(orientation_x_smooth, orientation_y_smooth, orientation_z_smooth, orientation_w_smooth))
    euler = [tft.euler_from_quaternion(ori) for ori in orintation]
    orientation_roll = [e[0] for e in euler]
    orientation_pitch = [e[1] for e in euler]
    orientation_yaw = [e[2] for e in euler]

    angular_velocity_x = np.diff(orientation_roll) / np.diff(timestamps)
    angular_velocity_y = np.diff(orientation_pitch) / np.diff(timestamps)
    angular_velocity_z = np.diff(orientation_yaw) / np.diff(timestamps)

    # Apply a moving average to smooth the velocity data
    velocity_x_smooth = uniform_filter1d(velocity_x, size=window_size*5)
    velocity_y_smooth = uniform_filter1d(velocity_y, size=window_size*5)
    velocity_z_smooth = uniform_filter1d(velocity_z, size=window_size*5)

    angular_velocity_x_smooth = uniform_filter1d(angular_velocity_x, size=window_size*5)
    angular_velocity_y_smooth = uniform_filter1d(angular_velocity_y, size=window_size*5)
    angular_velocity_z_smooth = uniform_filter1d(angular_velocity_z, size=window_size*5)

    # Plot the velocity
    plt.figure()
    plt.plot(timestamps[:-1], velocity_x,  alpha=0.3, color='red')
    plt.plot(timestamps[:-1], velocity_x_smooth, label='Velocity X', color='red')
    plt.plot(timestamps[:-1], velocity_y,  alpha=0.3, color='green')
    plt.plot(timestamps[:-1], velocity_y_smooth, label='Velocity Y', color='green')
    plt.plot(timestamps[:-1], velocity_z, alpha=0.3, color='blue')
    plt.plot(timestamps[:-1], velocity_z_smooth, label='Velocity Z', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Linear Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('velocity_plot.png')
    plt.close()

    # Plot the angular velocity
    plt.figure()
    plt.plot(timestamps[:-1], angular_velocity_x, alpha=0.3, color='red')
    plt.plot(timestamps[:-1], angular_velocity_x_smooth, label='Angular Velocity X', color='red')
    plt.plot(timestamps[:-1], angular_velocity_y, alpha=0.3, color='green')
    plt.plot(timestamps[:-1], angular_velocity_y_smooth, label='Angular Velocity Y', color='green')
    plt.plot(timestamps[:-1], angular_velocity_z, alpha=0.3, color='blue')
    plt.plot(timestamps[:-1], angular_velocity_z_smooth, label='Angular Velocity Z', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Angular Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('angular_velocity_plot.png')
    plt.close()

    return velocity_x_smooth, velocity_y_smooth, velocity_z_smooth, angular_velocity_x_smooth, angular_velocity_y_smooth, angular_velocity_z_smooth

# Load the ROS bag
bag_path = 'rosbags/delay_2ms_x0.06_both_robots_all50hz_ry_2t10_2024-08-26-16-31-05.bag'
bag = rosbag.Bag(bag_path, 'r')

# Define how much time to cut from the start and the end
cut_first_x_seconds = 0  # Cut first X seconds
cut_last_y_seconds = 0   # Cut last Y seconds

# Get the actual start and end time of the bag
bag_start_time = bag.get_start_time()
bag_end_time = bag.get_end_time()
print(f"Bag start time: {bag_start_time}")
print(f"Bag end time: {bag_end_time}")

# Calculate the new start and end times after cutting
start_time = bag_start_time + cut_first_x_seconds
end_time = bag_end_time - cut_last_y_seconds
reference_start_time = get_topic_start_time(bag, '/action_topic')

# Get bag info and print it
info = get_bag_info(bag)
print(f"Bag Info: {info}")

# Extract data from a topic
action_data = extract_topic_data(bag, '/action_topic')
plot_action_topic(action_data)
gripper_pose_topic = '/capture_c_a_tool1/pose'
gripper_pose_data = extract_shifted_topic_data(bag, gripper_pose_topic, reference_start_time)
velocity_x, velocity_y, velocity_z, angular_velocity_x, angular_velocity_y, angular_velocity_z = calculate_velocity(gripper_pose_data)
# print(f"Velocity X: {velocity_x}")
target_pose_topic = '/capture_ot_offset/pose'
target_pose_data = extract_shifted_topic_data(bag, target_pose_topic, reference_start_time)
plot_pose_topic_w_ref(gripper_pose_data, target_pose_data)

topic = '/Way_pts_target_pose'
data = extract_shifted_topic_data(bag, topic, reference_start_time)
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
