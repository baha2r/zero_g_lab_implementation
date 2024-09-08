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
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
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
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
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

    # Plot the velocity
    plt.figure()
    plt.plot(timestamps[:-1], gripper_velocity_x,  alpha=0.3, color='red')
    plt.plot(timestamps[:-1], gripper_velocity_x_smooth, label='Velocity X', color='red')
    plt.plot(timestamps[:-1], gripper_velocity_y,  alpha=0.3, color='green')
    plt.plot(timestamps[:-1], gripper_velocity_y_smooth, label='Velocity Y', color='green')
    plt.plot(timestamps[:-1], gripper_velocity_z, alpha=0.3, color='blue')
    plt.plot(timestamps[:-1], gripper_velocity_z_smooth, label='Velocity Z', color='blue')
    plt.plot(timestamps[:-1], target_velocity_x,  alpha=0.3, color='orange')
    plt.plot(timestamps[:-1], target_velocity_x_smooth, label='Target Velocity X', color='orange')
    plt.plot(timestamps[:-1], target_velocity_y,  alpha=0.3, color='purple')
    plt.plot(timestamps[:-1], target_velocity_y_smooth, label='Target Velocity Y', color='purple')
    plt.plot(timestamps[:-1], target_velocity_z, alpha=0.3, color='cyan')
    plt.plot(timestamps[:-1], target_velocity_z_smooth, label='Target Velocity Z', color='cyan')
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
    plt.plot(timestamps[:-1], gripper_angular_velocity_x, alpha=0.3, color='red')
    plt.plot(timestamps[:-1], gripper_angular_velocity_x_smooth, label='Angular Velocity X', color='red')
    plt.plot(timestamps[:-1], gripper_angular_velocity_y, alpha=0.3, color='green')
    plt.plot(timestamps[:-1], gripper_angular_velocity_y_smooth, label='Angular Velocity Y', color='green')
    plt.plot(timestamps[:-1], gripper_angular_velocity_z, alpha=0.3, color='blue')
    plt.plot(timestamps[:-1], gripper_angular_velocity_z_smooth, label='Angular Velocity Z', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Angular Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.legend()
    plt.tight_layout()
    plt.savefig('angular_velocity_plot.png')
    plt.close()

    # return velocity_x_smooth, velocity_y_smooth, velocity_z_smooth, angular_velocity_x_smooth, angular_velocity_y_smooth, angular_velocity_z_smooth

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
        gripper_quat = [gripper_msg.pose.orientation.x, gripper_msg.pose.orientation.y, gripper_msg.pose.orientation.z, gripper_msg.pose.orientation.w]
        target_quat = [target_msg.pose.orientation.x, target_msg.pose.orientation.y, target_msg.pose.orientation.z, target_msg.pose.orientation.w]
        
        # Compute angular difference
        dot_product = np.dot(gripper_quat, target_quat)
        angular_difference = 2 * np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clipped for numerical stability
        orientation_differences.append(angular_difference)

    return distances, orientation_differences, timestamps

def plot_distance_and_orientation(distances, orientation_differences, timestamps):
    # Plot distances
    plt.figure()
    plt.plot(timestamps, distances, label='Distance', color='blue')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Distance between Gripper and Target over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend()
    plt.tight_layout()
    plt.savefig('distance_plot.png')
    plt.close()

    # Plot orientation differences
    plt.figure()
    plt.plot(timestamps, orientation_differences, label='Orientation Difference', color='orange')
    plt.axhline(0, color='black', linestyle='--', linewidth=.1)
    plt.title('Orientation Difference between Gripper and Target over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation Difference (radians)')
    plt.legend()
    plt.tight_layout()
    plt.savefig('orientation_difference_plot.png')
    plt.close()

# Load the ROS bag
bag_path = 'rosbags/delay_2ms_x0.06_both_robots_all50Hz_ry_p5_t11_2024-08-26-16-44-27.bag'
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
target_pose_topic = '/capture_ot_offset/pose'
target_pose_data = extract_shifted_topic_data(bag, target_pose_topic, reference_start_time)
target_way_pts_topic = '/Way_pts_target_pose'
target_way_pts_data = extract_shifted_topic_data(bag, target_way_pts_topic, reference_start_time)
chaser_way_pts_topic = '/Way_pts_chaser_pose'
chaser_way_pts_data = extract_shifted_topic_data(bag, chaser_way_pts_topic, reference_start_time)
plot_pose_topic_w_ref(gripper_pose_data, target_pose_data)

calculate_velocity(gripper_pose_data, target_pose_data)

distances, orientation_differences, timestamps = calculate_distance_and_orientation_difference(gripper_pose_data, target_pose_data)
plot_distance_and_orientation(distances, orientation_differences, timestamps)

# plot_pose_topic_w_ref(target_way_pts_data, gripper_pose_data)
# print(f"Type of data: {type(data)}") # <class 'list'>
# print(f"Number of messages: {len(data)}") # 17218
# print(f"Number of fields in a message: {len(data[0].__slots__)}") # 2
# print(f"Type of message: {type(data[0])}") # <class 'tmprpp995o7._geometry_msgs__TwistStamped'> <class 'tmp97szykb4._geometry_msgs__PoseStamped'>
# plot_pose_topic(data)

# Close the bag after use
bag.close()
