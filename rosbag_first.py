import rosbag
import matplotlib.pyplot as plt
import rospy
from datetime import datetime

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


# Load the ROS bag
bag_path = 'rosbags/delay_2ms_x0.06_both_robots_all50Hz_ry_p5_t11_2024-08-26-16-44-27.bag'
bag = rosbag.Bag(bag_path)

# Get bag info and print it
info = get_bag_info(bag)
print(f"Bag Info: {info}")

# Extract data from a topic
action_data = extract_topic_data(bag, '/action_topic')
topic = '/capture_c_a_tool1/pose'
data = extract_topic_data(bag, topic)
print(f"Type of data: {type(data)}") # <class 'list'>
print(f"Number of messages: {len(data)}") # 17218
print(f"Number of fields in a message: {len(data[0].__slots__)}") # 2
print(f"Type of message: {type(data[0])}") # <class 'tmprpp995o7._geometry_msgs__TwistStamped'>

# Print details of the extracted messages
# print_action_topic_messages(data)

# Plot the action topic
plot_action_topic(action_data)


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
