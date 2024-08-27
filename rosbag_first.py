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
    plt.show()

# Load the ROS bag
bag_path = 'path_to_your_rosbag.bag'
bag = rosbag.Bag(bag_path)

# Get bag info and print it
info = get_bag_info(bag)
print(f"Bag Info: {info}")

# Choose a topic to extract headers from
topic = '/your_topic_name'

# Get and plot headers
headers = get_headers(bag, topic)
plot_headers(headers)

# Close the bag after use
bag.close()
