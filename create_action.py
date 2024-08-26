#!/usr/bin/env python3
import os
ros_master_uri = "http://192.168.88.11:11311"
os.environ['ROS_MASTER_URI'] = ros_master_uri

import numpy as np
import socket
import struct
from stable_baselines3 import SAC
from create_obs_new import create_robot_obs_array
from pose_listener import PoseListener
import time
import pickle
# Define the server address and port
server_address = ('192.168.88.11', 65432)

# Create a TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def load_model():
    """Load the pre-trained SAC model."""
    model = SAC.load("./models/trained_agent/best_model.zip")
    return model

def get_action(model, obs):
    """Predict the next action based on the observation using the SAC model."""
    action, _states = model.predict(obs, deterministic=True)
    return action

def return_action():
    model = load_model()
    pose_listener = PoseListener()
    pose_listener.start_listening()
    # Connect to the server
    sock.connect(server_address)
    time.sleep(1)
    try:
        while True:
            obs = create_robot_obs_array(pose_listener)
            print("obs: ", obs)
            action = get_action(model, obs)
            action = list(action)
            data = pickle.dumps(action, protocol=2)
            sock.sendall(data)
            time.sleep(.002)
    except KeyboardInterrupt:
        print("Shutting down.")


if __name__ == '__main__':
    return_action()