#!/usr/bin/env python3

import numpy as np
from stable_baselines3 import SAC
from create_obs import create_robot_obs_array

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
    obs = create_robot_obs_array()
    action = get_action(model, obs)
    return action

if __name__ == '__main__':
    return_action()