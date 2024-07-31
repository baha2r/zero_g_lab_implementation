import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# import os
# import sys

# Read data
# header = pd.read_csv('evaluation/simulation/data/output_data1.csv', nrows=0).columns.tolist()
# instead of read_csv, use pandas to read pkl file
db = pd.read_pickle('data.pkl')
# print(db) 
# db = pd.read_csv('evaluation/simulation/data/output_data1.csv',header=0)
# delete the last row
# db = db[:,0]
# db.columns = header

# Setting up the figure and subplots
fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(30, 20))
fig.tight_layout(pad=5.0)  # To ensure plots don't overlap with each other
font_size = 18
title_size = 22
pad = 1
solid_line = plt.Line2D([0], [0], color='black', linestyle='-', label='Gripper')
dashed_line = plt.Line2D([0], [0], color='black', linestyle='--', label='Target')

# Displacement Action
position_action = [x["position_action"] for x in db]
x_position_action = [x[0] for x in position_action][5:]
y_position_action = [x[1] for x in position_action][5:]
z_position_action = [x[2] for x in position_action][5:]
axes[0, 0].plot(x_position_action, color='r')
axes[0, 0].plot(y_position_action, color='g')
axes[0, 0].plot(z_position_action, color='b')
axes[0, 0].set_title('Displacement Action', fontsize=title_size)
axes[0, 0].set_ylabel('x 0.01 $m$', fontsize=font_size, labelpad=-10)
# axes[0, 0].set_xlabel('timestep', fontsize=font_size)
axes[0, 0].legend(['x', 'y', 'z'], fontsize=font_size, labelspacing=0.00001)
axes[0, 0].tick_params(labelsize=font_size, pad=pad)

# Rotation Action
orientation_action = [x["orientation_action"] for x in db]
roll_orientation_action = [x[0] for x in orientation_action][5:]
pitch_orientation_action = [x[1] for x in orientation_action][5:]
yaw_orientation_action = [x[2] for x in orientation_action][5:]
axes[1, 0].plot(roll_orientation_action, color='r')
axes[1, 0].plot(pitch_orientation_action, color='g')
axes[1, 0].plot(yaw_orientation_action, color='b')
axes[1, 0].set_title('Rotation Action', fontsize=title_size)
axes[1, 0].set_ylabel('x 0.1 $rad$', fontsize=font_size, labelpad=-10)
# axes[1, 0].set_xlabel('timestep', fontsize=font_size)
axes[1, 0].legend(['roll', 'pitch', 'yaw'], fontsize=font_size, labelspacing=0.00001)
axes[1, 0].tick_params(labelsize=font_size, pad=pad)

# Gripper vs Target Position
gripper_position = [x["gripper_position"] for x in db]
block_position = [x["block_position"] for x in db]
x_gripper_position = [x[0] for x in gripper_position][5:]
y_gripper_position = [x[1] for x in gripper_position][5:]
z_gripper_position = [x[2] for x in gripper_position][5:]
x_block_position = [x[0] for x in block_position][5:]
y_block_position = [x[1] for x in block_position][5:]
z_block_position = [x[2] for x in block_position][5:]
axes[0, 1].plot(x_gripper_position, color='r')
axes[0, 1].plot(y_gripper_position, color='g')
axes[0, 1].plot(z_gripper_position, color='b')
axes[0, 1].plot(x_block_position, linestyle='--', color='r')
axes[0, 1].plot(y_block_position, linestyle='--', color='g')
axes[0, 1].plot(z_block_position, linestyle='--', color='b')
axes[0, 1].set_title('Gripper vs Target Position', fontsize=title_size)
axes[0, 1].set_ylabel('$m$', fontsize=font_size, labelpad=-10)
axes[0, 1].legend(handles=[solid_line, dashed_line], fontsize=font_size, labelspacing=0.00001)
axes[0, 1].tick_params(labelsize=font_size, pad=pad)
# axes[0, 2].legend(['x gripper', 'y gripper', 'z gripper', 'x target', 'y target', 'z target'])

# Gripper vs Target Rotation
gripper_orientation = [x["gripper_orientation"] for x in db]
block_orientation = [x["block_orientation"] for x in db]
roll_gripper_orientation = [x[0] for x in gripper_orientation][5:]
pitch_gripper_orientation = [x[1] for x in gripper_orientation][5:]
yaw_gripper_orientation = [x[2] for x in gripper_orientation][5:]
roll_block_orientation = [x[0] for x in block_orientation][5:]
pitch_block_orientation = [x[1] for x in block_orientation][5:]
yaw_block_orientation = [x[2] for x in block_orientation][5:]
axes[1, 1].plot(roll_gripper_orientation, color='r')
axes[1, 1].plot(pitch_gripper_orientation, color='g')
axes[1, 1].plot(yaw_gripper_orientation, color='b')
axes[1, 1].plot(roll_block_orientation, linestyle='--', color='r')
axes[1, 1].plot(pitch_block_orientation, linestyle='--', color='g')
axes[1, 1].plot(yaw_block_orientation, linestyle='--', color='b')
axes[1, 1].set_title('Gripper vs Target Rotation', fontsize=title_size)
axes[1, 1].set_ylabel('$rad$', fontsize=font_size, labelpad=pad)
axes[1, 1].legend(handles=[solid_line, dashed_line], fontsize=font_size, labelspacing=0.00001)
axes[1, 1].tick_params(labelsize=font_size, pad=pad)
# axes[0, 3].legend(['roll gripper', 'pitch gripper', 'yaw gripper', 'roll target', 'pitch target', 'yaw target'])

# Gripper vs Target Velocity
gripper_linear_velocity = [x["gripper_linear_velocity"] for x in db]
block_linear_velocity = [x["block_linear_velocity"] for x in db]
x_gripper_linear_velocity = [x[0] for x in gripper_linear_velocity][5:]
y_gripper_linear_velocity = [x[1] for x in gripper_linear_velocity][5:]
z_gripper_linear_velocity = [x[2] for x in gripper_linear_velocity][5:]
x_block_linear_velocity = [x[0] for x in block_linear_velocity][5:]
y_block_linear_velocity = [x[1] for x in block_linear_velocity][5:]
z_block_linear_velocity = [x[2] for x in block_linear_velocity][5:]
axes[0, 2].plot(x_gripper_linear_velocity, color='r')
axes[0, 2].plot(y_gripper_linear_velocity, color='g')
axes[0, 2].plot(z_gripper_linear_velocity, color='b')
axes[0, 2].plot(x_block_linear_velocity, linestyle='--', color='r')
axes[0, 2].plot(y_block_linear_velocity, linestyle='--', color='g')
axes[0, 2].plot(z_block_linear_velocity, linestyle='--', color='b')
axes[0, 2].set_title('Gripper vs Target Velocity', fontsize=title_size)
axes[0, 2].set_ylabel('$m/s$', fontsize=font_size, labelpad=pad)
axes[0, 2].set_xlabel('timestep', fontsize=font_size)
axes[0, 2].legend(handles=[solid_line, dashed_line], fontsize=font_size, labelspacing=0.00001)
axes[0, 2].tick_params(labelsize=font_size, pad=pad)
# axes[1, 0].legend([r'$\dot{x}$ gripper', r'$\dot{y}$ gripper', r'$\dot{z}$ gripper', 
#                    r'$\dot{x}$ target', r'$\dot{y}$ target', r'$\dot{z}$ target'])

# Gripper vs Target angular Velocity
gripper_angular_velocity = [x["gripper_angular_velocity"] for x in db]
block_angular_velocity = [x["block_angular_velocity"] for x in db]
x_gripper_angular_velocity = [x[0] for x in gripper_angular_velocity][5:]
y_gripper_angular_velocity = [x[1] for x in gripper_angular_velocity][5:]
z_gripper_angular_velocity = [x[2] for x in gripper_angular_velocity][5:]
x_block_angular_velocity = [x[0] for x in block_angular_velocity][5:]
y_block_angular_velocity = [x[1] for x in block_angular_velocity][5:]
z_block_angular_velocity = [x[2] for x in block_angular_velocity][5:]
axes[1, 2].plot(x_gripper_angular_velocity, color='r')
axes[1, 2].plot(y_gripper_angular_velocity, color='g')
axes[1, 2].plot(z_gripper_angular_velocity, color='b')
axes[1, 2].plot(x_block_angular_velocity, linestyle='--', color='r')
axes[1, 2].plot(y_block_angular_velocity, linestyle='--', color='g')
axes[1, 2].plot(z_block_angular_velocity, linestyle='--', color='b')
axes[1, 2].set_title('Gripper vs Target Angular Velocity', fontsize=title_size)
axes[1, 2].set_ylabel('$rad/s$', fontsize=font_size, labelpad=pad)
axes[1, 2].set_xlabel('timestep', fontsize=font_size)
axes[1, 2].legend(handles=[solid_line, dashed_line], fontsize=font_size, labelspacing=0.00001)
axes[1, 2].tick_params(labelsize=font_size, pad=pad)

# Closest Distance
closest_points = [np.linalg.norm(x["closest_points"]) for x in db]
axes[2, 0].plot(closest_points)
axes[2, 0].set_title('Closest Distance', fontsize=title_size)
axes[2, 0].set_ylabel('$m$', fontsize=font_size, labelpad=pad)
axes[2, 0].set_xlabel('timestep', fontsize=font_size)
axes[2, 0].tick_params(labelsize=font_size, pad=pad)

# Reward
position_reward = [x["positioning_reward"] for x in db]
axes[2, 1].plot(position_reward)
axes[2, 1].set_title('Reward', fontsize=title_size)
# axes[1, 2].set_ylabel('x 10mm', fontsize=font_size)
axes[2, 1].set_xlabel('timestep', fontsize=font_size)
axes[2, 1].tick_params(labelsize=font_size, pad=pad)

# Contact Force
# axes[2, 2].plot(db["contact force"])
# axes[2, 2].set_title('Contact Force', fontsize=title_size)
# axes[2, 2].set_ylabel('$N$', fontsize=font_size, labelpad=-10)
# axes[2, 2].set_xlabel('timestep', fontsize=font_size)
# axes[2, 2].tick_params(labelsize=font_size, pad=pad)


# save the plot as a file in png
plt.savefig('plot.png')