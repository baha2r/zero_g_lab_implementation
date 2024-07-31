import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'
from pathlib import Path
import sys
import gymnasium
sys.modules["gym"] = gymnasium
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
from robotiqGymEnv import robotiqGymEnv
from stable_baselines3 import SAC

repo_root = Path(__file__).resolve().parent

def main():

  env = robotiqGymEnv(records=False, renders=False)

  # mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
  # print(mean_reward)

  model = SAC.load(f'{repo_root}/models/trained_agent/best_model.zip')

  dones = False
  obs = env.reset()
  t = 0
 
  while not dones:
  # while t < 10:
      # xtargetvel = p.getBaseVelocity(env.blockUid)[0][0]
      # ytargetvel = p.getBaseVelocity(env.blockUid)[0][1]
      # ztargetvel = p.getBaseVelocity(env.blockUid)[0][2]
      # print("xvel: ", xtargetvel)
      # print("yvel: ", ytargetvel)
      # print("zvel: ", ztargetvel)
      # if env._env_step_counter < 300:
    action = model.predict(obs, deterministic=True)[0]
    # print(action)
    # action = [0 , 0 , 0 , 0 , 0 , 0]
      # else:
      #   action = [0 , 0 , 0 , 0 , 0 , 0 ]
      # action = env.action_space.sample()
    # action = [0 , 0 , 0 , 0 , 0 , 0]
    obs, rewards, dones, info = env.step(action)
    print("obs: ", obs)
      # targetspeed = p.getBaseVelocity(env.blockUid)
      # print(p.getAABB(env.blockUid))
      # print((p.getBasePositionAndOrientation(env._robotiq.robotiqUid)[1]))
      # print(p.getBasePositionAndOrientation(env.blockUid)[0])
      # print(p.getBaseVelocity(env.blockUid)[0])
      # print(p.getBaseVelocity(env._robotiq.robotiq_uid)[0])
      # print(targetspeed)
      # print(len(env._robotiq.linkpos))
      # print(env._contactinfo()[4])
      # env.render()
  print("info: ", info)
    # t += 1


if __name__ == "__main__":
  main()