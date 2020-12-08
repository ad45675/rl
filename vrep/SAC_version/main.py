# env = gym.make('CartPole-v0').unwrapped
# print(env.action_space)  # 输出动作信息
# print(env.action_space.n)  # 输出动作个数
# print(env.observation_space)  # 查看状态空间
# print(env.observation_space.shape[0])  # 输出状态个数
# print(env.observation_space.high)  # 查看状态的最高值
# print(env.observation_space.low)  # 查看状态的最低值
# Discrete(2)
# 2
# Box(4,)
# 4
# [4.8000002e+00 3.4028235e+38 4.1887903e-01 3.4028235e+38]
# [-4.8000002e+00 -3.4028235e+38 -4.1887903e-01 -3.4028235e+38]


"""
ya0000

2020/11/23

"""




from env_new import robot_env
# import gym
import numpy as np
#from sac import SAC_agent
#from utils import plot_learning_curve
# from gym import wrappers
#from utils import plot_learning_curve
import config
import time
from RL_brain import DDPG


ON_TRAIN = config.ON_TRAIN


if ON_TRAIN:
    Mode = 'Train'
else:
    Mode = 'Eval'


#for train
MAX_EPISODES = config.MAX_EPISODES

MAX_EP_STEPS = config.MAX_EP_STEPS
print(MAX_EPISODES,MAX_EP_STEPS)
eval_iteration = config.eval_iteration
cost_iteration = config.cost_iteration
PATH = time.strftime('%m%d%H%M')

#for eval
#PATH=config.PATH_EVAL

#set env
env=robot_env()
#RL method(continuous)

s_dim=env.state_dim
a_dim=env.action_dim
# a_bound=[env.joint1_bound[1],env.joint2_bound[1],env.joint3_bound[1],env.joint4_bound[1],env.joint5_bound[1],env.joint6_bound[1]]
a_bound=[env.joint1_bound[1],env.joint2_bound[1],env.joint3_bound[1],env.joint5_bound[1]]
# a_bound = [[env.arm1_bound[1], env.arm2_bound[1]]]


#
# rl = Agent(alpha=0.0003, beta=0.0003, reward_scale=2, env_id=env_id,
#                 input_dims=env.observation_space.shape, tau=0.005,
#                 env=env, batch_size=256, layer1_size=256, layer2_size=256,
#                 n_actions=env.action_space.shape[0])
rl = DDPG(a_dim, s_dim, a_bound)

def train():

    view=True
    step_set, reward_set, avg_reward_set, repeat_set, cost_actor_set, cost_critic_set = [], [], [], [], [], []
    for i in range(MAX_EPISODES):
        s=env.reset()
        ep_r,count_cost_store,count_repeat=0,0,0
        for j in range(MAX_EP_STEPS):
            view=True
            if view:
                env.render()
            
            a=rl.choose_action(s)
            
            s_,r,done=env.step(a)

            rl.store_transition(s,a,r,s_)

            ep_r+=r

            s=s_

            if rl.memory_full:

                rl.learn()

                cost_actor,cost_critic=rl.learn()
                count_cost_store+=1
                if count_cost_store % cost_iteration==0:
                    cost_actor_set.append(cost_actor)
                    cost_critic_set.append(cost_critic)

            if done or j==MAX_EP_STEPS-1:

                print('Ep: %i | %s | repeat: %d | ep_r: %.1f | step: %i' % (i, '----' if not done else 'done', count_repeat, ep_r, j+1))
                step_set.append(j+1)
                reward_set.append(ep_r)
                avg_reward_set.append(ep_r/(j+1))           
                break



def main():
    if ON_TRAIN:
        # creat_path('./model/' + PATH + '/train/data')
        # creat_path('./model/' + PATH + '/train/fig')
        # # creat_path('./model/' + PATH + '/test')
        # save_parameter()
        start_time = time.time()
        train()
        end_time = time.time()
        cost_time = np.array([[end_time - start_time]])
       # save_txt(path='./model/' + PATH + '/train/', name='time.txt', data=cost_time, fmt='%f')
    else:
        Eval()


if __name__ == "__main__":
    main()