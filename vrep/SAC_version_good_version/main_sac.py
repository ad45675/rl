
from env_new2 import robot_env
import numpy as np
from sac4 import SAC_agent
# from utils import plot_learning_curve
import config
import time
import os
import matplotlib.pyplot as plt
import math

MAX_EPISODES = config.MAX_EPISODES
MAX_EP_STEPS = config.MAX_EP_STEPS
eval_iteration = config.eval_iteration
PATH = time.strftime('%m%d%H%M')
ON_TRAIN = config.ON_TRAIN

# for eval
PATH_EVAL = config.PATH_EVAL

env=robot_env()
#a_bound = [env.joint1_bound[1], env.joint2_bound[1], env.joint3_bound[1], env.joint5_bound[1]]
a_bound = [0.02,0.02,0.02]
print('a_bound',a_bound)
s_dim = env.state_dim
a_dim = env.action_dim
print('s_dim', s_dim,'a_dim',a_dim)
agent = SAC_agent(s_dim, a_dim, a_bound)

if ON_TRAIN:
    Mode = 'Train'
else:
    Mode = 'Eval'

def train():


    # uncomment this line and do a mkdir tmp && mkdir video if you want to
    # record video of the agent playing the game.
    #env = wrappers.Monitor(env, 'tmp/video', video_callable=lambda episode_id: True, force=True)
    best_score = float("-inf")


    load_checkpoint = False

    if load_checkpoint:
        agent.load_models(PATH_EVAL)
    env.initial()
    # agent.load_models(PATH_EVAL)
    #------record memory------#
    step_set, reward_set, avg_reward_set, state_set ,action_set,joint1_set,joint2_set,joint3_set,joint5_set,distance_set= [], [], [], [], [], [], [], [], [], []
    for i in range(MAX_EPISODES):
        s = env.reset()
        done = False
        ep_r = 0
        for j in range(MAX_EP_STEPS):
            a = agent.choose_action(s)
            # print('a',a*180/3.14)

            action_set.append(a)

            s_, r, done= env.step(a)

            # print('s_',s_,'r',r)
            agent.remember(s, a ,r, s_, done)

            ep_r += r
            if ep_r<-250:
                done=True

            if not load_checkpoint:
                agent.learn()


            s = s_
            state_set.append(s)

            # score_history.append(ep_r)
            # avg_score = np.mean(score_history[-100:])
            #
            # if avg_score > best_score:
            #     best_score = avg_score

            if done or j == MAX_EP_STEPS - 1:
                print('episode: %i | %s | ep_r %.1f |step:%i' % (i, '...' if not done else 'done', ep_r, j + 1))
                step_set.append(j + 1)
                reward_set.append(ep_r)
                avg_reward_set.append(ep_r / (j + 1))
                # avg_score_set.append(avg_score )
                # best_score_set.append(best_score)
                break

        if i % eval_iteration == 0:
            agent.save_models(PATH, i / eval_iteration)

    joint1_set = [x[0] for x in action_set]
    joint2_set = [x[1] for x in action_set]
    joint3_set = [x[2] for x in action_set]
    distance_set = [x[6] for x in state_set]

    # ----- record data ----- #
    folder_data = './model/' + PATH + '/train/data/'
    file_name = ['step.txt', 'reward.txt', 'avgR.txt', 'joint1.txt','joint2.txt','joint3.txt','distance.txt','state_set.txt']
    data_set = ['step_set', 'reward_set','avg_reward_set','joint1_set','joint2_set','joint3_set','distance_set','state_set']
    for i in range(len(file_name)):
        save_txt(path=folder_data, name=file_name[i], data=eval(data_set[i]))


    # ----- plot fig -----
    folder_fig = './model/' + PATH + '/train/fig/'
    xlabel = ['Episode', 'Episode', 'Episode', 'Episode', 'Episode','Episode','Episode','Episode']
    ylabel = ['step', 'r', 'avgR','joint1','joint2','joint3','distance','state']
    title = ['step', 'reward', 'avgR','joint1','joint2','joint3','distance','state']
    for i in range(len(file_name)):
        plot_txt(path=folder_data, name=file_name[i], xlabel=xlabel[i], ylabel=ylabel[i], title=title[i],
                 save_location=folder_fig)


def Eval():

    agent.load_models(PATH_EVAL)
    env.initial()
    s=env.reset()
    for i in range(100):
        a=agent.choose_action(s)
        s_,r,done=env.step(a)
        s=s_
        print(i)

def plot(data, xlabel, ylabel, title, save_location):
    plt.plot(np.arange(data.shape[0]), data)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.savefig(save_location+title)
    plt.clf()
    plt.close()

    # if not load_checkpoint:
    #     x = [i+1 for i in range(MAX_EPISODES)]
    #     plot_learning_curve(x, score_history, figure_file)
def plot_txt(path, name, xlabel, ylabel, title, save_location):
    # path exist
    if path_exsit(path+name):
        # load data
        data = load_txt(path=path, name=name)

        # plot
        plot(data=data, xlabel=xlabel, ylabel=ylabel, title=title, save_location=save_location)
    else:
        print(path+name +' does not exist')

def load_txt(path, name):
    f = open(path + name, 'r')
    data = np.loadtxt(f)
    f.close()
    return data





def main():
    if ON_TRAIN:
        creat_path('./model/' + PATH + '/train/data')
        creat_path('./model/' + PATH + '/train/fig')
        creat_path('./model/' + PATH + '/test')
        save_parameter()
        start_time = time.time()
        train()
        end_time = time.time()
        cost_time = np.array([[end_time - start_time]])
        save_txt(path='./model/' + PATH + '/train/', name='time.txt', data=cost_time, fmt='%f')
    else:
        Eval()

def creat_path(path):
    if path_exsit(path=path):
        print(path+' exist')
    else:
        os.makedirs(path)

def path_exsit(path):
    if os.path.exists(path):
        return True
    else:
        return False
def save_txt(path, name, data, fmt='%f'):
    f = open(path + name, 'w')
    np.savetxt(f, data, fmt=fmt)
    f.close()

def save_parameter():
    with open('./model/' + PATH + '/train/parameter.txt', 'w') as f:

        f.writelines("Method: {}\n".format('sac'))
        # f.writelines("epsilon: {}\n".format(config.epsilon))
        f.writelines("Max Episodes: {}\nMax Episodes steps: {}\n".format(MAX_EPISODES, MAX_EP_STEPS))
        f.writelines("LR_A: {}\nLR_C: {}\nGAMMA: {}\n".format(config.A_LR, config.C_LR, config.gamma))
        f.writelines("reward_scale: {}\n".format(config.reward_scale))
        # f.writelines("A_UPDATE_STEPS: {}\nC_UPDATE_STEPS: {}\n".format(config.A_UPDATE_STEPS,config.C_UPDATE_STEPS))
        f.writelines("BATCH_SIZE: {}\n".format(config.batch_size))
        # f.writelines("layer1_size: {}\nlayer2_size: {}\n".format(config.layer1_size, config.layer2_size))
        # f.writelines("layer_critic: {}\nneurons_critic: {}\n".format(config.layer_critic, config.neurons_critic))
        # f.writelines("Tolerance: {}\n".format(config.Tolerance))

if __name__ == "__main__":

    main()