import numpy as np

ON_TRAIN =True
render = True


method = 'sac'
sac = True
hidden_sizes=[256,256,128,64]
reparameterize_critic=False
reparameterize_actor=True

state_dim = 10
action_dim = 4
a_bound = [0.02,0.02,0.02,0.02]

PATH_EVAL = ['02041551', '9']  # for eval

MAX_EPISODES = 500
MAX_EP_STEPS = 800

A_LR = 0.001
C_LR = 0.001
gamma = 0.99  # reward discount
reward_scale =0.05

tau = 0.005  # soft replacement
MEMORY_CAPACITY = 200000
batch_size = 256

eval_iteration = 5  #存model

initial_joint=[0,0,0,0,0,0]


#checkpoint_path = os.path.join('./model/' + path + '/net/' + str(int(i)))





