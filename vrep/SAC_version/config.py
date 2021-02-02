import numpy as np

ON_TRAIN = True
render = True


method = 'sac'
sac = True
hidden_sizes=[256,256]
reparameterize_critic=False
reparameterize_actor=True

state_dim = 7
action_dim = 4
a_bound = [0.02,0.02,0.02,0.02]

PATH_EVAL = ['01261124', '98']  # for eval

MAX_EPISODES = 500
MAX_EP_STEPS = 500

A_LR = 0.001
C_LR = 0.001
gamma = 0.99  # reward discount
reward_scale =0.01

tau = 0.005  # soft replacement
MEMORY_CAPACITY = 100000
batch_size = 128

eval_iteration = 5  #存model

initial_joint=[0,0,0,0,0,0]


#checkpoint_path = os.path.join('./model/' + path + '/net/' + str(int(i)))





