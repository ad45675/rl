import numpy as np

ON_TRAIN = False
method = 'sac'
sac = True
hidden_sizes=[256,256]
reparameterize_critic=False
reparameterize_actor=True
PATH_EVAL = ['01261124', '98']  # for eval

MAX_EPISODES = 500
MAX_EP_STEPS = 500

A_LR = 0.001
C_LR = 0.001
gamma = 0.99  # reward discount
reward_scale =0.01

tau = 0.005  # soft replacement
MEMORY_CAPACITY = 100000
# layer1_size = 256
# layer2_size = 256
# layer3_size=256
batch_size = 128

eval_iteration = 5


#checkpoint_path = os.path.join('./model/' + path + '/net/' + str(int(i)))





