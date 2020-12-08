import numpy as np

ON_TRAIN = True
method = 'sac'

reparameterize_critic=False
reparameterize_actor=True
PATH_EVAL = ['12012132', '1']  # for eval

MAX_EPISODES = 400
MAX_EP_STEPS = 400

A_LR = 0.003
C_LR = 0.003
gamma = 0.99  # reward discount
reward_scale =0.5

tau = 0.005  # soft replacement
MEMORY_CAPACITY = 1000000
layer1_size = 256
layer2_size = 256
# layer3_size=256
batch_size = 256

eval_iteration = 10


#checkpoint_path = os.path.join('./model/' + path + '/net/' + str(int(i)))





