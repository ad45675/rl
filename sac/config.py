import numpy as np


ON_TRAIN = True
method=DDPG



epsilon = 0.2

MAX_EPISODES = 20
MAX_EP_STEPS = 200
eval_iteration = 10

A_LR = 0.00001    # learning rate for actor
C_LR = 0.00002    # learning rate for critic
GAMMA = 0.9     # reward discount
A_UPDATE_STEPS = 10
C_UPDATE_STEPS = 10
BATCH = 32
neurons_actor = [64, 128, 64]
layer_actor = np.size(neurons_actor) + 1
neurons_critic = [64, 128, 256, 128]
layer_critic = np.size(neurons_critic) + 1

Tolerance = 5

# for performance
validation_size = 200
