# tensorflow 2.0 version



import tensorflow as tf
import numpy as np
import config
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
# import tensorflow.compat.v1 as tf
tf.compat.v1.disable_eager_execution()
tf.compat.v1.disable_v2_behavior()

####################  hyper parameters  ####################

LR_A = config.LR_A    # learning rate for actor
LR_C = config.LR_C    # learning rate for critic
GAMMA = config.GAMMA     # reward discount
TAU = config.TAU      # soft replacement
MEMORY_CAPACITY = config.MEMORY_CAPACITY  # memory size
BATCH_SIZE = config.BATCH_SIZE  # batch size
layer_actor = config.layer_actor  # Actor Network layer
neurons_actor = config.neurons_actor  # Actor Network Design
layer_critic = config.layer_critic  # Critic Network layer
neurons_critic = config.neurons_critic  # Critic Network Design

class DDPG(object):
    def __init__(self, a_dim, s_dim, a_bound):
        self.memory = np.zeros((MEMORY_CAPACITY, s_dim * 2 + a_dim + 1), dtype=np.float32)  # ( s, s_, a, r )
        self.pointer = 0
        self.memory_full = False
        # self.sess = tf.Session()
        # self.sess = tf.compat.v1.Session(config=tf.ConfigProto(log_device_placement=True))
        self.sess = tf.compat.v1.Session()
        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound
        self.S = tf.compat.v1.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.compat.v1.placeholder(tf.float32, [None, s_dim], 's_')
        self.R = tf.compat.v1.placeholder(tf.float32, [None, 1], 'r')

        # Create Network Architecture
        with tf.compat.v1.variable_scope('Actor'):
            self.a = self._build_a(self.S, scope='eval', trainable=True)
            a_ = self._build_a(self.S_, scope='target', trainable=False)

        with tf.compat.v1.variable_scope('Critic'):
            # assign self.a = a in memory when calculating q for td_error,
            # otherwise the self.a is from Actor when updating Actor
            q = self._build_c(self.S, self.a, scope='eval', trainable=True)
            q_ = self._build_c(self.S_, a_, scope='target', trainable=False)

        # networks parameters
        self.ae_params = tf.compat.v1.get_collection(tf.compat.v1.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval')
        self.at_params = tf.compat.v1.get_collection(tf.compat.v1.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target')
        self.ce_params = tf.compat.v1.get_collection(tf.compat.v1.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval')
        self.ct_params = tf.compat.v1.get_collection(tf.compat.v1.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target')

        # target net replacement
        self.soft_replace = [[tf.compat.v1.assign(ta, (1 - TAU) * ta + TAU * ea), tf.compat.v1.assign(tc, (1 - TAU) * tc + TAU * ec)]
                             for ta, ea, tc, ec in zip(self.at_params, self.ae_params, self.ct_params, self.ce_params)]

        q_target = self.R + GAMMA * q_
        self.td_error = tf.compat.v1.losses.mean_squared_error(labels=q_target, predictions=q)
        self.ctrain = tf.compat.v1.train.AdamOptimizer(LR_C).minimize(self.td_error, var_list=self.ce_params)

        self.a_loss = tf.reduce_mean(q)    # maximize the q
        self.atrain = tf.compat.v1.train.AdamOptimizer(LR_A).minimize(- self.a_loss, var_list=self.ae_params)

        self.sess.run(tf.compat.v1.global_variables_initializer())

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[None, :]})[0]

    def learn(self):
        # soft target replacement
        self.sess.run(self.soft_replace)

        indices = np.random.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        bt = self.memory[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]

        cost_a = self.sess.run(self.a_loss, {self.S: bs})
        self.sess.run(self.atrain, {self.S: bs})
        cost_c = self.sess.run(self.td_error, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

        return cost_a, cost_c

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.pointer % MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.pointer += 1
        if self.pointer > MEMORY_CAPACITY:      # indicator for learning
            self.memory_full = True

    def _build_a(self, x, scope, trainable):

        with tf.compat.v1.variable_scope(scope):
            for i, layer_dim in enumerate(neurons_actor, 1):
                layer_name = 'layer_{}'.format(i)
                x = tf.compat.v1.layers.dense(x, layer_dim, activation=tf.nn.relu, name=layer_name, trainable=trainable)
            a = tf.compat.v1.layers.dense(x, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)  # product 0~1 value
            return tf.multiply(a, self.a_bound, name='scaled_a')  # remap to workspace

    def _build_c(self, s, a, scope, trainable):

        with tf.compat.v1.variable_scope(scope):
            n_l1 = neurons_critic[0]
            w1_s = tf.compat.v1.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.compat.v1.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.compat.v1.get_variable('b1', [1, n_l1], trainable=trainable)
            x = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            neurons = neurons_critic[1:]
            for i, layer_dim in enumerate(neurons, 2):
                layer_name = 'layer_{}'.format(i)
                x = tf.compat.v1.layers.dense(x, layer_dim, activation=tf.nn.relu, name=layer_name, trainable=trainable)
            return tf.compat.v1.layers.dense(x, 1, trainable=trainable)  # Q(s,a)

    def save(self, path, i):
        saver = tf.train.Saver(max_to_keep=0)
        os.makedirs('./model/'+path+'/net/'+str(int(i)))
        saver.save(self.sess, './model/'+path+'/net/'+str(int(i))+'/params', write_meta_graph=False)

    def restore(self, path):
        saver = tf.train.Saver()
        saver.restore(self.sess, './model/'+path[0]+'/net/'+path[1]+'/params')

