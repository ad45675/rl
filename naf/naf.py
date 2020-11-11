import tensorflow as tf
import tensorflow.keras as keras
from tensorflow.keras.layers import Dense
import numpy as np
import os


# naf code with tensorflow.keras


class NAF_net(keras.Model):
    def __init__(self,n_actions,fc1_dim,fc2_dim,name='critic',chkpt_dir='naf'):
        super(NAF_net,self).__init__()
        self.fc1_dim=fc1_dim
        self.fc2_dim=fc2_dim
        self.n_actions=n_actions

        self.model_name=name
        self.checlpoint_dir=chkpt_dir
        self.checkfile=os.path.joint(self.checkpoint_dir,self.model_name+'naf.h5')

        self.fc1=Dense(self.fc1_dim, activation='relu')
        self.fc2=Dense(self.fc2_dim, activation='relu')

        self.mu=Dense(self.n_actions,activation='tanh')  
        self.v=Dense(1,activation=None)
        self.L=Dense(int(self.n_actions*(self.n_actions+1)/2),activation=None)


    def call(self, state, action):
            
        x=self.fc1(state)
        x=self.fc2(x)
        value=self.v(x)
        mu=self.mu(x)
        L=self.L(x)

        

 


