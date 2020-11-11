#----------------------------------------
#       this is the RL arm env          #
#         Ya0000000000000000 
#            2020/10/28                 
#----------------------------------------

import numpy as np
import math
import pyglet
import time
import config

class ArmEnv(object):

    viewer = None
    joint1_bound = [-170, 170]
    joint2_bound = [-135, 80]
    joint3_bound = [-70, 104]
    joint4_bound = [-190, 190]
    joint5_bound = [-115, 115]
    joint6_bound = [-360, 360]
    state_dim = 4  # end-effector (x, y), target ( x, y )
    action_dim = 3 #x y z

    def __init__(self):
        #将会初始化动作空间与状态空间，便于强化学习算法在给定的状态空间中搜索合适的动作
        self.distance=0
        
        self.EEF_arm0_robot = np.zeros((3, 1))  # 3*1 ( Xr , Yr , Zr )
        self.EEF_arm1_robot = np.zeros((3, 1))  # 3*1 ( Xr , Yr , Zr )
        self.EEF_arm2_robot = np.zeros((3, 1))  # 3*1 ( Xr , Yr , Zr )
        self.EEF_arm0_camera = np.zeros((3, 1))  # 3*1 ( Xc , Yc , Zc )
        self.EEF_arm1_camera = np.zeros((3, 1))  # 3*1 ( Xc , Yc , Zc )
        self.EEF_arm2_camera = np.zeros((3, 1))  # 3*1 ( Xc , Yc , Zc )
        self.EEF_arm0_image = np.zeros((2, 1))  # 2*1 ( u, v )
        self.EEF_arm1_image = np.zeros((2, 1))  # 2*1 ( u, v )
        self.EEF_arm2_image = np.zeros((2, 1))  # 2*1 ( u, v )


        # Intrisic matix(要修改)
        self.matrix_camera = np.array([[1.5627044545649730 * math.pow(10, 3), 0, 2.8477025480870020 * math.pow(10, 2)],
                                       [0, 1.5637307281700662 * math.pow(10, 3), 2.8292536344105076 * math.pow(10, 2)],
                                       [0, 0, 1]])
        self.pinvmatrix_camera = np.zeros((3, 3))
        self.pinvmatrix_camera = np.linalg.inv(self.matrix_camera)


        # hand-eye Transformation matrix  -- > camera frame to robot base frame 6*6
        self.SbRc = np.zeros((3, 3))
        self.Sbtc = np.zeros((3, 3))
        self.bTc = np.zeros((6, 6))








        pass
    def step(self, action, inf):
        #用于编写智能体与环境交互的逻辑，它接受action的输入，给出下一时刻的状态、当前动作的回报、是否结束当前episode及调试信息

        x = action[0] 
        y = action[1] 
        z = action[2] 

        self.Transformation()


       pass
    def reset(self):
        #用于在每轮开始之前重置智能体的状态
       pass

    def render(self):
        pass

    def sample_action(self):
        return np.random.rand(2)    # two radians


    def update(self):
        pass



if __name__ == '__main__':

