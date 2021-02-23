"""
yaooooo
this is robot env  code
update time:12/11

將距離和角度正規化並修改reward和結束條件

state dim=5
0~3:joint pos
# 4~6:cuboid pos
# 7~9:EEF pos
4:dis
-------------------
action dim=4
joint pos
--------------------
joint_bound
joint 1=[-170 170]
joint 2=[-135  80]
joint 3=[-70  104]
joint 4=[-190 190]
joint 5=[-115 115]
joint 6=[-360 360]

"""

import numpy as np
import os
import math
import time
import inverseKinematics as IK
import Kinematics as FK
from IK_FindOptSol import FindOptSol
from simulation_robot import simulation_robot as id_robot
from robot_vrep import my_robot
from controller import controller
import config


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

radtodeg = 180 / math.pi  # 弧度轉角度
degtorad = math.pi / 180  # 角度轉弧度
#這裡單位是 cm  吸嘴加0.068m
DH_table = np.array([[0,            0.345,  0.08,   math.pi / 2],
					 [0+math.pi / 2 , 0,  0.27,     0],
					 [0,             0,     0.09,    math.pi / 2],
					 [0,            0.295,  0,       -math.pi / 2],
					 [0,            0,      0,       math.pi / 2],
					 [0,       0.102+0.068, 0,          0]])


def save_txt(data, fmt='%f'):
    f = open('C:/Users/user/Desktop/rl/data.txt', 'a')
    np.savetxt(f, data, fmt=fmt)
    f.close()

class robot_env(object):
    # joint_bound
    degtorad = math.pi / 180
    state_dim = config.state_dim
    action_dim = config.action_dim
    SamplingTime = 0.01
    def __init__(self):
        self.radtodeg = 180 / math.pi  # 弧度轉角度
        self.degtorad = math.pi / 180  # 角度轉弧度
        self.my_robot = my_robot()
        self.my_robot.connection()
        self.joint_cmd = np.zeros((6,),np.float)
        self.vs = np.zeros((6,),np.float)


    def initial(self):
        self.my_robot.stop_sim()
        self.my_robot.start_sim()

    def reset(self):
        # return state containing joint ,EFF ,target ,dis
        # robot to initial pos and random the target

        self.joint = [0, 0, 0, 0, -1.57, 0]
        self.my_robot.move_all_joint(self.joint)
        print('reset')

        # 目標物隨機擺放
        self.my_robot.random_object()
        self.cubid_pos, self.cuboid_x_range , self.cuboid_y_range  = self.my_robot.get_cuboid_pos()  # dim=3


        return self.get_state()

    def get_state(self):


        # 順向運動學得到末端點資訊
        Info, EulerAngle_vrep, EulerAngle, EEF_pos = FK.ForwardKinemetics(self.joint, DH_table)
        self.EEF_pos = np.round(EEF_pos, 4)

        distance = np.linalg.norm(self.cubid_pos - self.EEF_pos)
        # gripper state
        object_rel_pos = self.cubid_pos - self.EEF_pos

        s = np.concatenate([self.cubid_pos,self.EEF_pos,object_rel_pos])

        return s


    def step(self, action, record):
        #action 是末端點 xyz
        joint_pos_out = np.zeros((6,),np.float)
        joint_cmd = np.zeros((6,), np.float)
        target_height= np.zeros((3,), np.float)


        done = False
        outbound = False
        reward = 0
        success = 0
        coutbound = 0
        cuboid_out = 0


        eef_x = action[0] + self.EEF_pos[0]
        eef_y = action[1] + self.EEF_pos[1]
        eef_z = action[2] + self.EEF_pos[2]

        EEF = np.array([eef_x,eef_y,eef_z])

        if (record):
        #################### record data #####################
            EEF_record = np.reshape(EEF, (1, 3))
            # print(joint_out_record)
            path = './Trajectory/'
            name = 'EEF_record.txt'
            f = open(path + name, mode='a')
            np.savetxt(f, error_record, fmt='%f')
            f.close()
        #################### record data #####################

        if(self.check_c_space_bound(EEF)):
            coutbound = coutbound + 1
        finalpos = [0,0,180]

        [tip_Jangle, flag] = IK.InverseKinematics(finalpos, EEF, DH_table)
        joint = FindOptSol(tip_Jangle, self.joint)
        self.my_robot.move_all_joint(joint)
        self.joint = joint

        distance = np.linalg.norm(self.cubid_pos - EEF)


        if (distance < 0.002) or (abs(EEF[2] - self.cubid_pos[2]) < 0.005) :
            suction_value = self.my_robot.enable_suction(True)
            height = 0.3  # (m)要抬高幾公分
            target_height = np.array([0.4250, 0.025, height])
            [tip_Jangle, flag] = IK.InverseKinematics(finalpos, target_height, DH_table)
            joint = FindOptSol(tip_Jangle, self.joint)
            self.my_robot.move_all_joint(joint)
            self.joint = joint

        cuboid_pos_now, self.cuboid_x_range , self.cuboid_y_range  = self.my_robot.get_cuboid_pos()  # dim=3

        if (abs(cuboid_pos_now[2] - self.cubid_pos[2])>0.02):
           success = 1
        else:
           success = 0
        time.sleep(0.5)  #需要
        suction_value = self.my_robot.enable_suction(False)
        if (self.check_c_space_bound(cuboid_pos_now)):
            cuboid_out = cuboid_out + 1

        self.cubid_pos = cuboid_pos_now
        reward = -distance -0.2*coutbound -0.5*cuboid_out + success

        if(cuboid_out):
            done = True
            print('cuboid_out done')

        if (success):
            done = True
            print('lift done')

        s_ = self.get_state()

        return s_, reward, done





    def check_bound(self,joint_pos,outbound):
        pos_lim=np.array([[-110*degtorad*0.9,110*degtorad*0.9],
                         [-85*degtorad*0.9,130*degtorad*0.9],
                         [-110*degtorad*0.9,170*degtorad*0.9],
                         [-190*degtorad*0.9,190*degtorad*0.9],
                         [-125*degtorad*0.9,125*degtorad*0.9],
                         [-360*degtorad*0.9,360*degtorad*0.9]])

        for i in range(6):
            if pos_lim[i][0]>joint_pos[i] or joint_pos[i]>pos_lim[i][1]:
                outbound=True
        return outbound

    def check_c_space_bound(self,EEF):
        c_out=False
        if EEF[0]<0.25 or EEF[0]>0.6:
            c_out=True
        if EEF[1]<-0.324 or EEF[1]>0.374 or EEF[2] < 0:
            c_out=True
        return c_out
    def sample_action(self):
        return np.random.rand(4)  # 4 joints


    def reward_fun(self,d_t1,d_t2):
        r1=-d_t2
        if (d_t2<0.01):
            r2=5
        elif(0.01<d_t2 and d_t2<0.7):
            r2=0
        elif(d_t2>=0.7):
            r2=-2
        r3=-math.exp(1.2+d_t2)

        if(d_t2<d_t1):
            r4=math.exp(d_t2-d_t1+0.7)
        elif(d_t2>=d_t1):
            r4=0
        R1=4*r1+r2
        R2=r2+r3+r4
        # print('r1',r1,'r2',r2,'r3',r3,'r4',r4,'R1','R2',R2)
        return R1 ,R2


if __name__ == '__main__':
    render=True
    env = robot_env()
    env.initial()
    env.reset(render)
    # time.sleep(10)
    # while True:
    time.sleep(5)
    action = np.array([0.03, 0.03, 0.03, -0.03], dtype=np.float32)
    env.step(action,render)
    # env.step(env.sample_action())
    print(action *env.radtodeg)
    time.sleep(10)

# self.my_robot.move_all_joint(self.joint)
##################### record data #####################
# error_record = np.reshape(error, (1, 6))
# # print(joint_out_record)
# path = './Trajectory/'
# name = 'error_record.txt'
# f = open(path + name, mode='a')
# np.savetxt(f, error_record, fmt='%f')
# f.close()
##################### record data #####################