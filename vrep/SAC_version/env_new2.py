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
    joint1_bound = [-50 * degtorad, 50 * degtorad]#(-0.87~0.87)
    joint2_bound = [-80 * degtorad, 70 * degtorad]#(-1.430~1.22)
    joint3_bound = [-60 * degtorad, 60 * degtorad]#(-1.22~1.04)
    joint4_bound = [0 * degtorad, 0 * degtorad]
    joint5_bound = [-90 * degtorad, 3 * degtorad]#(-1.57~0)
    joint6_bound = [-360 * degtorad, 360 * degtorad]
    state_dim = config.state_dim
    action_dim = config.action_dim

    def __init__(self):
        self.radtodeg = 180 / math.pi  # 弧度轉角度
        self.degtorad = math.pi / 180  # 角度轉弧度
        self.my_robot = my_robot()
        self.my_robot.connection()
        self.joint = np.zeros((6,1),np.float)

    def initial(self):
        self.my_robot.stop_sim()
        self.my_robot.start_sim()

    def reset(self,initial_joint,render):
        # return state containing joint ,EFF ,target ,dis
        # robot to initial pos and random the target
        self.repeat = 0
        self.joint=initial_joint
        self.vs = np.zeros((6,1),np.float)

        if render:
            self.my_robot.move_all_joint(self.joint)
        print('reset')

        # 目標物隨機擺放

        self.my_robot.random_object()
        return self.get_state(render)

    def get_state(self,render):

        #  順向運動學得到末端點資訊
        Info, EulerAngle_vrep, EulerAngle, EEF_pos = FK.ForwardKinemetics(self.joint, DH_table)


        # 從 vrep 得到末端點資訊
        if render:
            EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
            EEF_pos = np.round(EEF_pos , 4)

        # 從 vrep 得到目標位置
        cubid_pos = self.my_robot.get_cuboid_pos()  # dim=3


        # 末端點 與 目標物距離
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        self.distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))

        s = np.hstack((EEF_pos,  cubid_pos, self.distance ))

        return s


    def step(self, action,render):
        joint_pos_cmd = np.zeros((6,), np.float)
        done = False
        outbound=False
        reward = 0

        if render:
            joint_pos = self.my_robot.get_joint_pos()  # dim=6
            self.joint = joint_pos

        time.sleep(0.2)
        joint_pos_cmd[0] = self.joint[0] + action[0]
        joint_pos_cmd[1] = self.joint[1] + action[1]
        joint_pos_cmd[2] = self.joint[2] + action[2]
        joint_pos_cmd[3] = self.joint[3]
        joint_pos_cmd[4] = self.joint[4] + action[3]
        joint_pos_cmd[5] = self.joint[5]


        outbound = self.check_bound(joint_pos_cmd,outbound) #------joint space bound


        if not outbound:
            joint_pos_out, vs_out = controller(joint_pos_cmd, self.joint, self.vs)
            print('joint_pos_out',joint_pos_out)
            self.joint = joint_pos_out
            self.vs = vs_out
            if render:
                # joint = self.joint.tolist
                joint = self.joint.reshape(1,6)
                print('joint----', joint.shape)
                self.my_robot.move_all_joint(joint  )
        else:
            joint_pos_out, vs_out = controller(self.joint, self.joint, self.vs)
            self.joint = joint_pos_out
            self.vs = vs_out

            if render:
                joint = self.joint.tolist
                self.my_robot.move_all_joint(joint)

            self.repeat +=1
            reward-=1


        Info, EulerAngle_vrep, EulerAngle, EEF_pos = FK.ForwardKinemetics(self.joint , DH_table)
        if render:
            EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        c_out = self.check_c_space_bound(EEF_pos)       #------卡式空間限制
        if c_out:
            reward=reward-1


        cubid_pos = self.my_robot.get_cuboid_pos()
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))



        if self.distance<distance:
            reward = -distance-0.1
        else:
            reward = -distance+0.1

        self.distance = distance



        if (cubid_pos[0]-0.05 < EEF_pos[0] <cubid_pos[0]+0.05 and cubid_pos[1]-0.05 < EEF_pos[1] <cubid_pos[1]+0.05 and cubid_pos[2]< EEF_pos[2] <cubid_pos[2]+0.005):
            suction_value = self.my_robot.enable_suction(True)

            if suction_value ==1:
                print('suction enable')
                joint_pos_cmd[0] = self.joint[0]
                joint_pos_cmd[1] = -50*self.degtorad
                joint_pos_cmd[2] = self.joint[2]
                joint_pos_cmd[3] = self.joint[3]
                joint_pos_cmd[4] = self.joint[4]
                joint_pos_cmd[5] = self.joint[5]
                joint_pos_out, vs_out = controller(joint_pos_cmd, self.joint, self.vs)
                self.joint = joint_pos_out
                self.vs = vs_out
                if render:
                    joint = self.joint.tolist
                    self.my_robot.move_all_joint(joint)


            time.sleep(0.5)
            cubid_pos_now = self.my_robot.get_cuboid_pos()


            if cubid_pos_now[2] - cubid_pos[2] > 0.1:
                reward += 10
                done = True
            else:
                reward -= 1
                done = False
        suction_value = self.my_robot.enable_suction(False)

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
        if EEF[1]<-0.324 or EEF[1]>0.374:
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
    env = robot_env()
    env.reset()
    # time.sleep(10)
    # while True:
    time.sleep(5)
    action = np.array([0, 0, 0, -0.5], dtype=np.float32)
    env.my_robot.move_4_joint(action)
    # env.step(env.sample_action())
    print(action *env.radtodeg)
    time.sleep(10)
