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

        self.joint = [0, 0, 0, 0, 0, 0]
        # if render:
        self.my_robot.move_all_joint(self.joint)
        print('reset')

        # 目標物隨機擺放

        self.my_robot.random_object()
        return self.get_state()

    def get_state(self):
        ## state 是 6 個joint 和末端點與物體距離


        #  順向運動學得到末端點資訊
        Info, EulerAngle_vrep, EulerAngle, EEF_pos = FK.ForwardKinemetics(self.joint, DH_table)
        EEF_pos = np.round(EEF_pos, 4)

        # 從 vrep 得到末端點資訊
        # if render:
        #     vrep_EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        #     vrep_EEF_pos = np.round(vrep_EEF_pos , 4)
        #     # joint_pos = self.my_robot.get_joint_pos()  # dim=6


        # 從 vrep 得到目標位置
        self.cubid_pos = self.my_robot.get_cuboid_pos()  # dim=3


        # 末端點 與 目標物距離
        diffence = [(self.cubid_pos[0] - EEF_pos[0]), (self.cubid_pos[1] - EEF_pos[1]), (self.cubid_pos[2] - EEF_pos[2])]
        self.distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))

        # xy_diffence = [(self.cubid_pos[0] - EEF_pos[0]), (self.cubid_pos[1] - EEF_pos[1])]
        # self.xy_distance = np.sqrt(pow(xy_diffence[0], 2) + pow(xy_diffence[1], 2))
        # self.z_distance = abs((self.cubid_pos[2] - EEF_pos[2]))

        s = np.hstack((self.joint, self.cubid_pos, self.distance ))

        return s


    def step(self, action):
        #action 是4個joint的位移
        joint_pos_out = np.zeros((6,),np.float)

        done = False
        outbound = False
        reward = 0

        # if render:
        #     joint_pos = self.my_robot.get_joint_pos()  # dim=6
        #     self.joint = joint_pos

        time.sleep(0.2)
        self.joint_cmd[0] = self.joint[0] + action[0]
        self.joint_cmd[1] = self.joint[1] + action[1]
        self.joint_cmd[2] = self.joint[2] + action[2]
        self.joint_cmd[3] = self.joint[3]
        self.joint_cmd[4] = self.joint[4] + action[3]
        self.joint_cmd[5] = self.joint[5]



        outbound = self.check_bound(self.joint_cmd,outbound) #------joint space bound


        if not outbound:
            # self.joint = self.joint_cmd

            while (abs(math.pow(sum((joint_pos_out-self.joint_cmd)*(joint_pos_out-self.joint_cmd)),0.5))>0.005):
                joint_pos_out, vs_out,ts = controller(self.joint_cmd, self.joint, self.vs)
                self.joint = joint_pos_out
                self.vs = vs_out

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
        else:
            while (abs(math.pow(sum((joint_pos_out - self.joint_cmd) * (joint_pos_out - self.joint_cmd)), 0.5)) > 0.005):
                joint_pos_out, vs_out,ts = controller(self.joint, self.joint, self.vs)
                self.joint = joint_pos_out
                self.vs = vs_out
            self.my_robot.move_all_joint(self.joint)

            reward-=1


        #################### record data #####################
        # ts_record = np.reshape(ts, (1, 6))
        # # print(joint_out_record)
        # path = './Trajectory/'
        # name = 'ts_record.txt'
        # f = open(path + name, mode='a')
        # np.savetxt(f, ts_record, fmt='%f')
        # f.close()
            #################### record data #####################
        Info, EulerAngle_vrep, EulerAngle, EEF_pos = FK.ForwardKinemetics(self.joint , DH_table)

        # if render:
        #     vrep_EEF_pos = self.my_robot.get_EEF_pos()  # dim=3

        c_out = self.check_c_space_bound(EEF_pos)       #------卡式空間限制
        if c_out:
            reward=reward-1
            # print('c_out')


        diffence = [(self.cubid_pos[0] - EEF_pos[0]), (self.cubid_pos[1] - EEF_pos[1]), (self.cubid_pos[2] - EEF_pos[2])]
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))

        # cubid_pos = self.my_robot.get_cuboid_pos()
        # xy_diffence = [(self.cubid_pos[0] - EEF_pos[0]), (self.cubid_pos[1] - EEF_pos[1])]
        # xy_distance = np.sqrt(pow(xy_diffence[0], 2) + pow(xy_diffence[1], 2))
        # z_distance = abs((self.cubid_pos[2] - EEF_pos[2]))


      ######-----------reward function-----------######

        if self.distance < distance:
            reward = reward-distance-0.1
        else:
            reward = reward-distance+0.1


        # self.xy_distance = xy_distance
        # self.z_distance = z_distance
        self.distance=distance

        if (self.cubid_pos[0]-0.05 < EEF_pos[0] and EEF_pos[0]<self.cubid_pos[0]+0.05 and self.cubid_pos[1]-0.05 < EEF_pos[1] and EEF_pos[1]<self.cubid_pos[1]+0.05 and self.cubid_pos[2]< EEF_pos[2] and EEF_pos[2]<self.cubid_pos[2]+0.04+0.004):
            print('move')
            self.my_robot.move_all_joint(self.joint)
            time.sleep(0.2)
            suction_value = self.my_robot.enable_suction(True)
            reward = reward + 0.5
            print('suction enable',suction_value)
            if suction_value == 1:
                self.joint_cmd[0] = self.joint[0]
                self.joint_cmd[1] = -40*self.degtorad
                self.joint_cmd[2] = self.joint[2]
                self.joint_cmd[3] = self.joint[3]
                self.joint_cmd[4] = self.joint[4]
                self.joint_cmd[5] = self.joint[5]
                self.joint = self.joint_cmd
                joint_pos_out, vs_out,ts = controller(self.joint_cmd, self.joint, self.vs)
                self.joint = joint_pos_out
                self.vs = vs_out
                # if render:
                #     # joint = self.joint.reshape(1, 6)
                print("move")
                self.my_robot.move_all_joint(self.joint)


            time.sleep(0.5)
            cubid_pos_now = self.my_robot.get_cuboid_pos()


            if cubid_pos_now[2] - self.cubid_pos[2] > 0.1:
                reward += 5
                done = True
            else:
                reward -= 1
                done = False
        suction_value = self.my_robot.enable_suction(False)

        s_ = self.get_state()
        # s_ = np.hstack((self.joint, self.cubid_pos, self.distance ))


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
