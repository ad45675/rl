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
    state_dim = 7
    action_dim = 3

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

        self.joint=[0,0,0,0,0,0]
        self.my_robot.move_all_joint(self.joint)
        print('reset')

        #self.my_robot.start_sim()
        self.my_robot.random_object()
        return self.get_state()

    def get_state(self):
        # state:{物體位置,末端點位置}


        Info, EulerAngle_vrep, self.EulerAngle,test_EEF= FK.ForwardKinemetics(self.joint , DH_table)
        test_EEF=np.round(test_EEF,4)

        # record data
        test_EEF_pos_record = np.reshape(test_EEF,(1,3))
        path = './Trajectory/'
        name = 'test_EEF.txt'
        f = open(path+name,mode='a')
        np.savetxt(f,test_EEF_pos_record, fmt='%f')
        f.close()


        EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        EEF_pos = np.round(EEF_pos , 4)

        ##################### record data #####################
        EEF_pos_record = np.reshape(EEF_pos,(1,3))
        path = './Trajectory/'
        name = 'vrep_EEF.txt'
        f = open(path+name,mode='a')
        np.savetxt(f,EEF_pos_record, fmt='%f')
        f.close()
        ##################### record data #####################

        cubid_pos = self.my_robot.get_cuboid_pos()  # dim=3
        # print('cuboid_pos',cubid_pos)


        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        self.distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))

        s = np.hstack((EEF_pos,  cubid_pos, self.distance ))
        # print('s',s)
        # 吸嘴狀態還沒加上去
        return s


    def step(self, action):
        done = False
        outbound=False
        reward = 0



        ##------------vrep 的------------##
        # joint_pos = self.my_robot.get_joint_pos()  # dim=6
        #
        # ##################### record data #####################
        # joint_origin_record = np.reshape(joint_pos,(1,6))
        # path = './Trajectory/'
        # name = 'joint_origin_record.txt'
        # f = open(path+name,mode='a')
        # np.savetxt(f,joint_origin_record , fmt='%f')
        # f.close()
        # ##################### record data #####################
        #
        #
        #
        # time.sleep(0.2)
        # joint_pos[0] = joint_pos[0] + action[0]
        # joint_pos[1] = joint_pos[1] + action[1]
        # joint_pos[2] = joint_pos[2] + action[2]
        # joint_pos[3] = joint_pos[3] +0
        # joint_pos[4] = joint_pos[4] +0
        # joint_pos[5] = joint_pos[5] +0
        #
        # ##################### record data #####################
        # vrep_joint = np.reshape(joint_pos,(1,6))
        # path = './Trajectory/'
        # name = 'vrep_joint.txt'
        # f = open(path+name,mode='a')
        # np.savetxt(f,vrep_joint , fmt='%f')
        # f.close()
        # ##################### record data #####################


        ##------------算 的------------##
        self.joint_cmd[0] = self.joint[0] + action[0]
        self.joint_cmd[1] = self.joint[1] + action[1]
        self.joint_cmd[2] = self.joint[2] + action[2]
        self.joint_cmd[3] = self.joint[3]
        self.joint_cmd[4] = self.joint[4]
        self.joint_cmd[5] = self.joint[5]

        ##################### record data #####################
        # joint_record = np.reshape(self.joint_cmd,(1,6))
        # path = './Trajectory/'
        # name = 'joint_record.txt'
        # f = open(path+name,mode='a')
        # np.savetxt(f,joint_record , fmt='%f')
        # f.close()
        # ##################### record data #####################

        outbound = self.check_bound(self.joint_cmd,outbound)


        if not outbound:
            # self.my_robot.move_all_joint(joint_pos)
            # self.joint = self.joint_cmd

            ##------------算 的------------##
            joint_pos_out, vs_out = controller(self.joint_cmd, self.joint, self.vs)
            error=self.joint_cmd-joint_pos_out
            self.joint = joint_pos_out
            self.vs = vs_out

            ##################### record data #####################
            error_record = np.reshape(error, (1, 6))
            # print(joint_out_record)
            path = './Trajectory/'
            name = 'error_record.txt'
            f = open(path + name, mode='a')
            np.savetxt(f,error_record, fmt='%f')
            f.close()
            ##################### record data #####################



            self.my_robot.move_all_joint(self.joint)
        else:
            reward-=1


        # suction_flag=False#是否吸取物體
        EEF_pos = self.my_robot.get_EEF_pos()  # dim=3

        c_out=self.check_c_space_bound(EEF_pos)

        if c_out:
            reward=reward-1
        cubid_pos = self.my_robot.get_cuboid_pos()
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))
        # print('dis',distance)

        # joint_pos = self.my_robot.get_joint_pos()
        # # print('joint_pos',joint_pos)
        # joint_state = np.hstack((joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[4]))   # dim4

        if self.distance<distance:
            reward=-distance-0.1
        else:
            reward=-distance+0.1

        self.distance=distance
        # print('dis',self.distance)
        if (self.distance < 0.05 ):
            euler_angles = self.my_robot.EEF_ori()
            joint_pos[4] = joint_pos[4] - euler_angles[1]
            self.my_robot.one_joint(4, joint_pos[4])
            time.sleep(1)
        if (self.distance < 0.005):
            time.sleep(0.5)
            self.my_robot.enable_suction(True)
            joint_pos[1]=-30*self.degtorad
            self.my_robot.one_joint(1, joint_pos[1])
            time.sleep(0.5)
            cubid_pos_now = self.my_robot.get_cuboid_pos()

            if cubid_pos_now[2] - cubid_pos[2] > 0.1:
                reward += 1
                done = True
            else:
                reward -= 1
                done = False
        self.my_robot.enable_suction(False)
            # if cubid_pos[2]>0.5:
            #     reward += 1
            #     done = True
            # else:
            #     reward -=1
            #     done = True

        s_ = self.get_state()

        return s_, reward, done

    def check_bound(self,joint_pos,outbound):
        pos_lim=np.array([[-110*degtorad,110*degtorad],
                         [-85*degtorad*0.85,130*degtorad*0.85],
                         [-110*degtorad*0.85,170*degtorad*0.85],
                         [-190*degtorad*0.85,190*degtorad*0.85],
                         [-125*degtorad*0.85,125*degtorad*0.85],
                         [-360*degtorad*0.85,360*degtorad*0.85]])
        # print('lim',pos_lim[2])
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

    def render(self):

        pass


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
