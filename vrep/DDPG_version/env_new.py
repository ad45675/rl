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
import math
import time
import inverseKinematics as IK
import Kinematics as FK
from IK_FindOptSol import FindOptSol
from simulation_robot import simulation_robot as id_robot
from robot_vrep import my_robot

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

    def initial(self):
        self.my_robot.stop_sim()
        self.my_robot.start_sim()

    def reset(self):
        # return state containing joint ,EFF ,target ,dis
        # robot to initial pos and random the target

        #self.my_robot.stop_sim()
        #self.my_robot.start_sim()
        self.my_robot.move_all_joint([0,0,0,0,0,0])
        print('reset')

        #self.my_robot.start_sim()
        self.my_robot.random_object()
        return self.get_state()

    def get_state(self):
        # state:{物體位置,末端點位置}
        # self.joint_pos = self.my_robot.get_joint_pos()  # dim=6
        # self.joint_pos=np.round(self.joint_pos,5)
        # Info, EulerAngle_vrep, self.EulerAngle, Position = FK.ForwardKinemetics(self.joint_pos , DH_table)
        # Position=np.round(Position,4)
        EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        EEF_pos = np.round(EEF_pos , 4)

        cubid_pos = self.my_robot.get_cuboid_pos()  # dim=3


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
        optimalsol_set=[]
        # print('action',action)
        # EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        joint_pos = self.my_robot.get_joint_pos()  # dim=6

        time.sleep(0.2)
        joint_pos[0] = joint_pos[0] + action[0]
        joint_pos[1] = joint_pos[1] + action[1]
        joint_pos[2] = joint_pos[2] + action[2]


        outbound = self.check_bound(joint_pos,outbound)


        if not outbound:
            self.my_robot.move_all_joint(joint_pos)
        else:

            reward-=1

        # suction_flag=False#是否吸取物體
        EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        cubid_pos = self.my_robot.get_cuboid_pos()
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))
        # print('dis',distance)

        # joint_pos = self.my_robot.get_joint_pos()
        # # print('joint_pos',joint_pos)
        # joint_state = np.hstack((joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[4]))   # dim4

        # if self.distance<distance:
        #     reward=-distance-0.1
        # else:
        #     reward=-distance+0.1
        _,reward=self.reward_fun(self.distance,distance)

        self.distance=distance

        if (self.distance < 0.05 ):
            euler_angles = self.my_robot.EEF_ori()
            # print('euler_angles', euler_angles)
            joint_pos[4] = joint_pos[4] - euler_angles[1]
            self.my_robot.one_joint(4, joint_pos[4])
            time.sleep(1)

        # height=EEF_pos[2]-cubid_pos[2]
        # print('self.distance',self.distance)
        if (self.distance<0.01 ):
            # print('hei',abs(EEF_pos[2]-cubid_pos[2]))
            # print('hello')
            time.sleep(0.5)
            self.my_robot.enable_suction(True)
            time.sleep(2)
            joint_pos[1]=-30*self.degtorad
            self.my_robot.one_joint(1, joint_pos[1])
            time.sleep(1)

            cubid_pos_now = self.my_robot.get_cuboid_pos()

            if cubid_pos_now[2]-cubid_pos[2]>0.1:
                reward += 1
                done = True
            else:
                reward -=1
                done=False
        self.my_robot.enable_suction(False)

        s_ = self.get_state()
        # print('re',reward)
        return s_, reward, done

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

    def check_bound(self,joint_pos,outbound):
        pos_lim=np.array([[-170*degtorad*0.85,170*degtorad*0.85],
                         [-85*degtorad*0.85,130*degtorad*0.85],
                         [-170*degtorad*0.85,110*degtorad*0.85],
                         [-190*degtorad*0.85,190*degtorad*0.85],
                         [-125*degtorad*0.85,125*degtorad*0.85],
                         [-360*degtorad*0.85,360*degtorad*0.85]])
        # print('lim',pos_lim[2])
        for i in range(6):
            if pos_lim[i][0]>joint_pos[i] or joint_pos[i]>pos_lim[i][1]:
                outbound=True
        return outbound


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
