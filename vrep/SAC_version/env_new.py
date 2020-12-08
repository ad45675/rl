"""
yaooooo
this is robot env  code
update time:11/26

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
from robot_vrep import my_robot


class robot_env(object):
    # joint_bound
    degtorad = math.pi / 180
    joint1_bound = [-50 * degtorad, 50 * degtorad]#(-0.87~0.87)
    joint2_bound = [-80 * degtorad, 70 * degtorad]#(-1.430~0)
    joint3_bound = [-60 * degtorad, 60 * degtorad]#(-1.22~1.814)
    joint4_bound = [0 * degtorad, 0 * degtorad]
    joint5_bound = [-90 * degtorad, 3 * degtorad]#(-1.57~0)
    joint6_bound = [-360 * degtorad, 360 * degtorad]
    state_dim = 5
    action_dim = 4

    def __init__(self):
        self.radtodeg = 180 / math.pi  # 弧度轉角度
        self.degtorad = math.pi / 180  # 角度轉弧度
        self.my_robot = my_robot()
        self.my_robot.connection()

    def reset(self):
        # return state containing joint ,EFF ,target ,dis
        # robot to initial pos and random the target

        self.my_robot.stop_sim()
        self.my_robot.start_sim()
        time.sleep(0.1)
        self.my_robot.start_sim()
        # self.my_robot.random_object()
        return self.get_state()

    def get_state(self):
        # state:{軸角度,物體位置,末端點位置,距離,吸嘴狀態(未完成)}
        joint_pos = self.my_robot.get_joint_pos()  # dim=6
        # print(joint_pos*self.radtodeg)

        joint_state = np.hstack((joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[4])) #dim4

        cubid_pos = self.my_robot.get_cuboid_pos()  # dim=3
        # print('cubid',cubid_pos)

        EEF_pos = self.my_robot.get_EEF_pos()  # dim=3
        # print('EEF',EEF_pos)
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        self.distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))
        # dis_norm=distance/0.66235257
        # print('dis',distance)

        # s=np.hstack([np.ravel(joint_pos),np.ravel(cubid_pos),np.ravel(EEF_pos),distance])
        s = np.hstack((joint_state, self.distance))
        # print('s',s)
        # 吸嘴狀態還沒加上去
        return s

    def step(self, action):
        # action為6個joint.
        # Move the robot arm according to the action.
        # 要回傳下一刻狀態跟reward根是否結束
        # action[0] = np.clip(action[0], *self.joint1_bound)
        # action[1] = -np.clip(action[1], *self.joint2_bound)
        # action[2] = np.clip(action[2], *self.joint3_bound)
        # action[3] = np.clip(action[3], *self.joint4_bound)
        # action[4] = np.clip(action[4], *self.joint5_bound)
        # action[5] = np.clip(action[5], *self.joint6_bound)

        # print('action1', action)
        action[0] = np.clip(action[0], *self.joint1_bound)
        action[1] = np.clip(action[1], *self.joint2_bound)
        action[2] = np.clip(action[2], *self.joint3_bound)
        action[3] = np.clip(action[3], *self.joint5_bound)

        # print('action_bound', action)

        tolerance = 0.005
        done = False
        reward = 0
        # suction_flag=False#是否吸取物體


        # self.my_robot.move_all_joint(action)
        self.my_robot.move_4_joint(action)
        time.sleep(0.5)

        EEF_pos = self.my_robot.get_EEF_pos()
        cubid_pos = self.my_robot.get_cuboid_pos()
        diffence = [(cubid_pos[0] - EEF_pos[0]), (cubid_pos[1] - EEF_pos[1]), (cubid_pos[2] - EEF_pos[2])]
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))

        joint_pos = self.my_robot.get_joint_pos()
        # print('joint_pos',joint_pos)
        joint_state = np.hstack((joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[4]))   # dim4

        reward = -distance

        if self.distance<distance:
            reward-=0.1
        else:
            reward+0.1

        self.distance=distance

        if self.distance <= tolerance:
            self.my_robot.enable_suction(True)
            if cubid_pos[2]>1:
                reward += 1
                done = True
        else:
            reward -= 0.1
            done = False




        s_ = np.hstack((joint_state, distance))
        return s_, reward, done

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
