"""
yaooooo
this is robot env  code
update time:11/21

state dim=11
0~3:joint pos
4~6:cuboid pos
7~9:EEF pos
10:dis
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
    joint1_bound = [-170 * degtorad, 170 * degtorad]
    joint2_bound = [-135 * degtorad, 80 * degtorad]
    joint3_bound = [-70 * degtorad, 104 * degtorad]
    joint4_bound = [-190 * degtorad, 190 * degtorad]
    joint5_bound = [-115 * degtorad, 115 * degtorad]
    joint6_bound = [-360 * degtorad, 360 * degtorad]
    state_dim = 11
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
        time.sleep(1)
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
        distance = np.sqrt(pow(diffence[0], 2) + pow(diffence[1], 2) + pow(diffence[2], 2))
        # print('dis',distance)

        # s=np.hstack([np.ravel(joint_pos),np.ravel(cubid_pos),np.ravel(EEF_pos),distance])
        s = np.hstack((joint_state, cubid_pos, EEF_pos, distance))

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

        action[0] = np.clip(action[0], *self.joint1_bound)
        action[1] = -np.clip(action[1], *self.joint2_bound)
        action[2] = np.clip(action[2], *self.joint3_bound)
        action[3] = np.clip(action[3], *self.joint5_bound)

        print('action', action)

        tolerance = 0.05
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
        joint_state = np.hstack((joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[4]))  # dim4

        reward = -distance

        if distance <= tolerance:
            self.my_robot.enable_suction(True)
            reward += 10
            done = True
            if self.my_robot.suction == 1:
                reward += 1
                done = True
            else:
                done = False
        else:
            reward -= 1

        s_ = np.hstack((joint_state, cubid_pos, EEF_pos, distance))
        return s_, reward, done

    def sample_action(self):
        return np.random.rand(4)  # 4 joints

    def render(self):

        pass


if __name__ == '__main__':
    env = robot_env()
    env.reset()
    # time.sleep(10)
    while True:
        time.sleep(0.5)
        env.step(env.sample_action())
        print(env.sample_action()*env.radtodeg)
        time.sleep(10)
