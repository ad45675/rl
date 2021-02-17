"""
yaooooo
this is vrep connect code
update time:11/25

"""

import numpy as np
import math
import sim as vrep
import time
import config
from Rot2RPY import Rot2RPY,euler2mat,Rot2RPY_version2
import cv2 as cv
import PIL.Image as Image
import os
# 配置關節資訊
jointNum = 6
baseName = 'my_robot'
jointName = 'joint'

# V-REP data transmission modes:
WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot  # 非阻塞式,只想給vrep發送指令
STREAMING = vrep.simx_opmode_streaming  # 數據流模式
BUFFER = vrep.simx_opmode_buffer  # 非阻塞式
BLOCKING = vrep.simx_opmode_blocking  # 阻塞式必須等待從vrep返回信息


class my_robot(object):
    def __init__(self):
        self.radtodeg = 180 / math.pi  # 弧度轉角度
        self.degtorad = math.pi / 180  # 角度轉弧度
        self.time_step = 0.5  # sampling time
        self.joint_pos = np.zeros((jointNum,), np.float)
        self.joint_handle = np.zeros((jointNum,), np.int)
        self.plane = 0
        self.work_space = []
        self.EEF = []
        # self.initial_joint = [0.0, 0.524, -0.349, 0, -0.785, 0]  # 這是弧度,vrep裡是角度
        self.initial_joint2 = [0, 0, 0.2, 0, 0, 0]  # my_robot_initial
        self.initial_joint = [0.0, 0, 0, 0, 0, 0]  # my_robot_initial
        self.suction = "suctionPad_active"
        self.object_height = 0
        self.objects = None
        self.object_pos = None
        self.Cuboid_pos = []
        self.action_bound = [1, -1]
#------------------------camera info------------------------#
        self.radtodeg = 180 / math.pi  # 弧度轉角度
        self.degtorad = math.pi / 180  # 角度轉弧度
        self.width = config.width
        self.height = config.height
        self.theta = config.theta
        self.dis_far = config.dis_far
        self.dis_near = config.dis_near
        self.depth_scale = config.depth_scale
        self.save_image_path = config.SAVE_IMAGE_PATH

    def connection(self):

        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
        # clientID stores the ID assingned by coppeliaSim, if the connection failed it will be assigned -1
        if self.clientID != -1:
            print('Connected to remote API server')
        else:
            time.sleep(0.2)
            print('fail to connect!!')
        self.show_msg('Python: Hello')
        time.sleep(0.5)

        # -------Setup the simulation
        vrep.simxSetFloatingParameter(self.clientID,
                                      vrep.sim_floatparam_simulation_time_step,
                                      self.time_step,  # specify a simulation time step
                                      ONESHOT)

        #vrep.simxSynchronous(self.clientID, True)  # if we need to be syncronous

    def disconnect(self):
        self.show_msg('Python: byebye')
        time.sleep(0.2)
        self.stop_sim()

    def stop_sim(self):
        vrep.simxStopSimulation(self.clientID, ONESHOT)

    def start_sim(self):
        # 讀取 robot id
        self.read_object_id()

        # 開始模擬
        vrep.simxStartSimulation(self.clientID, ONESHOT)

        #while vrep.simxGetConnectionId(self.clientID) != -1:
        # 讓模擬先走一步
        vrep.simxSynchronousTrigger(self.clientID)
        # 暫停溝通等等一次發送
        vrep.simxPauseCommunication(self.clientID, True)

        for i in range(jointNum):
            vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[i], self.initial_joint[i], ONESHOT)

        vrep.simxPauseCommunication(self.clientID, False)

        # self.random_object()
        #vrep.simxSynchronousTrigger(self.clientID)  # 进行下一步
        vrep.simxGetPingTime(self.clientID)  # 使得该仿真步走完

        self.suction_enable = False

    def random_object(self):
        # 得到工作空間範圍(plame大小)

        err, x_max = vrep.simxGetObjectFloatParameter(self.clientID, self.plane, vrep.sim_objfloatparam_modelbbox_max_x,
                                                      BLOCKING)
        err, x_min = vrep.simxGetObjectFloatParameter(self.clientID, self.plane, vrep.sim_objfloatparam_modelbbox_min_x,
                                                      BLOCKING)
        err, y_max = vrep.simxGetObjectFloatParameter(self.clientID, self.plane, vrep.sim_objfloatparam_modelbbox_max_y,
                                                      BLOCKING)
        err, y_min = vrep.simxGetObjectFloatParameter(self.clientID, self.plane, vrep.sim_objfloatparam_modelbbox_min_y,
                                                      BLOCKING)

        # 得到plane位置
        err, plane_pos = vrep.simxGetObjectPosition(self.clientID, self.plane, -1, BLOCKING)
        POS_MIN, POS_MAX = [plane_pos[0] + x_min, plane_pos[1] + y_min,0], [plane_pos[0] + x_max, plane_pos[1] + y_max,0]
        # print('min', POS_MIN, 'MAX', POS_MAX)
        pos = list(np.random.uniform(POS_MIN, POS_MAX))

        pos[2]=0.049872

        # 物體隨機擺放


        #for eval
        # pos_eval=[ 0.483295, 0.009622, 0.049872]
        vrep.simxSetObjectPosition(self.clientID, self.Cuboid, -1, pos, ONESHOT)

    def get_cuboid_pos(self):
        self.Cuboid_pos = self.get_position(self.Cuboid)  # 讀物體位置
        # self.Cuboid_pos[2] += self.get_object_height(self.Cuboid)  # 得到物體表面位置
        return (self.Cuboid_pos)

    def get_EEF_pos(self):
        _, self.EEF_pos = vrep.simxGetObjectPosition(self.clientID, self.EEF, -1, BLOCKING)
        return self.EEF_pos

    def get_joint_pos(self):

        for i in range(jointNum):
            err, self.joint_pos[i] = vrep.simxGetJointPosition(self.clientID, self.joint_handle[i], BLOCKING)

        # print(self.joint_pos*self.radtodeg)

        return self.joint_pos

    def simxSetJointTargetPosition(self, new_joint_pos):
        for i in range(jointNum):
            vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[i], new_joint_pos, STREAMING)

    def get_object_height(self, handle):
        # 得到物體高度
        time.sleep(0.2)
        err, minval = vrep.simxGetObjectFloatParameter(self.clientID, handle, vrep.sim_objfloatparam_modelbbox_min_z,
                                                       BLOCKING)

        err, maxval = vrep.simxGetObjectFloatParameter(self.clientID, handle, vrep.sim_objfloatparam_modelbbox_max_z,
                                                       BLOCKING)

        return (maxval - minval) / 2

    def get_position(self, handle):
        # 得到物體位置3D
        err, pos = vrep.simxGetObjectPosition(self.clientID, handle, -1, BLOCKING)
        return pos

    def orientation(self, handle):
        # 得到物體位置3D
        err, euler_angles = vrep.simxGetObjectOrientation(self.clientID, handle, -1,BLOCKING)
        return euler_angles

    def EEF_ori(self):

        euler_angles=self.orientation(self.EEF)

        return euler_angles

    def show_msg(self, message):
        """ send a message for printing in V-REP """
        vrep.simxAddStatusbarMessage(self.clientID, message, WAIT)
        return

    def read_object_id(self):

        # 讀robot base id
        _,self.my_robot=vrep.simxGetObjectHandle(self.clientID, my_robot, BLOCKING)

        # 拿取joint id
        for i in range(jointNum):
            _, self.joint_handle[i] = vrep.simxGetObjectHandle(self.clientID, jointName + str(i + 1), BLOCKING)

        # 第一次讀取joint一定要streaming
        for i in range(jointNum):
            _, joint_pos = vrep.simxGetJointPosition(self.clientID, self.joint_handle[i], STREAMING)

        # 讀取 末端點 id
        _, self.EEF = vrep.simxGetObjectHandle(self.clientID, 'tip', BLOCKING)

        # 讀 cuboid id
        _, self.Cuboid = vrep.simxGetObjectHandle(self.clientID, 'Cuboid', BLOCKING)

        # 讀suction id
        res, self.suctionPad = vrep.simxGetObjectHandle(self.clientID, 'suctionPad', BLOCKING)

        # 讀plane id
        res, self.plane = vrep.simxGetObjectHandle(self.clientID, 'Plane', BLOCKING)

        # 讀camera id
        _, self.camera_handle = vrep.simxGetObjectHandle(self.clientID, 'kinect', BLOCKING)
        _, self.kinectRGB_handle = vrep.simxGetObjectHandle(self.clientID, 'kinect_rgb', BLOCKING)
        _, self.kinectDepth_handle = vrep.simxGetObjectHandle(self.clientID, 'kinect_depth', BLOCKING)


        print('handle available!!!')

    def move_all_joint(self, joint_angle):

        vrep.simxPauseCommunication(self.clientID, True)
        for i in range(jointNum):
            vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[i], joint_angle[i], ONESHOT)
        vrep.simxPauseCommunication(self.clientID, False)

    def move_4_joint(self, joint_angle):
        #MOVE JOINT 1,2,3,5
        vrep.simxPauseCommunication(self.clientID, True)

        vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[0], joint_angle[0], ONESHOT)
        vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[1], joint_angle[1], ONESHOT)
        vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[2], joint_angle[2], ONESHOT)
        vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[4], joint_angle[3], ONESHOT)

        vrep.simxPauseCommunication(self.clientID, False)

    def one_joint(self, i,joint_angle):
        # MOVE ONE JOINT
        vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[i], joint_angle, ONESHOT)



    def enable_suction(self, active):
        if active:
            vrep.simxSetIntegerSignal(self.clientID, self.suction, 1, ONESHOT)
            _,value = vrep.simxGetIntegerSignal(self.clientID, self.suction, BLOCKING)
        else:
            vrep.simxSetIntegerSignal(self.clientID, self.suction, 0, ONESHOT)
            _,value = vrep.simxGetIntegerSignal(self.clientID, self.suction, BLOCKING)

        return value


    def test_env(self):
        self.connection()
        self.start_sim()
        # self.get_joint_pos()
        lastCmdTime = vrep.simxGetLastCmdTime(self.clientID)  # 記錄當前時間
        vrep.simxSynchronousTrigger(self.clientID)  # 讓仿真走一步
        while vrep.simxGetConnectionId(self.clientID) != -1:
            currCmdTime = vrep.simxGetLastCmdTime(self.clientID)  # 記錄當前時間
            # dt = currCmdTime - lastCmdTime # 記錄時間間隔，用於控制
            # robot.get_joint_pos()
            vrep.simxPauseCommunication(self.clientID, True)

            # self.simxSetJointTargetPosition(120/self.radtodeg)

            for i in range(jointNum):
                vrep.simxSetJointTargetPosition(self.clientID, self.joint_handle[i], 30 / self.radtodeg, STREAMING)
            vrep.simxPauseCommunication(self.clientID, False)

            lastCmdTime = currCmdTime  # 記錄當前時間
            vrep.simxSynchronousTrigger(self.clientID)  # 進行下一步
            vrep.simxGetPingTime(self.clientID)  # 使得該仿真步走完

    def set_object_pos(self, target_pos):
        object=self.suctionPad
        vrep.simxSetObjectPosition(self.clientID, object, -1, target_pos, ONESHOT)


    def reset(self):
        self.stop_sim()
        self.start_sim()




#----------------------camera info-----------------------#

    def set_up_camera(self,camera_handle):
        # ----------------------------get camera pose
        _, cam_position = vrep.simxGetObjectPosition(self.clientID, camera_handle, -1, vrep.simx_opmode_blocking)
        _, cam_orientation = vrep.simxGetObjectOrientation(self.clientID, camera_handle, -1, vrep.simx_opmode_blocking)

        cam_trans = np.eye(4, 4)
        cam_trans[0:3, 3] = np.asarray(cam_position)
        cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
        cam_rotm = np.eye(4, 4)

        cam_rotm[0:3, 0:3] = euler2mat(cam_orientation[0], cam_orientation[1], cam_orientation[2])  # 逆矩陣
        cam_pose = np.dot(cam_trans, cam_rotm)
        return cam_position, cam_rotm, cam_pose
    def get_depth_camera_pose(self):
        return self.set_up_camera(self.kinectDepth_handle)

    def intri_camera(self):
        # ----------------------------get camera 內參
        fx = -self.width / 2.0 / (math.tan(self.theta * self.degtorad / 2.0))
        fy = -fx
        u0 = self.width / 2
        v0 = self.height / 2
        intri = np.array([
            [fx, 0, u0],
            [0, fy, v0],
            [0, 0, 1]])

        return intri
    def get_camera_data(self):
        # 從VREP得到圖片資訊
        # ---------------------------彩色圖片
        res, resolution, raw_image = vrep.simxGetVisionSensorImage(self.clientID, self.kinectRGB_handle, 0,BLOCKING)
        color_img = np.array(raw_image, dtype=np.uint8)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float) / 255
        color_img[color_img < 0] += 1  # 這甚麼??
        color_img *= 255
        color_img = np.flipud(color_img)  # 翻轉列表

        color_img = color_img.astype(np.uint8)  # np.uint8[0,255]  如果是float 就是灰階圖片

        # ---------------------------深度圖片
        res, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.kinectDepth_handle,BLOCKING)
        depth_img = np.array(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.flipud(depth_img)  # 翻轉列表
        depth_img[depth_img < 0] = 0
        depth_img[depth_img > 1] = 0.9999

        depth_img = (self.dis_far * self.dis_near / (self.dis_far - (self.dis_far - self.dis_near))) * depth_img  # 0.01124954

        depth_img_for_show = (depth_img - np.min(depth_img)) * 255 / (np.max(depth_img) - np.min(depth_img))  # 正規化 0~255

        depth_img_for_show = depth_img_for_show.astype(np.uint8)

        depth_img_for_show = cv.cvtColor(depth_img_for_show, cv.COLOR_GRAY2BGR)

        return color_img, depth_img,depth_img_for_show

    def save_image_and_show(self,cur_color,cur_depth,depth_img_for_show,img_idx):
        ## 存影像圖片  將原本array轉成image
        img = Image.fromarray(cur_color.astype(np.uint8), mode='RGB')  # .convert('RGB')  #array到image的實現轉換
        img_path = os.path.join(self.SAVE_PATH_COLOR, str(img_idx) + '_rgb.png')
        img.save(img_path)
        ##   存深度圖
        depth_img = Image.fromarray(cur_depth.astype(np.uint8), mode='I')  # array到image的實現轉換
        depth_path = os.path.join(self.SAVE_PATH_COLOR, str(img_idx) + '_depth.png')
        depth_img.save(depth_path)
        ##   存深度圖(3channel)
        depth_img_for_show = Image.fromarray(cur_depth.astype(np.uint8), mode='RGB')  # array到image的實現轉換
        depth_show_path = os.path.join(self.SAVE_PATH_COLOR, str(img_idx) + '_depth_for_show.png')
        depth_img_for_show.save(depth_show_path)

        bg_depth=cv.imread(depth_path,-1)/10000
        depth_img_for_show = cv.imread(depth_img_for_show)
        bg_color=cv.imread(img_path)/255
        cv.imshow('color Image',bg_color)
        cv.imshow('depth 3channel Image', depth_img_for_show )
        cv.imshow('depth Image', bg_depth)
        # 按下任意鍵則關閉所有視窗
        cv.waitKey(0)
        cv.destroyAllWindows()

    def get_depth_from_RGB(self,num=5, resy=config.height, pixel=np.array([255, 177])):
        depth_path = os.path.join(self.SAVE_PATH_COLOR, str(num) + '_depth.png')
        bg_depth = cv.imread(depth_path, 0)
        bg_depth [bg_depth  < 0] = 0
        bg_depth [bg_depth > 1] = 0.9999


        # # -----翻轉照片
        # depth_img_flip = np.zeros([424, 512])
        # for i in range(424):
        #     for j in range(512):
        #         depth_img_flip[i][j] = depth_img[423 - i][j]
        # # -----翻轉照片

        pixel_depth =bg_depth [pixel[1]][pixel[0]]
        # print('de',pixel_depth)
        pixel_depth = self.dis_near + (self.dis_far-self.dis_near)*pixel_depth

        # pixel_depth_img_flip = depth_img_flip[resy - pixel[1]][pixel[0]]
        # pixel_depth_img_flip = dis_near + (dis_far-dis_near)*pixel_depth_img_flip

        return pixel_depth

    def xyz_2_uv(self,x, y, z):
        # ------------xyz to pixel
        u = x * (self.width / 2) * (-1 / math.tan(self.theta * self.deg2rad / 2.0)) * (1 / z) + self.width/ 2
        v = y * (self.width / 2) * (1 / math.tan(self.theta * self.deg2rad / 2.0)) * (1 / z) + self.height / 2

        # call funtion EX: u,v=xyz_2_uv(o_in_cam_vrep[0],o_in_cam_vrep[1],o_in_cam_vrep[2],512,424,70)
        # cam_intri = intri_camera()
        # xyz = np.array([x,y,z])
        # xyz = np.reshape(xyz,(3,1))
        # uv = (1 / z) * np.dot(cam_intri , xyz)  #------>用內參matrix算
        # u,v = uv[0],uv[1]
        v = self.height - v
        return u, v

    def uv_2_xyz(self,z, u, v):
        # ------------pixel to xyz
        v = self.height - v

        x = z * (math.tan(self.theta * self.deg2rad / 2)) * 2 * ((self.width / 2 - u) / self.width)
        y = z * (math.tan(self.theta * self.deg2rad / 2)) * 2 * ((v - self.height / 2) / self.width)
        # call funtion EX: x,y=uv_2_xyz(o_in_cam_vrep[2],256.5,261.999,512,424,70)

        # cam_intri = intri_camera()
        # intri_inver = np.linalg.inv(cam_intri)
        # uv = np.array([u,v,1])
        # xyz = np.dot(intri_inver, uv) * z
        return x, y

if __name__ == '__main__':
    joint_pos =[]
    cuboid=[]
    robot = my_robot()
    robot.connection()
    robot.start_sim()

    cuboid=robot.get_cuboid_pos()

    robot.set_object_pos(cuboid)
    robot.enable_suction(True)
    # action = np.array([0, -0.5, 0, 0], dtype=np.float32)
    # robot.move_4_joint(action)
    #
    # joint_pos=robot.get_joint_pos()
    # joint_pos=joint_pos .astype(np.float)
    # # print(joint_pos)
    # print("joint_pos " ,joint_pos,end="\n")