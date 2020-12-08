# """
# keyboard Instructions:
#     robot moving velocity: <=5(advise)
#     Q,W: joint 0
#     A,S: joint 1
#     Z,X: joint 2
#     E,R: joint 3
#     D,F: joint 4
#     C,V: joint 5
#     P: exit()
#     T: close RG2
#     Y: open RG2
#     L: reset robot
#     SPACE: save image
# """


# import os
# import cv2
# import sys
# import math
# import time
# import random
# import string
# import pygame
# import sim as vrep
# import numpy as np

# class UR5_RG2:
#     resolutionX = 640
#     resolutionY = 480
#     joint_angle = [0,0,0,0,0,0]
#     RAD2DEG = 180 / math.pi
    
#     # Handles information
#     jointNum = 6
#     baseName = 'UR5'
#     rgName = 'RG2'
#     jointName = 'UR5_joint'
#     camera_rgb_Name = 'kinect_rgb'
#     camera_depth_Name = 'kinect_depth'

#      # communication and read the handles
#     def __init__(self):
#         jointNum = self.jointNum
#         baseName = self.baseName
#         rgName = self.rgName
#         jointName = self.jointName
#         camera_rgb_Name = self.camera_rgb_Name
#         camera_depth_Name = self.camera_depth_Name
        
#         print('Simulation started')  
#         vrep.simxFinish(-1)     # 关闭潜在的连接    

#         while True:
#             # simxStart的参数分别为：服务端IP地址(连接本机用127.0.0.1);端口号;是否等待服务端开启;连接丢失时是否尝试再次连接;超时时间(ms);数据传输间隔(越小越快)
#             clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
#             if clientID > -1:
#                 print("Connection success!")
#                 break
#             else:
#                 time.sleep(0.2)
#                 print("Failed connecting to remote API server!")
#                 print("Maybe you forget to run the simulation on vrep...")
#         vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)    # 仿真初始化

#         # 读取Base和Joint的句柄
#         jointHandle = np.zeros((jointNum, 1), dtype=np.int)
#         for i in range(jointNum):
#             _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i+1), vrep.simx_opmode_blocking)
#             jointHandle[i] = returnHandle

#             _, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
#             _, rgHandle = vrep.simxGetObjectHandle(clientID, rgName, vrep.simx_opmode_blocking)
#             _, cameraRGBHandle = vrep.simxGetObjectHandle(clientID, camera_rgb_Name, vrep.simx_opmode_blocking)
#             _, cameraDepthHandle = vrep.simxGetObjectHandle(clientID, camera_depth_Name, vrep.simx_opmode_blocking)

#         # 读取每个关节角度
#         jointConfig = np.zeros((jointNum, 1))
#         for i in range(jointNum):
#              _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_blocking)
#              jointConfig[i] = jpos
             
#         self.clientID = clientID
#         self.jointHandle = jointHandle
#         self.rgHandle = rgHandle
#         self.cameraRGBHandle = cameraRGBHandle
#         self.cameraDepthHandle = cameraDepthHandle
#         self.jointConfig = jointConfig

#     def __del__(self):
#         clientID = self.clientID
#         vrep.simxFinish(clientID)
#         print('Simulation end')                    

#     # show Handles information
#     def showHandles(self):
        
#         RAD2DEG = self.RAD2DEG
#         jointNum = self.jointNum
#         clientID = self.clientID
#         jointHandle = self.jointHandle
#         rgHandle = self.rgHandle
#         cameraRGBHandle = self.cameraRGBHandle
#         cameraDepthHandle = self.cameraDepthHandle
        
#         print('Handles available!')
#         print("==============================================")
#         print("Handles:  ")
#         for i in range(len(jointHandle)):
#             print("jointHandle" + str(i+1) + ": " + jointHandle[i])
#         print("rgHandle:" + rgHandle)
#         print("cameraRGBHandle:" + cameraRGBHandle)
#         print("cameraDepthHandle:" + cameraDepthHandle)
#         print("===============================================")

#     # show each joint's angle
#     def showJointAngles(self):
#         RAD2DEG = self.RAD2DEG
#         jointNum = self.jointNum
#         clientID = self.clientID
#         jointHandle = self.jointHandle
        
#         for i in range(jointNum):
#             _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_blocking)
#             print(round(float(jpos) * RAD2DEG, 2), end = ' ')
#         print('\n')

#     # get RGB images
#     def getImageRGB(self):
#         clientID = self.clientID
#         cameraRGBHandle = self.cameraRGBHandle
#         resolutionX = self.resolutionX
#         resolutionY = self.resolutionY
        
#         res1, resolution1, image_rgb = vrep.simxGetVisionSensorImage(clientID, cameraRGBHandle, 0, vrep.simx_opmode_blocking)

#         image_rgb_r = [image_rgb[i] for i in range(0,len(image_rgb),3)]
#         image_rgb_r = np.array(image_rgb_r)
#         image_rgb_r = image_rgb_r.reshape(resolutionY,resolutionX)
#         image_rgb_r = image_rgb_r.astype(np.uint8)

#         image_rgb_g = [image_rgb[i] for i in range(1,len(image_rgb),3)]
#         image_rgb_g = np.array(image_rgb_g)
#         image_rgb_g = image_rgb_g.reshape(resolutionY,resolutionX)
#         image_rgb_g = image_rgb_g.astype(np.uint8)

#         image_rgb_b = [image_rgb[i] for i in range(2,len(image_rgb),3)]
#         image_rgb_b = np.array(image_rgb_b)
#         image_rgb_b = image_rgb_b.reshape(resolutionY,resolutionX)
#         image_rgb_b = image_rgb_b.astype(np.uint8)

#         result_rgb = cv2.merge([image_rgb_b,image_rgb_g,image_rgb_r])
#         # 镜像翻转, opencv在这里返回的是一张翻转的图
#         result_rgb = cv2.flip(result_rgb, 0)
#         return result_rgb
        
#     # get depth images
#     def getImageDepth(self):
#         clientID = self.clientID
#         cameraDepthHandle = self.cameraDepthHandle
#         resolutionX = self.resolutionX
#         resolutionY = self.resolutionY
        
#         res2, resolution2, image_depth = vrep.simxGetVisionSensorImage(clientID, cameraDepthHandle, 0, vrep.simx_opmode_blocking)

#         image_depth_r = [image_depth[i] for i in range(0,len(image_depth),3)]
#         image_depth_r = np.array(image_depth_r)
#         image_depth_r = image_depth_r.reshape(resolutionY,resolutionX)
#         image_depth_r = image_depth_r.astype(np.uint8)
        
#         image_depth_g = [image_depth[i] for i in range(1,len(image_depth),3)]
#         image_depth_g = np.array(image_depth_g)
#         image_depth_g = image_depth_g.reshape(resolutionY,resolutionX)
#         image_depth_g = image_depth_g.astype(np.uint8)
        
#         image_depth_b = [image_depth[i] for i in range(2,len(image_depth),3)]
#         image_depth_b = np.array(image_depth_b)
#         image_depth_b = image_depth_b.reshape(resolutionY,resolutionX)
#         image_depth_b = image_depth_b.astype(np.uint8)
        
#         result_depth = cv2.merge([image_depth_b,image_depth_g,image_depth_r])
#         # 镜像翻转, opencv在这里返回的是一张翻转的图
#         result_depth = cv2.flip(result_depth, 0)
        
#         # 黑白取反
#         height, width, channels = result_depth.shape
#         for row in range(height):
#             for list in range(width):
#                 for c in range(channels):
#                     pv = result_depth[row, list, c]
#                     result_depth[row, list, c] = 255 - pv
                
#         return result_depth
        
#     # open rg2
#     def openRG2(self):
#         rgName = self.rgName
#         clientID = self.clientID
#         res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, rgName,\
#                                                         vrep.sim_scripttype_childscript,'rg2Open',[],[],[],b'',vrep.simx_opmode_blocking)
        
#     # close rg2
#     def closeRG2(self):
#         rgName = self.rgName
#         clientID = self.clientID
#         res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, rgName,\
#                                                         vrep.sim_scripttype_childscript,'rg2Close',[],[],[],b'',vrep.simx_opmode_blocking)
        
#     # joint_angle是这种形式: [0,0,0,0,0,0], 所有的关节都旋转到对应的角度
#     def rotateAllAngle(self, joint_angle):
#         clientID = self.clientID
#         jointNum = self.jointNum
#         RAD2DEG = self.RAD2DEG
#         jointHandle = self.jointHandle
        
#         # 暂停通信，用于存储所有控制命令一起发送
#         vrep.simxPauseCommunication(clientID, True)
#         for i in range(jointNum):
#             vrep.simxSetJointTargetPosition(clientID, jointHandle[i], joint_angle[i]/RAD2DEG, vrep.simx_opmode_oneshot)
#         vrep.simxPauseCommunication(clientID, False)
        
#         self.jointConfig = joint_angle
        
#     # 将第num个关节正转angle度
#     def rotateCertainAnglePositive(self, num, angle):
#         clientID = self.clientID
#         RAD2DEG = self.RAD2DEG
#         jointHandle = self.jointHandle
#         jointConfig = self.jointConfig
        
#         vrep.simxSetJointTargetPosition(clientID, jointHandle[num], (jointConfig[num]+angle)/RAD2DEG, vrep.simx_opmode_oneshot)
#         jointConfig[num] = jointConfig[num] + angle
        
#         self.jointConfig = jointConfig
        
#     # 将第num个关节反转angle度
#     def rotateCertainAngleNegative(self, num, angle):
#         clientID = self.clientID
#         RAD2DEG = self.RAD2DEG
#         jointHandle = self.jointHandle
#         jointConfig = self.jointConfig
        
#         vrep.simxSetJointTargetPosition(clientID, jointHandle[num], (jointConfig[num]-angle)/RAD2DEG, vrep.simx_opmode_oneshot)
#         jointConfig[num] = jointConfig[num] - angle
        
#         self.jointConfig = jointConfig
        
#     # convert array from vrep to image
#     def arrayToImage(self):
#         path = "imgTemp\\frame.jpg"
#         if os.path.exists(path):
#             os.remove(path)
#         ig = self.getImageRGB()
#         cv2.imwrite(path, ig)
    
#     # convert array from vrep to depth image
#     def arrayToDepthImage(self):
#         path = "imgTempDep\\frame.jpg"
#         if os.path.exists(path):
#             os.remove(path)
#         ig = self.getImageDepth()
#         cv2.imwrite(path, ig)
        
# # control robot by keyboard
# def main():
#     robot = UR5_RG2()
#     resolutionX = robot.resolutionX
#     resolutionY = robot.resolutionY
    
#     #angle = float(eval(input("please input velocity: ")))
#     angle = 1
    
#     pygame.init()
#     screen = pygame.display.set_mode((resolutionX, resolutionY))
#     screen.fill((255,255,255))
#     pygame.display.set_caption("Vrep yolov3 ddpg pytorch")
#     # 循环事件，按住一个键可以持续移动
#     pygame.key.set_repeat(200,50)
    
#     while True:
#         robot.arrayToImage()
#         ig = pygame.image.load("imgTemp\\frame.jpg")
#         #robot.arrayToDepthImage()
#         #ig = pygame.image.load("imgTempDep\\frame.jpg")
#         screen.blit(ig, (0, 0))
#         pygame.display.update()
        
#         key_pressed = pygame.key.get_pressed()
#         for event in pygame.event.get():
#             # 关闭程序
#             if event.type == pygame.QUIT:
#                 sys.exit()
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_p:
#                     sys.exit()
#                 # joinit 0
#                 elif event.key == pygame.K_q:
#                     robot.rotateCertainAnglePositive(0, angle)
#                 elif event.key == pygame.K_w:
#                     robot.rotateCertainAngleNegative(0, angle)
#                 # joinit 1
#                 elif event.key == pygame.K_a:
#                     robot.rotateCertainAnglePositive(1, angle)
#                 elif event.key == pygame.K_s:
#                     robot.rotateCertainAngleNegative(1, angle)
#                 # joinit 2
#                 elif event.key == pygame.K_z:
#                     robot.rotateCertainAnglePositive(2, angle)
#                 elif event.key == pygame.K_x:
#                     robot.rotateCertainAngleNegative(2, angle)
#                 # joinit 3
#                 elif event.key == pygame.K_e:
#                     robot.rotateCertainAnglePositive(3, angle)
#                 elif event.key == pygame.K_r:
#                     robot.rotateCertainAngleNegative(3, angle)
#                 # joinit 4
#                 elif event.key == pygame.K_d:
#                     robot.rotateCertainAnglePositive(4, angle)
#                 elif event.key == pygame.K_f:
#                     robot.rotateCertainAngleNegative(4, angle)
#                 # joinit 5
#                 elif event.key == pygame.K_c:
#                     robot.rotateCertainAnglePositive(5, angle)
#                 elif event.key == pygame.K_v:
#                     robot.rotateCertainAngleNegative(5, angle)
#                 # close RG2
#                 elif event.key == pygame.K_t:
#                     robot.closeRG2()
#                 # # open RG2
#                 elif event.key == pygame.K_y:
#                     robot.openRG2()
#                 # save Images
#                 elif event.key == pygame.K_SPACE:
#                     rgbImg = robot.getImageRGB()
#                     depthImg = robot.getImageDepth()
#                     # 随机生成8位ascii码和数字作为文件名
#                     ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
#                     cv2.imwrite("saveImg\\rgbImg\\"+ran_str+"_rgb.jpg", rgbImg)
#                     cv2.imwrite("saveImg\\depthImg\\"+ran_str+"_depth.jpg", depthImg)
#                     print("save image")
#                 # reset angle
#                 elif event.key == pygame.K_l:
#                     robot.rotateAllAngle([0,0,0,0,0,0])
#                     angle = float(eval(input("please input velocity: ")))
#                 else:
#                     print("Invalid input, no corresponding function for this key!")
                    
# if __name__ == '__main__':
#     main()
# Load coppeliaSim library
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

# Load other libraries for delays and specifying coordinates as multiples of pi
import time
import math as m
import numpy as np

def ur5_params():
    """
    Returns
    -------
    home_position : Home position of the UR5 with DH parameters
    screw : Screw matrix for UR5 with DH parameters
    """
    # UR5 link parameters (in meters)
    l1 = 0.425
    l2 = 0.392
    h1 = 0.160
    h2 = 0.09475
    w1 = 0.134
    w2 = 0.0815

    home_position = np.array([
      [-1, 0, 0, l1 + l2],
      [0, 0, 1, w1 + w2],
      [0, 1, 0, h1 - h2],
      [0, 0, 0, 1]
    ])

    screw = np.array([
      [0,0,1,0,0,0],
      [0,1,0,-h1,0,0],
      [0,1,0,-h1, 0,l1],
      [0,1,0, -h1, 0, l1+l2],
      [0,0,-1, -w1, l1+l2, 0],
      [0,1,0, h2-h1, 0, l1+l2]
    ])
    return home_position, screw

def crossProductOperator(vector):
    w1,w2,w3 = vector
    W = [
        [  0, -w3,  w2],
        [ w3,   0, -w1],
        [-w2,  w1,   0]
        ]
    Ax = np.asarray(W)
    return Ax

def exponential_map(action_axis, theta):
    action_axis = np.asarray(action_axis)
    linear = action_axis[3:]
    angular = action_axis[:3]

    exp_rot = exponential_form_rotation(angular, theta)
    exp_tras = exponential_form_traslation(linear, angular, theta)

    expMap = np.block([[exp_rot, exp_tras], [np.zeros( (1,3) ), 1]])
    return(expMap)

def exponential_form_rotation(angular, theta):
    c = crossProductOperator(angular) @ crossProductOperator(angular)
    expRot = np.eye(3) + np.sin(theta) * crossProductOperator(angular) + ( 1-np.cos(theta) ) * c
    return expRot

def exponential_form_traslation(linear, angular, theta):
    l1, l2, l3 = linear
    lin = np.array([[l1, l2, l3]])
    angular_mat = crossProductOperator(angular)
    c = angular_mat  @ angular_mat
    expTras = (theta * np.eye(3) + ( 1 - np.cos(theta) ) * angular_mat + ( theta - np.sin(theta) ) * c) @ (lin.transpose())
    return expTras

def Ad(mat4):
    mat4  = np.asarray(mat4)
    rot = mat4[:3, :3]
    tras = mat4[0:3, 3]
    Ad = np.block([[rot, crossProductOperator(tras) @ rot],[np.zeros((3,3)), rot]])
    return(Ad)

def denavit_transformation(theta, index):
    """
    Computes the homogeneous transformation according to the Denavit-Hatenberg convention
    Parameters
    ----------
    theta : Rotation in z-axis [radians].
    Internal variables
    ------------------
    d : Distance between x-axes in meters.
    alpha : Angle between z_1 and and z_0 axes.
    r : Distance between z-axes.
    Returns
    -------
    G : Homogeneous transformation.
    """

    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0523]
    alpha = [m.pi/2, 0, 0, m.pi/2, -m.pi/2, 0]
    r = [0, -0.425, -0.3922, 0, 0, 0]

    c_theta = m.cos(theta)
    s_theta = m.sin(theta)
    c_alpha = m.cos(alpha[index])
    s_alpha = m.sin(alpha[index])

    print('DH Values: ', c_theta, s_theta, c_alpha, s_alpha)

    R_z = np.array([
                    [c_theta, -s_theta, 0, 0],
                    [s_theta, c_theta, 0, 0],
                    [0, 0, 1 ,0],
                    [0, 0, 0, 1]
                    ]) # DH rotation z-axis
    T_z = np.array([
                    [1, 0 ,0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1 , d[index]],
                    [0, 0, 0, 1]
                    ]) # DH translation z-axis
    T_x = np.array([
                    [1, 0, 0, r[index]],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ]) # DH translation x-axis
    R_x = np.array([
                    [1, 0, 0, 0],
                    [0, c_alpha, -s_alpha, 0],
                    [0, s_alpha, c_alpha, 0],
                    [0, 0, 0, 1]
                    ]) # DH rotation x-axis

    # print(R_z, T_z, T_x, R_x)

    G = R_z @ T_z @ T_x @ R_x

    return G

def compute_jacobian(theta, screw, dof=6 ):
    # Compute DH transformations
    # Compute exponential maps too

    G_local = []
    expMap_local = []
    for i in range(dof):
        G_local.append(denavit_transformation(theta[i], i))
        expMap_local.append(G_local[i] @ exponential_map(screw[i], theta[i]))

    # Get G for the adjoint operator
    G = []
    for i in range(dof):
        if i == 0:
            g = np.eye(4)
        else:
            g = g @ G[i-1]
        G.append(g @ expMap_local[i])

    # Get space Jacobian
    J_s = []
    for i in range(6):
        J_s.append(Ad(G[i]) @ screw[i])

    # print('Space : \n', J_s)

    # Get location of end effector tip (p_tip)
    p_k = np.zeros((3,1))
    # p_k = np.array([[0],[0],[0.5]])

    p_0_extended = G[5] @ np.block([[p_k],[1]])
    p_0 = p_0_extended[:3]

    p_0_cpo = np.array([[0, -p_0[2], p_0[1]],[p_0[2], 0, -p_0[0]],[-p_0[1], p_0[0], 0]])

        # Geometric Jacobian
    """
    The geometric Jacobian is obtained from the spatial Jacobian and the vector p_tip
    p_tip : tip point of the end effector
    p_0^tip : p measured from the inertial reference frame
    p_k^tip : p measured from the frame k, if k is the end effector and the tip is at its origin then p_k = 0
    [p_0^tip; 1] = G_0^k [p_k^tip; 1]
    """
    J_g = np.block([[np.eye(3), -p_0_cpo],[np.zeros((3,3)), np.eye(3)]]) @ J_s

    # print('Geometric : \n', J_g)

    # Get Roll, Pitch, Yaw coordinates
    R = G[5][0:3][0:3]

    r_roll = m.atan2(R[2][1],R[2][2])
    r_pitch = m.atan2(-R[2][0],m.sqrt(R[2][2]*R[2][2] + R[2][1]*R[2][1]))
    r_yaw = m.atan2(R[1][0],R[0][0])

    # Build kinematic operator for Roll, Pitch, Yaw configuration
    # Taken from Olguin's formulaire book

    B = np.array(
        [
            [m.cos(r_pitch) * m.cos(r_yaw), -m.sin(r_yaw), 0],
            [m.cos(r_pitch) * m.cos(r_yaw), m.cos(r_yaw), 0],
            [- m.sin(r_pitch), 0, 1]
            ]
        )

    print('Kinematic : \n', B)

    # Get Analytic Jacobian
    """
    Obtained from function
    J_a(q) = [[I 0],[0, B(alpha)^-1]] J_g(q)
    B(alpha) =    for roll, pitch, yaw
    """
    J_a = np.block(
        [
            [np.eye(3), np.zeros((3, 3))],
            [np.zeros((3,3)), np.linalg.inv(B)]
            ]
        )

    return J_a

def compute_e(theta_d, theta_0, dof, home_position, screw):
    T_0 = compute_T(screw, theta_0, dof, home_position)
    T_d = compute_T(screw, theta_d, dof, home_position)
    e = T_d - T_0

    # e = theta_d - theta_0
    return e

def root_finding(theta_0, theta_d, tryMax, dof, home_position, screw):

    n_try = 1; # Count number of iterations
    tol = 0.0001; # error tolerance
    theta = theta_0
    e = compute_e(theta_d, theta, dof, home_position, screw) # Gets error from the transformation matrix

    while n_try < tryMax and np.linalg.norm(e) > tol :
        ja = compute_jacobian(theta, screw)
        j_temp = np.zeros((6,6))
        for i in range(6):
            for j in range (6):
                j_temp[i][j] = ja[i][j]

        inverse_jacobian = np.linalg.inv(j_temp)
        theta = theta + inverse_jacobian @ (theta_d - theta)
        e = compute_e(theta_d, theta, dof, home_position, screw)
        n_try += 1
    return theta, e, n_try


def compute_T(screw, theta, dof, M):
    """
    Parameters
    ----------
    screw : screw matrix.
    theta : coordinates.
    dof : robot degrees of freedom.
    M : 4 x 4 matrix describing the position of the end effector at theta_i = 0.
    Returns
    -------
    T : New end effector position.
    """

    expMap_local = []
    T = np.eye(4)
    for i in range(dof):
        expMap_local.append(exponential_map(screw[i], theta[i]))
        T = T @ expMap_local[i]
    T = T @ M
    return T

def main():
    np.set_printoptions(precision=3, suppress=True) # Restrict decimals in console output

    dof = 6
    home_position, screw = ur5_params() # Get UR5 parameters

    theta_0 = np.array([0, -m.pi/2, 0, m.pi/2, 0, 0]) # Initial position
    theta_d = np.array([0,-m.pi/2, -m.pi/2, 0, 0, 0]) # Desired position, converted to x_d in the solver

    T_0 = compute_T(screw, theta_0, dof, home_position)
    T_d = compute_T(screw, theta_d, dof, home_position)

    print("Home position: \n", T_0, "\n")
    print("Desired position: \n", T_d, "\n")

    # Find solution to the system
    theta, delta, n = root_finding(theta_0, theta_d, 20, dof, home_position, screw)
    T_theta = compute_T(screw, theta, dof, home_position)

    print('Solution : \n', theta, '\n', 'Transformation : \n', T_theta, '\n')

    R = T_theta[0:3][0:3] # Get RPY for the solution

    r_roll = m.atan2(R[2][1],R[2][2])
    r_pitch = m.atan2(-R[2][0],m.sqrt(R[2][2]*R[2][2] + R[2][1]*R[2][1]))
    r_yaw = m.atan2(R[1][0],R[0][0])

    # Begin connection with coppeliaSim.
    # Robot simulation must be running in coppeliaSim to connect
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

# clientID stores the ID assingned by coppeliaSim, if the connection failed it will be assigned -1
    if clientID!=-1:
        print ('Connected to remote API server')

# Get ID handles for each joint of the robot
        res,UR5_joint1 =sim.simxGetObjectHandle( clientID, 'UR5_joint1',    sim.simx_opmode_blocking)
        res,UR5_joint2 =sim.simxGetObjectHandle( clientID, 'UR5_joint2', sim.simx_opmode_blocking)
        res,UR5_joint3 =sim.simxGetObjectHandle( clientID, 'UR5_joint3', sim.simx_opmode_blocking)
        res,UR5_joint4 =sim.simxGetObjectHandle( clientID, 'UR5_joint4', sim.simx_opmode_blocking)
        res,UR5_joint5 =sim.simxGetObjectHandle( clientID, 'UR5_joint5', sim.simx_opmode_blocking)
        res,UR5_joint6 =sim.simxGetObjectHandle( clientID, 'UR5_joint6', sim.simx_opmode_blocking)

        res,UR5_endEffector =sim.simxGetObjectHandle( clientID, 'UR5_connection', sim.simx_opmode_blocking)

# Store the handles as a list
    UR5 = [UR5_joint1, UR5_joint2, UR5_joint3, UR5_joint4, UR5_joint5, UR5_joint6] # Just grab the first 3 for now to test

# Get current coordinates of each joint
    UR5_q = []
    for joint in UR5:
        res, value = sim.simxGetJointPosition(clientID, joint, sim.simx_opmode_oneshot)
        UR5_q.append(value) # Store current values

# Set new coordinates for the robot in coppeliaSim
# Add time delays so the animation can be displayed correctly
    steps = 100
    position_desired = theta

    for t in range(steps):
        i = 0
        k = 0
        for joint in UR5:
            print(t*(position_desired[i])/steps)
            sim.simxSetJointTargetPosition(clientID, joint, t*(position_desired[i])/steps, sim.simx_opmode_streaming)
            res, value = sim.simxGetJointPosition(clientID, joint, sim.simx_opmode_oneshot)
            if t == 0 or t == 99:
                k += 1
                res, position = sim.simxGetObjectPosition(clientID, UR5_endEffector, UR5_joint1,  sim.simx_opmode_oneshot)
                res, orientation = sim.simxGetObjectOrientation(clientID, UR5_endEffector, UR5_joint1,  sim.simx_opmode_oneshot)
            i += 1
            time.sleep(2/steps)

    # Convert robot angles to the 4x4 matrix representation for comparison
    c1, s1 = np.cos(orientation[0]), np.sin(orientation[0])
    c2, s2 = np.cos(orientation[1]), np.sin(orientation[1])
    c3, s3 = np.cos(orientation[2]), np.sin(orientation[2])

    R_ur5 = np.block([
        [c2 * c3, -c2 * s3, s2],
        [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
        [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]
        ])

    p_ur5 = np.array(position).reshape((3,1))

    T_ur5 = np.block([
        [R_ur5, p_ur5],
        [0, 0, 0, 1]
        ])

    print('\n Robot coordinates: \n ', T_ur5 )

    # print('\n Absolute Error : \n', abs(T_theta - T_ur5))

    time.sleep(5)

    # print('\n Returning to home position...')
    # for joint in UR5:
            # sim.simxSetJointTargetPosition(clientID, joint, 0, sim.simx_opmode_streaming)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Excuting forward kinematics in Python',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)


if __name__ == '__main__':
        main()