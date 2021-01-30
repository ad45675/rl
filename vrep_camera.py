import sim as vrep
import time
import math
import numpy as np
import cv2 as cv
import PIL.Image as Image
import os
from Rot2RPY import Rot2RPY,euler2mat,Rot2RPY_version2
import matplotlib.pyplot as mpl
from imutils import perspective
from imutils import contours
import imutils
import argparse
from scipy.spatial import distance as dist
#-----影像長寬
width=512
heihth=424

#-----角度(不知道要幹嘛跟內參有關)
theta=70

deg2rad=math.pi/180

#-----深度圖的東西
dis_far=10
depth_scale=1000

#-----存照片路徑
SAVE_PATH_COLOR='C:/Users/ad456/Desktop/my_robot_dh'



def save_txt(path, name, data, fmt='%f'):
    f = open(path + name, 'w')
    np.savetxt(f, data, fmt=fmt)
    f.close()
def path_exsit(path):
    if os.path.exists(path):
        return True
    else:
        return False
def creat_path(path):
    if path_exsit(path=path):
        print(path+' exist')
    else:
        os.makedirs(path)

def save_image(cur_depth,cur_color,img_idx):
    ##   存影像圖  array转换成image
    img=Image.fromarray(cur_color.astype(np.uint8),mode='RGB')   #.convert('RGB')  #array到image的實現轉換
    img_path=os.path.join(SAVE_PATH_COLOR,str(img_idx)+'_rgb.png')
    img.save(img_path)
    ##   存深度圖
    depth_img = Image.fromarray(cur_depth.astype(np.uint32),mode='I')  # array到image的實現轉換
    depth_path=os.path.join(SAVE_PATH_COLOR,str(img_idx)+'_depth.png')
    depth_img.save(depth_path)
    # creat_path('./IK_ans/')
    # save_txt(path='./IK_ans/',name='IK.txt',data=invsol[7],fmt='%f')
    return img_path,depth_path


def set_up_camera(camera_handle):
    #----------------------------get camera pose
    _,cam_position=vrep.simxGetObjectPosition(clientID,camera_handle,-1,vrep.simx_opmode_blocking)
    _,cam_orientation=vrep.simxGetObjectOrientation(clientID,camera_handle,-1,vrep.simx_opmode_blocking)

    cam_trans=np.eye(4,4)
    cam_trans[0:3,3]=np.asarray(cam_position)
    cam_orientation=[-cam_orientation[0],-cam_orientation[1],-cam_orientation[2]]
    cam_rotm=np.eye(4,4)

    cam_rotm[0:3,0:3]=euler2mat(cam_orientation[0],cam_orientation[1],cam_orientation[2])#逆矩陣
    cam_pose=np.dot(cam_trans,cam_rotm)
    return cam_position,cam_pose,cam_rotm

def set_up_robot(robot_handle):
    _,robot_position=vrep.simxGetObjectPosition(clientID,robot_handle,-1,vrep.simx_opmode_blocking)
    _,robot_orientation=vrep.simxGetObjectOrientation(clientID,robot_handle,-1,vrep.simx_opmode_blocking)

    robot_trans = np.eye(4,4)
    robot_trans[0:3,3] = np.asarray(robot_position)
    robot_rotm = np.eye(4,4)
    robot_rotm[0:3,0:3] = np.linalg.inv(euler2mat(robot_orientation[0],robot_orientation[1],robot_orientation[2]))
    robot_pose=np.dot(robot_trans,robot_rotm)

    return robot_position,robot_pose

def intri_camera():
#----------------------------get camera 內參
    fx=width/2.0/(math.tan(theta*deg2rad/2.0))
    fy=heihth/2.0/(math.tan(theta*deg2rad/2.0))
    u0=heihth/2
    v0=width/2
    intri=np.array([
                    [fx,0,u0],
                    [0,fy,v0],
                    [0, 0, 1]])
    intri_real=np.array([
        [1.5627044545649730 * math.pow(10, 3),                                   0, 2.8477025480870020 * math.pow(10, 2)],
        [                                   0,1.5637307281700662 * math.pow(10, 3), 2.8292536344105076 * math.pow(10, 2)],
        [                                   0,                                   0, 1]])

    return intri
#---------------------------讀影像資訊並轉為np array

def get_camera_data(kinectRGB_handle,kinectDepth_handle):
    #---------------------------彩色圖片
    res,resolution,raw_image=vrep.simxGetVisionSensorImage(clientID,kinectRGB_handle,0,vrep.simx_opmode_blocking)
    color_img=np.array(raw_image,dtype=np.uint8)
    color_img.shape=(resolution[1],resolution[0],3)
    color_img=color_img.astype(np.float)/255
    color_img[color_img<0]+=1   #這甚麼??
    color_img*=255
    color_img=np.flipud(color_img)  #翻轉列表

    # color_img = cv.flip(color_img, 0)

    color_img=color_img.astype(np.uint8)  #np.uint8[0,255]  如果是float 就是灰階圖片


    #---------------------------深度圖片
    res,resolution,depth_buffer=vrep.simxGetVisionSensorDepthBuffer(clientID,kinectDepth_handle,vrep.simx_opmode_blocking)
    # print('depth_buffer',depth_buffer)
    depth_img=np.array(depth_buffer)
    depth_img.shape=(resolution[1],resolution[0])
    depth_img=np.flipud(depth_img)  #翻轉列表
    depth_img[depth_img<0]=0
    depth_img[depth_img>1]=0.9999
    depth_img=depth_img*dis_far*depth_scale
    cur_depth=depth_img
    return color_img,depth_img


#---------------------------存圖片到資料夾
def save_image_and_show(kinectRGB_handle,kinectDepth_handle,img_indx):
    color_img,cur_depth=get_camera_data(kinectRGB_handle,kinectDepth_handle)
    img_path,depth_path=save_image(cur_depth,color_img,img_indx)
    #---------------------------從資料夾讀圖片並秀出來
    bg_depth=cv.imread(depth_path,-1)/10000
    bg_color=cv.imread(img_path)/255.0
    cv.imshow('color Image',bg_color)
    cv.imshow('depth Image', bg_depth)
    # 按下任意鍵則關閉所有視窗
    cv.waitKey(0)
    cv.destroyAllWindows()
    #---------------------------從資料夾讀圖片並秀出來

def pixel2myrobot(u,v,cur_depth,robot_position,cam_position,depth=0.0,is_dst=True):
    intri=intri_camera()

    # ---------------------------[u,v] to camera frame
    if is_dst==False:
        depth=cur_depth[int(u)][int(v)]/depth_scale
    depth=cur_depth[int(u)][int(v)]
    x=depth*(u-intri[0][2]/intri[0][0])
    y=depth*(v-intri[1][2]/intri[1][1])
    camera_coor=np.array([x,y,depth])

    # ---------------------------camera to robot frame
    camera_coor[2]=-camera_coor[2]
    print('camera_coor........',camera_coor)

    location=camera_coor+cam_position-np.asarray(robot_position)
    return location,depth

def get_pointcloud(color_img,depth_img,camera_intri):

    #get depth image size
    im_h = depth_img.shape[0]
    im_w = depth_img.shape[1]

    #project depth into 3D point cloud in camera coord

    pix_x, pix_y = np.meshgrid(np.linspace(0, im_w - 1, im_w), np.linspace(0, im_h - 1, im_h))
    # print('u,v',pix_y,pix_x)  #不知道是啥
    cam_pts_x = np.multiply(pix_x - camera_intri[0][2], depth_img / camera_intri[0][0])  # (u-u0)*z/kx
    cam_pts_y = np.multiply(pix_x - camera_intri[1][2], depth_img / camera_intri[1][1])  # (u-u0)*z/kx
    cam_pts_z = depth_img.copy()
    cam_pts_x.shape = (im_h * im_w, 1)
    cam_pts_y.shape = (im_h * im_w, 1)
    cam_pts_z.shape = (im_h * im_w, 1)

    rgb_pts_r = color_img[:, :, 0]
    rgb_pts_g = color_img[:, :, 1]
    rgb_pts_b = color_img[:, :, 2]
    rgb_pts_r.shape = (im_h * im_w, 1)
    rgb_pts_g.shape = (im_h * im_w, 1)
    rgb_pts_b.shape = (im_h * im_w, 1)

    cam_pts = np.concatenate((cam_pts_x,cam_pts_y,cam_pts_z),axis=1)  #(u,v),在相機座標
    rgb_pts = np.concatenate((rgb_pts_r,rgb_pts_g,rgb_pts_b),axis=1)  #點雲圖座標

    return cam_pts,rgb_pts

def get_heightmap(color_img,depth_img,camera_intri,cam_pos,workspace_limits,heightmap_resolution):
    #compute heightmap size
    heightmap_size = np.round(((workspace_limits[1][1]-workspace_limits[1][0])/heightmap_resolution,(workspace_limits[0][1]-workspace_limits[0][0])/heightmap_resolution))

    #get 3D point cloud from RGBD image
    surface_pts,color_pts = get_pointcloud(color_img,depth_img,camera_intri)
    print('robot ', surface_pts)
    #transfrom 3D point cloud from camera coord to robot coord
    surface_pts=np.transpose(np.dot(cam_pos[0:3,0:3],np.transpose(surface_pts))+np.tile(cam_pos[0:3,3:],(1,surface_pts.shape[0])))
    print('robot coor',surface_pts)

    #sort surface point by z values
    sort_z_ind=np.argsort(surface_pts[:,2])
    # print('z',sort_z_ind)
    surface_pts=surface_pts[sort_z_ind]
    color_pts = color_pts[sort_z_ind]

    #filter out surface points outside heightmap boundaries
    heightmap_valid_ind = np.logical_and(np.logical_and(np.logical_and(np.logical_and(surface_pts[:,0] >= workspace_limits[0][0], surface_pts[:,0] < workspace_limits[0][1]), surface_pts[:,1] >= workspace_limits[1][0]), surface_pts[:,1] < workspace_limits[1][1]), surface_pts[:,2] < workspace_limits[2][1])
    surface_pts=surface_pts[heightmap_valid_ind]
    color_pts=color_pts[heightmap_valid_ind]

    # #creat orthographic top-down-view RGB-D heightmaps
    # color_heightmap_r=np.zeros((heightmap_size[0],heightmap_size[1],1), dtype=np.uint8)
    # color_heightmap_g=np.zeros((heightmap_size[0],heightmap_size[1],1), dtype=np.uint8)
    # color_heightmap_b=np.zeros((heightmap_size[0],heightmap_size[1],1), dtype=np.uint8)
    # depth_heightmap=np.zeros(heightmap_size)
    # heightmap_pix_x = np.floor((surface_pts[:, 0] - workspace_limits[0][0]) / heightmap_resolution).astype(int)
    # heightmap_pix_y = np.floor((surface_pts[:, 1] - workspace_limits[1][0]) / heightmap_resolution).astype(int)
    # color_heightmap_r[heightmap_pix_y, heightmap_pix_x] = color_pts[:, [0]]
    # color_heightmap_g[heightmap_pix_y, heightmap_pix_x] = color_pts[:, [1]]
    # color_heightmap_b[heightmap_pix_y, heightmap_pix_x] = color_pts[:, [2]]
    # color_heightmap = np.concatenate((color_heightmap_r, color_heightmap_g, color_heightmap_b), axis=2)
    # depth_heightmap[heightmap_pix_y, heightmap_pix_x] = surface_pts[:, 2]
    # z_bottom = workspace_limits[2][0]
    # depth_heightmap = depth_heightmap - z_bottom
    # depth_heightmap[depth_heightmap < 0] = 0
    # depth_heightmap[depth_heightmap == -z_bottom] = np.nan
    #
    # return color_heightmap, depth_heightmap


def detect(image,depth,color='green'):
    bg_depth = depth
    bg_color = image
    if color == 'green':
        lower = (40, 60, 60)
        upper = (80, 255, 255)
    if color == 'blue':
        lower = (86, 6, 6)
        upper = (255, 90, 255)
    if color == 'red':
        lower = (5, 5, 80)
        upper = (60, 60, 255)
    if color == 'wt':
        lower = (6, 86, 29)
        upper = (255, 255, 64)


    blur=cv.GaussianBlur(bg_color,(5,5),0)
    hsv = cv.cvtColor(blur,cv.COLOR_BGR2HSV)
    bg_color = cv.cvtColor(np.asarray(bg_color,dtype=np.uint8),cv.COLOR_RGB2BGR)

    # small = cv.resize(bg_color,(0,0),fx=scale,fy=scale)

    #creat a mask for the green areas of the image
    mask=cv.inRange(hsv ,lower,upper)

    bmask=cv.GaussianBlur(mask,(5,5),0)

    moments=cv.moments(mask)
    # print('bmask', moments)
    m00=moments['m00']
    centroid_x,centroid_y = None, None
    if m00!=0:
        centroid_x=int(moments['m10']/m00)
        centroid_y = int(moments['m01'] / m00)
    # ctr=None
    if centroid_x!=None and centroid_y!=None:
        ctr=(centroid_x,centroid_y)
        # print(',ctr',ctr)

    if ctr:
       cv.rectangle(bg_color,(ctr[0]-15,ctr[1]-15),(ctr[0]+15,ctr[1]+15),(0xff,0xf4,0x0d),2)
       cv.circle(bg_color, (int(ctr[0] ), int(ctr[1] )), 2, (0, 0, 255), -1)

    cv.imshow('color Image', bg_color)
    # cv.imshow('depth Image', bmask)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return ctr

def contours(image,depth,color='green',draw=True):   #有問題
    bg_depth = depth
    bg_color = image

    if color == 'green':
        lower = (40, 60, 60)
        upper = (80, 255, 255)
    if color == 'blue':
        lower = (86, 6, 6)
        upper = (255, 90, 255)
    if color == 'red':
        lower = (5, 5, 80)
        upper = (60, 60, 255)
    if color == 'wt':
        lower = (6, 86, 29)
        upper = (255, 255, 64)

    scale=1
    hsv = cv.cvtColor(np.asarray(bg_color,dtype=np.uint8),cv.COLOR_BGR2HSV)
    bg_color = cv.cvtColor(np.asarray(bg_color,dtype=np.uint8),cv.COLOR_RGB2BGR)
    small=cv.resize(hsv,(0,0),fx=scale,fy=scale) #圖像縮放
    mask=cv.inRange(small,lower,upper)
    #erosion and dilation to remove imperfections in masking
    mask=cv.erode(mask,None,iterations=2)  #腐蝕
    mask=cv.dilate(mask,None,iterations=2)  #膨脹


    #Find the contour of masked shapes
    contours=cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    contours = contours[0]
    # print('cour',contours)
    center=None

    #if there is a masked object
    if len(contours)>0:
        #largest contour
        c = max(contours,key=cv.contourArea)

        #Radius
        ((x,y),radius) = cv.minEnclosingCircle(c)

        #moment of the largest contour
        moments=cv.moments(c)
        # print('m',moments)
        center=(int(moments['m10'] / moments['m00']),int(moments['m01'] /moments['m00']))
        # print('center',center)
        if draw:
            #draw appropriate circles
            if radius>2:
                cv.circle(bg_color,(int(x/scale),int(y/scale)),int(radius*1.25),(0,255,255),2)
                cv.circle(bg_color, (int(center[0] / scale), int(center[1] / scale)), 2, (0, 0, 255), -1)

            cv.imshow('contours Image', bg_color)
            cv.waitKey(0)
            cv.destroyAllWindows()
    return center




# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

#----------------------------第一台相機的handle
_,camera_handle=vrep.simxGetObjectHandle(clientID,'kinect', vrep.simx_opmode_blocking)
_,kinectRGB_handle=vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_blocking)
_,kinectDepth_handle=vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_blocking)
#----------------------------第二台相機的handle
_,camera_handle2=vrep.simxGetObjectHandle(clientID,'kinect2', vrep.simx_opmode_blocking)
_,kinectRGB_handle2=vrep.simxGetObjectHandle(clientID,'kinect_rgb2',vrep.simx_opmode_blocking)
_,kinectDepth_handle2=vrep.simxGetObjectHandle(clientID,'kinect_depth2',vrep.simx_opmode_blocking)
#----------------------------myrobot的handle
_,myrobot_handle=vrep.simxGetObjectHandle(clientID,'my_robot_base', vrep.simx_opmode_blocking)
_,Cuboid_handle=vrep.simxGetObjectHandle(clientID,'Cuboid', vrep.simx_opmode_blocking)


# vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
# time.sleep(0.2)
# save_image_and_show(kinectRGB_handle,kinectDepth_handle,1)
# save_image_and_show(kinectRGB_handle2,kinectDepth_handle2,2)
# time.sleep(0.2)
# vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)


cam_intri = intri_camera()
bg_depth = cv.imread('1_depth.png', -1) / 10000
bg_color = cv.imread('1_rgb.png')
# ctr=detect(bg_color,bg_depth )
# print('ctr',ctr)
center=contours(bg_color,bg_depth)

get_camera_data(kinectRGB_handle,kinectDepth_handle)

cuboid_in_cam=np.array([[-0.03309895, -0.04838949 ,1.07094836]])
cuboid_in_cam=np.reshape(cuboid_in_cam,(3,1))
print('center',center)
print('intri',cam_intri)
pixel=np.dot(cam_intri,cuboid_in_cam)/cuboid_in_cam[2]
print('pixel',pixel)


fx=512/(2*math.tan(70*3.14/180/2))
print('f',fx)



def show_image(color_image,depth_image):
    cv.imshow('color Image', color_image)
    cv.imshow('depth Image', depth_image)
    cv.waitKey(0)
    cv.destroyAllWindows()






