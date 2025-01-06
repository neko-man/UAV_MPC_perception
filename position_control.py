import airsim
import threading
import queue
from collections import deque
from cvxopt import solvers,matrix
import numpy as np
import time
# import os
# import tempfile
# import pprint
# import cv2

#--------------initialization--------------
# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

## the rotation&translation matrix from cam to body
cam2body_R = np.array([[0, 0, 1],
                     [1, 0, 0],
                     [0, 1, 0]])
cam2body_t = np.array([[0.5, 0, 0.1]]).T

fx = fy = 320/(np.tan(np.pi/4))
cam_intrinsic = np.array([[fx, 0, 320],
                          [0, fy, 180],
                          [0, 0, 1]])
inv_intrinsic = np.linalg.inv(cam_intrinsic)

A = np.array([[1,0,0.0476,0,0,0],[0,1,0,0.0476,0,0],[0,0,0.9048,0,0,0],
              [0,0,0,0.9048,0,0],[0,0,0,0,1,0.0476],[0,0,0,0,0,0.9048]])
B = np.array([[0.0024,0,0],[0,0.0024,0],[0.0952,0,0],
              [0,0.0952,0],[0,0,0.0024],[0,0,0.0952]])
n = np.size(A,0)
p = np.size(B,1)
Q = np.diag([8,8,3,3,30,30])
R = np.diag([6,6,30])

prediction_step = 6

u_ref = np.array([[6,0,0]]).T


q = queue.Queue()
img_q = deque(maxlen=2)
class Target_p_thread(threading.Thread):
    def __init__(self,client):
        threading.Thread.__init__(self)
        self.client = client

    def run(self):
        while True:
            if not img_q:
                continue
            img_depth_perspective = img_q[0][0]
            state = img_q[0][1]
            index = img_q[0][2]
            print("\n-------%d------" %index)
            if index > 249:
                break

            position_uav   = state.kinematics_estimated.position   ##world frame  NED；  body frame FRD;  cam frame  RDF
            quaternion_uav = state.kinematics_estimated.orientation   
            rotation_uav   = quanternion2rotation(quaternion_uav)  ##from body to world
            # def simSetCameraPose(self, camera_name, pose, vehicle_name = '', external = False):                                
            # -控制选定摄影机的姿势

            target_point_in_img   = np.array([find_target_point(img_depth_perspective)]).T
            # print("\n target in img: \n", target_point_in_img)
            # print("target_depth: ", target_point_in_img[2])
            # target_point_in_cam   = inv_intrinsic * target_point_in_img

            target_point_in_body  = (cam2body_R @ (inv_intrinsic @ target_point_in_img) - cam2body_t)*0.5
            # print("\n target in body: \n", target_point_in_body)

            target_point_in_world = rotation_uav @ target_point_in_body + np.array([[position_uav.x_val, position_uav.y_val, position_uav.z_val]]).T
            # print("\n target in world: \n", target_point_in_world)
            
            if (target_point_in_world[2] < -10):
                target_point_in_world[2] = -10
            q.put(target_point_in_world)

def MPC_Matrices(A, B, Q, R, N, x_ref, u_ref):
    n = A.shape[0] # A 是 n x n 矩阵, 得到 n
    p = B.shape[1] # B 是 n x p 矩阵, 得到 p

    # 初始化 M 矩阵. M 矩阵是 (N+1)n x n 的，它上面是 n x n 个 "I"，这一步先把下半部分写成 0
    M = np.vstack([np.eye(n), np.zeros((N * n, n))])

    # 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0
    C = np.zeros(((N + 1) * n, N * p))

    # 定义一个 n x n 的 I 矩阵
    tmp = np.eye(n)

    # 更新 M 和 C
    for i in range(N):
        rows = slice((i+1) * n, (i + 2) * n) # 定义当前行数，从 i * n 开始，共 n 行
        C_part = C[rows.start - n:rows.stop - n, :-p]
        # print(np.size(tmp @ B),np.size(C_part))
        C[rows, :] = np.hstack([tmp @ B, C_part]) # 将 C 矩阵填满
        tmp = A @ tmp # 每一次将 tmp 左乘一次 A
        M[rows, :] = tmp # 将 M 矩阵写满

    # 定义 Q_bar 和 R_bar
    Q_bar = np.kron(np.eye(N + 1), Q)
    R_bar = np.kron(np.eye(N), R)
    X_ref = np.tile(x_ref, (N + 1, 1))
    U_ref = np.tile(u_ref, (N, 1))

    # 计算 G, E, H
    H = 2 * (C.T @ Q_bar @ C + R_bar)
    G = 2 * M.T @ Q_bar @ C
    E = 2 * (X_ref.T @ Q_bar @ C + U_ref.T @ R_bar)

    return H, G, E


def quanternion2rotation (quaternion):
    w = quaternion.w_val
    x = quaternion.x_val
    y = quaternion.y_val
    z = quaternion.z_val
    return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
                    [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
                    [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2* y**2]])

def rbf(x,y,sigma):
    return np.exp(-(x**2 + y**2) / (2 * sigma**2))

def find_target_point (depth_perspective):
    sigma = 10
    windowSize = 25
    X,Y = np.meshgrid(np.arange(-windowSize, windowSize),np.arange(-windowSize,windowSize))
    standardRBFMatrix = rbf(X, Y, sigma)
    # localDepthMap  = depth_perspective[windowSize:-windowSize, windowSize:-windowSize]
    # print(depth_perspective.shape)
    maxWeightedSum, maxX, maxY = 0, 0, 0
    
    for x0 in range(windowSize,depth_perspective.shape[0]-windowSize):
        for y0 in range(windowSize,depth_perspective.shape[1]-windowSize):
            localDepthPatch = depth_perspective[x0-windowSize:x0+windowSize, y0-windowSize:y0+windowSize]
            currentWeightedSum = np.sum(standardRBFMatrix * localDepthPatch)
            
            if currentWeightedSum > maxWeightedSum:
                # print(currentWeightedSum, maxX, maxY)
                maxWeightedSum = currentWeightedSum
                maxX = x0
                maxY = y0
 
    # weightedSum    = np.sum(standardRBFMatrix * localDepthMap)
    # maxX, maxY     = np.unravel_index(np.argmax(weightedSum), weightedSum.shape)
    print("x,y in img respectively:", maxX,maxY)
    target_depth = depth_perspective[maxX, maxY]
    return target_depth * maxY, target_depth * maxX, target_depth




#--------------take off & start img thread---------------
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

target_point_thread = Target_p_thread(client)
# target_point_thread.setDaemon(True)
target_point_thread.start()


target_point_in_world = np.array([0,0,0]).T


i = 0
while(True):
    i = i + 1

    # 获取DepthPlanar和DepthPerspective
    responses = client.simGetImages([airsim.ImageRequest('fpv_cam', airsim.ImageType.DepthPerspective, True, False)],vehicle_name="drone_1")
    img_depth_perspective = np.array(responses[0].image_data_float).reshape(responses[0].height, responses[0].width)
    img_depth_perspective[img_depth_perspective > 20] = 20

    state = client.getMultirotorState()
    position_uav   = state.kinematics_estimated.position   ##world frame  NED；  body frame FRD;  cam frame  RDF
    velocity_uav   = state.kinematics_estimated.linear_velocity
    quaternion_uav = state.kinematics_estimated.orientation   
    _,_,yaw = airsim.to_eularian_angles(quaternion_uav)

    img_q.append([img_depth_perspective,state,i])

    while q.empty() and (target_point_in_world == np.array([0,0,0]).T).all():
            print('not target point in world')
            time.sleep(0.5)

    if not q.empty():
        target_point_in_world = q.get()

        # print("\n target in the world at main thread:\n",target_point_in_world)


    x_current = np.array([[position_uav.x_val,position_uav.y_val,velocity_uav.x_val,velocity_uav.y_val,yaw,
                          state.kinematics_estimated.angular_velocity.z_val]]).T

    x_ref = np.array([[target_point_in_world[0,0],target_point_in_world[1,0],0,0,0,0]],dtype=float).T

    [H,G,E] = MPC_Matrices(A,B,Q,R,prediction_step,x_ref,u_ref)

    U_k = solvers.qp(P = matrix(H.astype(np.double)),q = matrix((x_current.T @ G - E).T.astype(np.double)))

    u_k = np.array(U_k['x'])[0:p,0]
    # def simSetCameraPose(self, camera_name, pose, vehicle_name = '', external = False):                                
    # -控制选定摄影机的姿势

    # target_point_in_img   = np.array([find_target_point(img_depth_perspective)]).T
    # print("\ntarget in img: \n", target_point_in_img)
    # # print("target_depth: ", target_point_in_img[2])
    # # target_point_in_cam   = inv_intrinsic * target_point_in_img

    # target_point_in_body  = (cam2body_R @ (inv_intrinsic @ target_point_in_img) - cam2body_t)*0.5
    # print("\ntarget in body: \n", target_point_in_body)

    # target_point_in_world = rotation_uav @ target_point_in_body + np.array([[position_uav.x_val, position_uav.y_val, position_uav.z_val]]).T
    # if (target_point_in_world[2] < -10):
    #     target_point_in_world[2] = -10
    # print("\n set_velocity:", u_k[0],u_k[1])
    # print("\n timestamp: ",time.time())
    error_Z = target_point_in_world[2,0] - position_uav.z_val
    client.moveByVelocityZBodyFrameAsync(u_k[0],u_k[1],-1 + 0.8 * error_Z,5,airsim.DrivetrainType.MaxDegreeOfFreedom,yaw_mode=airsim.YawMode(True,u_k[2]))
    # print("\ntarget_in_world: \n",target_point_in_world)
    # client.moveToPositionAsync(float(target_point_in_world[0]), float(target_point_in_world[1]), 
    #                            float(target_point_in_world[2]), 2).join()

    if i > 250:
        
        airsim.wait_key('Press any key to reset to original state')

        client.reset()
        break
    # time.sleep(0.5)
client.enableApiControl(False)