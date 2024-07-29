import numpy as np
import math
import cv2 as cv


################################################################
#######################旋转向量转旋转矩阵#########################
def rodrigues_rotation(r, theta): #shape(4,4)
    # n旋转轴[3x1]
    # theta为旋转角度
    # 旋转是过原点的，n是旋转轴
    r = np.array(r).reshape(3, 1)
    rx, ry, rz = r[:, 0]
    M = np.array([
        [0, -rz, ry],
        [rz, 0, -rx],
        [-ry, rx, 0]
    ])
    R = np.eye(4)
    R[:3, :3] = np.cos(theta) * np.eye(3) +        \
                (1 - np.cos(theta)) * r @ r.T +    \
                np.sin(theta) * M
    return R

def rodrigues_rotation_vec_to_R(v):
    # r旋转向量[3x1]
    theta = np.linalg.norm(v)
    r = np.array(v).reshape(3, 1) / theta
    return rodrigues_rotation(r, theta)
################################################################
################################################################


################################################################
#######################旋转矩阵转欧拉角##########################
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
################################################################
################################################################

################################################################
#######################旋转矩阵转旋转向量#########################
def rotation_matrix_to_rotation_vector(rotation_matrix): # rotation_matrix.shape(3×3)

    R_vec = cv.Rodrigues(rotation_matrix)[0]
    return R_vec
################################################################
################################################################


################################################################
#######################pose转旋转矩阵（4，4）#####################
def pose_to_matrix(pose): # 将pose（xyz，rxryrz）转换为变换矩阵   

    # 1 建立变换矩阵
    matrix_T = np.eye(4,4)

    # 2 得到旋转矩阵
    vector_R = [pose[3],pose[4],pose[5]]
    matrix_R = rodrigues_rotation_vec_to_R(vector_R) # 得到的是 4×4 的矩阵
    # print(matrix_R)

    # 3 得到位移向量
    vector_P = [pose[0],pose[1],pose[2]]

    # 4 赋值变换矩阵
    matrix_T[:3,:3]=matrix_R[:3,:3]
    matrix_T[:3,3]=vector_P

    # 5 得到变换矩阵
    return matrix_T
################################################################
################################################################

if __name__=="__main__":

    pass
