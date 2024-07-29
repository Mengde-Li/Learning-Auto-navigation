import os
import sys
import urx
import time
import datetime
from pathlib import Path
import cv2 as cv
import numpy as np
import signal
import threading

# For canceling signal
signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)

Abs_Path = os.path.dirname(os.path.abspath(__file__))


class UR_Robot:

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self._robot = urx.Robot(self.robot_ip)

        self._q = None
        self._x = None

        self.robot_state_thread = threading.Thread(target=self.robot_state_cb) 
        self.robot_state_thread.start()

        while self._q is None:
            print("Wait for robots joints...")
            time.sleep(1)

        print("UR robot: {} is ready!".format(self.robot_ip))

    def robot_state_cb(self):
        while True:
            self._q = self._robot.getj()
            self._x = self._robot.getl()

            time.sleep(0.008)  # Set in 125hz

    def get_robot_state(self):

        return self._q, self._x


def read_images(): #

    camera = cv.VideoCapture(0)
    while True:
        begin_time = time.time()
        ret, frame = camera.read()
        cv.imshow("image",frame)
        print("camera hz is:{}".format(1/(time.time()-begin_time))) 

        key_input=cv.waitKey(1)

        if key_input==ord('q'):
            break

      

def basic_get_robot():
    robot = UR_Robot(robot_ip="192.168.10.101")

    while True:
        robot_joints, robot_pose = robot.get_robot_state()
        print("Robot new state is:")
 
        print("six joints angle: {}".format(robot_joints))
        print("TCP pose: {}".format(robot_pose))

        time.sleep(1)

def see_all():

    # 2: init robot and camera
    robot = UR_Robot(robot_ip="192.168.10.101")
    camera = cv.VideoCapture(2)
    while True:
        begin_time = time.time()
        ret, frame = camera.read()

        robot_joints, robot_pose = robot.get_robot_state()

        print("six joints angle: {}".format(robot_joints))
        print("TCP pose: {}".format(robot_pose))

        cv.imshow("Record_image", frame)
        key_input =cv.waitKey(1)
        if key_input == ord('q'):
            break
        
    robot.robot_state_thread.stop()

def record_data():

    # 1: create save data
    timestr = str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')) 
    save_path = os.path.join(Abs_Path, "data/{}".format(timestr))

    if os.path.exists(save_path):
        print("!!!!!!!!Error, exist save path!!!!!!!!!!!!!!")
        return 
    else:
        os.mkdir(save_path)
    # 2: init robot and camera
    robot = UR_Robot(robot_ip="192.168.10.101")
    camera = cv.VideoCapture(2)
    save_number = 0

    joints_save_list = []
    pose_save_list = []

    while True:
        begin_time = time.time()
        ret, frame = camera.read()

        robot_joints, robot_pose = robot.get_robot_state()
        save_joints = [robot_joints]
        save_pose = [robot_pose]

        cv.imwrite(os.path.join(save_path, "color_{}.png".format(save_number)), frame)
        save_number=save_number+1

        joints_save_list.append(save_joints)
        pose_save_list.append(save_pose)

        cv.imshow("Record_image", frame)
        key_input = cv.waitKey(200)
        if key_input == ord('q'):
            break

        print("record hz is:{}".format(1/(time.time()-begin_time)))

    
    save_array_joints = np.array(joints_save_list)
    save_array_pose = np.array(pose_save_list) 


    np.savez(os.path.join(save_path, "joints.npz"), joints_list = save_array_joints)
    np.savez(os.path.join(save_path, "pose.npz"), pose_list = save_array_pose)


    print("Finish save {} data".format(save_array_pose.shape[0])) 

    robot.robot_state_thread.stop()

def example_see_data(data):

    dataset_path="C:/Users/Orca/Desktop/ral_data/Imitation_Learning/data20231103/2023-11-02_23-21/pose.npz"
    

    if data == 'joints':

        joints_list = np.load(os.path.join(dataset_path,"joints.npz"))['joints_list']
        for joints in joints_list:
            print(joints)
    
    if data == 'pose':

        pose_list = np.load(os.path.join(dataset_path,"pose.npz"))['pose_list']
        for pose in pose_list:
            print(pose)

    else:
                
        print('warning !!!: only [pose] or [joints] can be input')
    
def example_save_data(data):

    dataset_path="C:/Users/Orca/Desktop/ral_data/Imitation_Learning/data20231103/2023-11-02_23-21/pose.npz"
    
    pose_list = np.load(os.path.join(dataset_path))['pose_list']
    for pose in pose_list:
        print(pose)


if __name__ == "__main__":

    # read_images()
    # basic_get_robot()
    # see_all()
    # record_data()
    example_see_data('pose')
    

