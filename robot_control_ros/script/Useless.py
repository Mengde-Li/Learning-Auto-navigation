#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import time
import copy
import math
import numpy as np


import rospy
import actionlib
from std_msgs.msg import Bool,Int16
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal


Abs_Path=os.path.dirname(os.path.abspath(__file__))


class Franka_Robot:
    def __init__(self,init_node=True):
        if init_node:
            rospy.init_node("Franka_control")

        #1: define action
        self.franka_client=actionlib.SimpleActionClient("/position_joint_trajectory_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        

        #2:Define the robot state parameter
        # self.robot_joint_name=["panda_finger_joint1", "panda_finger_joint2", 
        #                         "panda_joint1", "panda_joint2", "panda_joint3",
        #                         "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        self.robot_joint_name=["panda_joint1", "panda_joint2", "panda_joint3",
                          "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

        
        self._num_joints=len(self.robot_joint_name) 
        self._q=None#Store joint position(rad)(1*6 joint)
        self._dq=None#Store joint velocity
        self._x=None#Store end pose(xyz,qxqyqzqw)

        self.home_joint=np.array([])
        self.home_x=np.array([])

        #3: set callback function
        self.joint_state_sub=rospy.Subscriber("/joint_states", JointState, self.robot_state_cb, queue_size=1000)#get joint state subscriber
        self.record_joints_sub=rospy.Subscriber("/record_state",Int16,self.record_joints_cb,queue_size=1000)
        self.Flag_record_joints=False


        #4: Start robot
        self.franka_client.wait_for_server()
        while not rospy.is_shutdown():
            if self._q is None:
                rospy.logwarn("Robot._q is None..")
                rospy.sleep(0.1)
            else:
                print("Robot is Ready!Connected to Franka action server")
                break


    def robot_state_cb(self,data):
        """
        callback function for /joint_states
        """
        q=np.zeros(self._num_joints)
        dq=np.zeros(self._num_joints)

        # for i,name in enumerate(data.name):
        #     index=self.robot_joint_name.index(name)
        #     q[index]=data.position[i]
        #     dq[index]=data.velocity[i]

        for i,name in enumerate(self.robot_joint_name):
            index=data.name.index(name)
            q[i]=data.position[index]
            dq[i]=data.position[index]

            
        self._q=q
        self._dq=dq
        

        time.sleep(0.0002)

    def record_joints_cb(self,data):
        self.Flag_record_joints=data.data
        print("Get Flag_record_joints state is:")
        print(self.Flag_record_joints)
        if self.Flag_record_joints==1:
            print("!!!!!!!!!!!!Begin to record data!!!!!!!!!!!!!!!!!!")
        
    def get_joints_state(self):
        data=rospy.wait_for_message('/joint_states',JointState)#ensure that joint_states is the newest
        # for i,name in enumerate(data.name):
        #     index=self.robot_joint_name.index(name)
        #     self._q[index]=data.position[i]
        #     self._dq[index]=data.velocity[i]

        for i,name in enumerate(self.robot_joint_name):
            index=data.name.index(name)
            self._q[i]=data.position[index]
            self._dq[i]=data.position[index]
        return list(self._q)


    def home(self):
        pass


    def move_joint(self,joints_all,t=5,safety_check=True):
        """
        move to all joint
        :param joints_all: 6 joint target pose,from shoulder_pan_joint to wrist_3_joint(different from /joint_states)
        :param t: the whole time to move all joints
        :param safety_check: avoid move too big.closed only if the robot move home
        :return:
        """
        joints_all=np.array(joints_all)
        if joints_all is None:
            print("[Warning] the joints_all is None,move_joint do not work")
            return False
        if joints_all.ndim==1:
            joints_all=joints_all.reshape(1,-1)

        print("target joints shape is:")
        print(joints_all.shape)
        

        #1:generate the action goal
        goal=FollowJointTrajectoryGoal()
        goal.trajectory=JointTrajectory()
        goal.trajectory.joint_names=self.robot_joint_name
        goal.trajectory.points=[]

        #2:if go home,directly move with out safety check
        if not safety_check:
            q=JointTrajectoryPoint()
            q.positions=self.home_joint
            q.velocities=[0.]*self._num_joints
            q.time_from_start=rospy.Duration(t)
            goal.trajectory.points.append(q)

        else:
            temp_time=t/float(joints_all.shape[0])
            print("temp time is:")
            print(temp_time)
            print("joints_all.shape:")
            print(joints_all.shape[0])
            for i in range(joints_all.shape[0]):
                #append one joints if it is safety
                q=JointTrajectoryPoint()
                joint_list=joints_all[i,:].tolist()
                q.positions=joint_list
                # q.velocities=[0.]*self._num_joints
                q.time_from_start=rospy.Duration(temp_time*(i+1))
                goal.trajectory.points.append(q)

             
        #3:send goal to robot and move        
        try:
            self.franka_client.send_goal(goal)
            begin=time.time()
            self.franka_client.wait_for_result()#will return True though franka_client.cancel_all_goals()
            stop=time.time()
            wait_time=stop-begin
            if wait_time<t:
                rospy.logerr("Move joint t is less than wait_time!!!!")
                print("**************************")
                print("move_joint t is:{}".format(t))
                print("Robot wait for result need time:{}".format(stop-begin))
                print("**************************")
                raw_input("wait!!")
                return False

        except KeyboardInterrupt:
            rospy.logerr("You press Ctrl+C,robot stop!!!")
            self.franka_client.cancel_goal()
            return False

        except Exception as e:
            print("Error is:{}".format(e))
            rospy.logerr("franka_client.send_goal failed")
            self.stop_move()
            raise

        return True


def get_robot_state():
    franka_Robot=Franka_Robot()
    while not rospy.is_shutdown():
        robot_joints=franka_Robot.get_joints_state()
        print("Get robot joints is:")
        print(robot_joints)
        break

def example_move_joint():
    franka_Robot=Franka_Robot( )
    while not rospy.is_shutdown():
        robot_joints=franka_Robot.get_joints_state()

        print("Get robot joints is:")
        print(robot_joints)
        target_joints=robot_joints
        target_joints[0]=target_joints[0]+0.1
        franka_Robot.move_joint(target_joints,t=3)
        time.sleep(1)


        robot_joints=franka_Robot.get_joints_state()
        print("Get robot joints is:")
        print(robot_joints)
        target_joints=robot_joints
        target_joints[0]=target_joints[0]-0.1
        franka_Robot.move_joint(target_joints,t=3)
        time.sleep(1)

        break

        
def record_trajectory():
    """
    Begin record:
        rostopic pub /record_state std_msgs/Int16 "data: 0" 
    Stop record:
        rostopic pub /record_state std_msgs/Int16 "data: 0" 
    
    """
    save_number=len(os.listdir(os.path.join(Abs_Path,"save_trajectorys")))
    franka_Robot=Franka_Robot()
    sample_hz=1000
    sample_time=1/float(sample_hz)

    record_poses_list=[]
    begin_record_flag=False
    while not rospy.is_shutdown():
        if franka_Robot.Flag_record_joints==1:
            begin_time=time.time()
            robot_joints=copy.deepcopy(franka_Robot._q)
            record_poses_list.append(robot_joints)
            sleep_time=sample_time-(time.time()-begin_time)
            time.sleep(max(sleep_time,0))

            if franka_Robot.Flag_record_joints==0:
                break

        else:
            print("wait for record data...")
            time.sleep(1)

    target_joints=np.array(record_poses_list)
    print("save target_joinst pose is:")
    print(target_joints.shape)

    np.save(os.path.join(Abs_Path,"save_trajectorys/target_joints_{}.npy".format(save_number)),target_joints)
    



def replay_robot_trajectory():
    target_poses=np.load(os.path.join(Abs_Path,"save_trajectorys/target_joints_3.npy"))
    print(target_poses.shape)

    first_pose=target_poses[0]


    franka_Robot=Franka_Robot()
    while not rospy.is_shutdown():
        franka_Robot.move_joint(first_pose,t=5)
        print("Robot move to first pose")
        break

    print("Robot begin to replay robot trajectory")
    time=target_poses.shape[0]*0.01
    while not rospy.is_shutdown():
        franka_Robot.move_joint(target_poses,t=time)
        break
        


if __name__ == "__main__":
    # get_robot_state()
    # example_move_joint()
    # record_trajectory()
    replay_robot_trajectory()

    