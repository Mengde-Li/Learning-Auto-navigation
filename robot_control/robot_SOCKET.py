"""
利用socket与机器人通信

"""

import socket
import time
import struct
import numpy as np
class Robot:
    def __init__(self, ip = "0.0.0.0" , port = 30003,):
        self.IPaddress = ip
        self.port=port
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.connect((self.IPaddress, self.port))
        self.dic= {
            'MessageSize': 'i',
            'Time': 'd', 
            'q target': '6d', 
            'qd target': '6d', 
            'qdd target': '6d',
            'I target': '6d',
            'M target': '6d', 
            'q actual': '6d', 
            'qd actual': '6d', 
            'I actual': '6d', 
            'I control': '6d',
            'Tool vector actual': '6d', 
            'TCP speed actual': '6d', 
            'TCP force': '6d', 
            'Tool vector target': '6d',
            'TCP speed target': '6d', 
            'Digital input bits': 'd', 
            'Motor temperatures': '6d', 
            'Controller Timer': 'd',
            'Test value': 'd', 
            'Robot Mode': 'd', 
            'Joint Modes': '6d', 
            'Safety Mode': 'd', 
            'empty1': '6d', 
            'Tool Accelerometer values': '3d', 
            'empty2': '6d', 
            'Speed scaling': 'd', 
            'Linear momentum norm': 'd', 
            'SoftwareOnly': 'd', 
            'softwareOnly2': 'd', 
            'V main': 'd', 
            'V robot': 'd', 
            'I robot': 'd', 'V actual': '6d', 
            'Digital outputs': 'd', 
            'Program state': 'd', 
            'Elbow position': '3d', 
            'Elbow velocity': '3d'
            }
        
    def recvData(self):
        """
        获取机器人的各项数据, 可用字典查寻所需数据
        """
        data = self.connection.recv(1108)
        #names=[]
        ii=range(len(self.dic))
        for key,i in zip(self.dic,ii):
            fmtsize=struct.calcsize(self.dic[key])
            data1,data=data[0:fmtsize],data[fmtsize:]
            fmt="!"+self.dic[key]
            #names.append(struct.unpack(fmt, data1))
            self.dic[key]=self.dic[key],struct.unpack(fmt, data1)

    def getl(self):
        """
        获取TPC position
        """
        self.recvData()
        return self.dic['Tool vector actual'][1]

    def getj(self):
        """
        获取关节各角度数据
        """
        self.recvData()
        return self.dic['q actual'][1]

    def sendCmd(self, cmd: str):
        """
        向机器人发送urscripts指令
        """
        #cmd = b"movej([0.0, 0.0, -0.5, 1.0, 0.0, 0.0] , t= 1)\n"
        self.connection.send(cmd)

    def movej(self, targ_q:list, a = 1.4, v = 1.05, t = 0, r = 0 ):
        cmd = 'movej(' + str(targ_q) + ',' + str(a) +','+ str(v) +','+str(t) +','+ str(r) + ')\n'
        self.connection.send(cmd.encode())
    
    def movel(self, targ_pose, a = 1.2, v = 0.25, t = 0, r = 0 ):
        cmd = 'movel(' + str(targ_pose) + ',' + str(a) +','+ str(v) +','+str(t) +','+ str(r) + ')\n'
        self.connection.send(cmd.encode())

    def movec(self,pose_via, pose_to, a=1.2, v=0.25, r= 0, mode = 0):
        cmd = 'movec(' + str(pose_via)+ ','+str(pose_to) + ',' + str(a) +','+ str(v) +','+str(r) +','+ str(mode) + ')\n'
        self.connection.send(cmd.encode())

    def movep(self, pose, a=1.2, v=0.25, r=0):
        cmd = 'movep(' + str(pose) + ',' + str(a) +','+ str(v) +','+ str(r) + ')\n'
        self.connection.send(cmd.encode())

    def servoj(self, q, a =0, v=0, t=0.02, lookahead_time=0.1, gain=300):
        cmd = 'servoj(' + str(q)+ ','+str(a) + ',' + str(v) +','+ str(t) + ','+ str(lookahead_time)+','+ str(gain)+')\n'
        self.connection.send(cmd.encode())

if __name__=='__main__':
    ur5=Robot("0.0.0.0", 30003)
    ur5.movej([0,0,0,0,0,0], t= 5)
    print(ur5.getl())