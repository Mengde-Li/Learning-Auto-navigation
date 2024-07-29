import os
import sys
import glob
import shutil
import logging
import argparse
import datetime
import numpy as np
from pathlib import Path

import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader


from vision_dataset import VisionDataset
from model import ImageCNN
import cv2 as cv




def test_all_result():
    dataset_path="/home/elevenjiang/Documents/Project/Easy_Robot/code/network/big_dataset"

    all_color_data=glob.glob(os.path.join(dataset_path,"color_*.png"))
    gt_delta_pose_array=np.load(os.path.join(dataset_path, "all_delta_poses.npz"))['all_poses']
    
    
    network=ImageCNN(output_delta=6).cuda()
    network.load_state_dict(torch.load("/home/elevenjiang/Documents/Project/Easy_Robot/code/network/log/example_train_data/2023-02-21_20-31/models/best_model.pth"))
    network.eval()
    
    
    for index in range(len(all_color_data)):
        print("Now index is:{}".format(index))
        #Load image
        image=cv.imread(os.path.join(dataset_path,"color_{}.png".format(index)))
        image=cv.resize(image,(128,128))
        image=np.transpose(image,[2,0,1]).astype(np.float32)
        tensor_image=torch.from_numpy(image).unsqueeze(0).cuda()
        
        
        #Get true delta pose and predict_pose
        gt_delta_pose=gt_delta_pose_array[index]
        predict_delta_pose=network(tensor_image)
        
        print(gt_delta_pose)
        print(predict_delta_pose)
        
        temp=input("wait")
        
        
    
if __name__ == '__main__':
    test_all_result()
