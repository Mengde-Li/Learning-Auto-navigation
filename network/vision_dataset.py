import os
import sys
import glob
import shutil
import random
import cv2 as cv
import numpy as np

import torch
from torch.utils.data.dataset import Dataset
from torch.utils.data import DataLoader
from Matrix_transformation import pose_to_matrix
from Matrix_transformation import rodrigues_rotation
from Matrix_transformation import rodrigues_rotation_vec_to_R

Abs_Path = os.path.dirname(os.path.abspath(__file__))

def see_data():
    """
    Basic see record data
    """
    data_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/robot/data/2023-02-17_15-25"

    indexs_files = glob.glob(os.path.join(data_path, "color_*.png"))
    # indexs_files表示所有的图片
    
    pose_array = np.load(os.path.join(data_path, "pose.npz"))['pose_list']
    joints_array = np.load(os.path.join(data_path, "joints.npz"))['joints_list']

    for index in range(len(indexs_files)):
        if index == 0:  # To compute delta xyz
            continue
        print("***************Now is loading:{}/{} data**************************".format(index, len(indexs_files)))
        
        color_image = cv.imread(os.path.join(data_path, "color_{}.png".format(index)))

        pose = pose_array[index]
        joints = joints_array[index]
        pose_before = pose_array[index-1]
        joints_before = joints_array[index-1]

        print('current is image {}\npose is {}\njoints is {}'.format(index,pose,joints))
        print('before is image {}\npose is {}\njoints is {}'.format(index,pose_before,joints_before))

        cv.imshow("color_image", color_image)
        # 从cv.imshow(参数1，参数2) 参数1:一个字符串，代表要显示的图像窗口的名称；参数2:要显示的图像的路径
        cv.waitKey(0)
        # waitKey() 控制着imshow的持续时间，0代表无限等待


def clean_dataset():
    """
    To delete target_objects
    """
    root_data_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/robot/data"
    all_time_folders = os.listdir(root_data_path)
    all_time_folders.sort()

    # 1: Load all time folder
    target_root_data_path = os.path.join(Abs_Path, "clean_data")
    for idx, time_folder in enumerate(all_time_folders):
        print("*********************Processing {}/{} folder...*********************".format(idx, len(all_time_folders)))
        data_path = os.path.join(root_data_path, time_folder)
        indexs_files = glob.glob(os.path.join(data_path, "color_*.png"))

        # 2: Find the xy_move begin index
        pose_array = np.load(os.path.join(
            data_path, "pose.npz"))['pose_list']
        joints_array = np.load(os.path.join(
            data_path, "joints.npz"))['joints_list']

        for index in range(len(indexs_files)):
            if index == 0:  # To compute delta xy
                continue

            pose = pose_array[index]
            pose_before = pose_array[index-1]
            xy_move = np.linalg.norm(pose[:2]-pose_before[:2])
            print("index:{} move is:{:.5}".format(index, xy_move))

            if xy_move > 0.005:  # 选择移动5mm后的照片做为第一张，之前的删除
                print("Robot begin move in {} in folder {}".format(
                    index, time_folder))
                temp_input = input(
                    "To generate new dataset? y for generate n for no")
                if temp_input == 'y':
                    start_move_index = index
                    # 3: Begin to move data
                    target_folder = os.path.join(
                        target_root_data_path, time_folder)
                    if os.path.exists(target_folder):
                        print("{} folder exist!!! please check!".format(
                            target_folder))
                        return
                    else:
                        os.mkdir(target_folder)

                    begin_index = start_move_index  # 确定开始移动的照片是哪一张

                    count = 0
                    pose_list = []
                    joints_list = []

                    while (count+begin_index) <= len(indexs_files):

                        pose_list.append(pose_array[int(begin_index + count)])
                        joints_list.append(joints_array[int(begin_index + count)])
 
                        origin_image_path = os.path.join(
                            data_path, "color_{}.png".format(count + begin_index))
                        
                        target_image_path = os.path.join(
                            target_folder, "color_{}.png".format(count))
                        
                        shutil.copy(origin_image_path, target_image_path)
                        count = count+1

                    pose_save_array = np.array(pose_list)
                    joints_save_array = np.array(joints_list)

                    np.savez(os.path.join(target_folder, "pose.npz"), pose = pose_save_array)
                    np.savez(os.path.join(target_folder, "joints.npz"), joints = joints_save_array)

                break 

def get_train_data():
    """
    build dataset for training
    """
    root_data_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/robot/clean_data"
    all_time_folders = os.listdir(root_data_path)
    all_time_folders.sort()

    # 1: Load all time folder
    
    for idx, time_folder in enumerate(all_time_folders):
        print("*********************Processing {}/{} folder...*********************".format(idx, len(all_time_folders)))
        data_path = os.path.join(root_data_path, time_folder)
        indexs_files = glob.glob(os.path.join(data_path, "color_*.png"))

        # 2: get delta theta and delta vector
        pose_array = np.load(os.path.join(
            data_path, "pose.npz"))['pose_list']
        joints_array = np.load(os.path.join(
            data_path, "joints.npz"))['joints_list']
        
        delta_theta_list = []
        transformation_vector_list = []

        for index in range(len(indexs_files-1)):
         
            # 2.1: get delta theta for each image
            theta = joints_array[index][5]
            theta_after = joints_array[index+1][5]

            delta_theta = theta_after - theta
            delta_theta_list.append(delta_theta)

            # 2.2: get delta vector
            # 2.2.1: get transformation matrix to base form for each image
            rotation_matrix = pose_to_matrix(pose_array[index])
            rotation_matrix_after = pose_to_matrix(pose_array[index+1])

            # 2.2.2: get transformation matrix (current to after) for each image
            transformation_matrix_from_current_to_after = np.dot(np.linalg.inv(rotation_matrix),rotation_matrix_after)

            # 2.2.3: get transformation vector for each image
            transformation_vector = transformation_matrix_from_current_to_after[:3,3]

            norm = np.linalg.norm(transformation_vector)  # 归一化
            normalized_vector = transformation_vector / norm

            transformation_vector_list.append(normalized_vector)
        

        delta_theta_save_array = np.array(delta_theta_list)
        transformation_vector_save_array = np.array(transformation_vector_list)

        np.savez(os.path.join(data_path, "delta_theta.npz"), delta_theta = delta_theta_save_array)
        np.savez(os.path.join(data_path, "transformation_vector.npz"), transformation_vector = transformation_vector_save_array)
        
def move_to_big_dataset(): # move all images, theta and vector to a big dataset, easy for train
    target_move_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/network/big_dataset"
    if not os.path.exists(target_move_path):
        os.mkdir(target_move_path)
    else:
        print("Exist {} folder!!!,please check".format(target_move_path))
        return

    clean_data_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/network/clean_data"
    folders_list = os.listdir(clean_data_path)

    save_count = 0
    train_data_list = []

    for folder in folders_list:
        print("Processing {} folder...".format(folder))
        folder_path = os.path.join(clean_data_path, folder)
        delta_theta_array = np.load(os.path.join(
            folder_path, "delta_theta.npz"))['delta_theta']
        transformation_vector_array = np.load(os.path.join(
            folder_path, "transformation_vector.npz"))['transformation_vector']

        all_color_files = glob.glob(os.path.join(folder_path, "color_*.png"))
        for index in range(len(all_color_files)):
          
            # save color image
            color_image_path = os.path.join(
                folder_path, "color_{}.png".format(index))
            target_color_image_path = os.path.join(
                target_move_path, "color_{}.png".format(save_count))
            shutil.copy(color_image_path, target_color_image_path)

            # save delta theta and transformation vector
            delta_theta = delta_theta_array[index]
            transformation_vector = transformation_vector_array[index]
  
            temporary_list = [transformation_vector[0],transformation_vector[1],transformation_vector[2],delta_theta[0]]
            train_data_list.append(temporary_list)

            save_count = save_count+1

    save_train_data_array = np.array(train_data_list)

    np.savez(os.path.join(target_move_path, "save_train_data_array"), train_data = save_train_data_array)

class VisionDataset(Dataset):

    def __init__(self, dataset_path, split=None, split_rate=None):

        # Example for load all big dataset
        all_color_images_index = glob.glob(os.path.join(dataset_path, "color_*.png"))
        random.shuffle(all_color_images_index)

        self.save_train_data = np.load(os.path.join(
            dataset_path, "save_train_data_array.npz"))['train_data']

        if split == 'train':
            self.index_list = all_color_images_index[:int(len(all_color_images_index)*split_rate)]
        else:
            self.index_list = all_color_images_index[int(len(all_color_images_index)*split_rate):]

        print("Load {} dataset, contain {} data".format(split, len(self.index_list)))

    def __len__(self):
        return len(self.index_list)

    def __getitem__(self, index):
       
        color_image_path = self.index_list[index]
        image = cv.imread(color_image_path)
        image=cv.resize(image,dsize=(128,128))
        image=np.transpose(image,[2,0,1])
                                                                                                                                                                                                                            
        color_file_name = color_image_path.split('/')[-1]
        color_index = color_file_name.split('_')[-1]
        color_index = int(color_index[:-4])                    # get index of image
        save_train_data = self.save_train_data[color_index]  
        
        return image, save_train_data.astype(np.float32)


def example_load_data():

    dataset_path = "/home/elevenjiang/Documents/Project/Easy_Robot/code/network/big_dataset"
    
    tactile_dataset = VisionDataset(dataset_path, split='test', split_rate=0.8)
    train_loader = DataLoader(tactile_dataset, batch_size=1, shuffle=False)

    for data in tactile_dataset:
        image, train_data = data

        print(image.shape)
        print(train_data.shape)
        break

    for data in train_loader:
        image, train_data = data

        print(image.shape)
        print(train_data.shape)

        print(train_data)
        break


if __name__ == '__main__':
    # see_data()
    # clean_dataset()
    # example_load_data()
    # get_train_data()
    move_to_big_dataset()
