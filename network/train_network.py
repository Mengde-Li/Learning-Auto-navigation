import os
import shutil
import logging
import argparse
import datetime
from pathlib import Path

import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import DataLoader


from vision_dataset import VisionDataset
from model import ImageCNN



#Printoptions for pytorch
torch.set_printoptions(
    precision=4,    # 精度，保留小数点后几位，默认4
    threshold=1000,
    edgeitems=3,
    linewidth=150,  # 每行最多显示的字符数，默认80，超过则换行显示
    profile=None,
    sci_mode=False  # 用科学技术法显示数据，默认True
)

#--------------------------------------------------------------------------------------------------#
################################# Generate log file for training ###################################
Record_Flag=True
Record_Info="Train data"
Record_File="example_train_data"
#--------------------------------------------------------------------------------------------------#
#region

Abs_Path=os.path.dirname(os.path.abspath(__file__))

def all_logger_info(str):
    if Record_Flag:
        print(str)
        all_logger.info(str)
    else:
        print(str)

def train_logger_info(str):
    if Record_Flag:    
        print(str)
        train_logger.info(str)
    else:
        print(str)

if Record_Flag:
    #1: Generate log file and save_model path
    #1.1: generate save path
    timestr = str(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M'))

    #generate big savepath path
    save_path=Path(Abs_Path+"/log")
    save_path.mkdir(exist_ok=True)
    save_path=save_path.joinpath(Record_File)
    save_path.mkdir(exist_ok=True)
    save_path=save_path.joinpath(timestr)
    save_path.mkdir(exist_ok=True)
    record_txt=open(os.path.join(save_path,"record.txt"),'a')
    record_txt.write(Record_Info)

    save_models_path=save_path.joinpath("models")
    save_models_path.mkdir(exist_ok=True)
    log_path=save_path.joinpath("logs")
    log_path.mkdir(exist_ok=True)

    #1.2: generate log file to save data
    train_logger=logging.getLogger("train_logger")
    train_logger.setLevel(logging.INFO)
    file_handler=logging.FileHandler(os.path.join(log_path,"train_logger.log"))
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    train_logger.addHandler(file_handler)
    train_logger_info("train logger begin to record data!!!")

    all_logger=logging.getLogger("all_logger")
    all_logger.setLevel(logging.INFO)
    file_handler=logging.FileHandler(os.path.join(log_path,"all_logger.log"))
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    all_logger.addHandler(file_handler)
    all_logger_info("all logger info begin to record data!!!")

    # 1.3: copy trian python file and model file to log data
    shutil.copy(os.path.abspath(__file__), save_path)#copy train file
    shutil.copy(os.path.join(Abs_Path,"model.py"),save_path)
    train_logger_info("copy train file:{} to save_path {}".format(os.path.abspath(__file__),save_path))

    # 1.4: set paraser info
    parser = argparse.ArgumentParser(description='VAE MNIST Example')
    parser.add_argument('--batch-size', type=int, default=100, metavar='N',
                        help='input batch size for training (default: 128)')
    parser.add_argument('--epochs', type=int, default=10, metavar='N',
                        help='number of epochs to train (default: 10)')
    parser.add_argument('--no-cuda', action='store_true', default=False,
                        help='disables CUDA training')
    parser.add_argument('--seed', type=int, default=1, metavar='S',
                        help='random seed (default: 1)')
    parser.add_argument('--log-interval', type=int, default=40, metavar='N',
                        help='how many batches to wait before logging training status')
    parser.add_argument('--lr', type=float, default=1e-3, metavar='N',
                        help='learning rate')
    args = parser.parse_args()

    #--------------------------------------------------------------------------------------------------#
    #---------------------------------------Sepcial setting--------------------------------------------#
    # shutil.copy(os.path.join(Abs_Path,"Models.py"), save_path)
    # results_path=save_path.joinpath("results")
    # results_path.mkdir(exist_ok=True)
    args.cuda = not args.no_cuda and torch.cuda.is_available()
    torch.manual_seed(args.seed)
#endregion
#--------------------------------------------------------------------------------------------------#1

def train_vision_data():
    """
    To test different serial generalization
    """
    #1: Init all data
    dataset_path="/home/elevenjiang/Documents/Project/Easy_Robot/code/network/big_dataset"

    NUM_batch_size=128
    NUM_workers=8
    NUM_train_epoch=200
    NUM_lowest_loss=1000
    Flag_shuffle=True

    train_dataset=VisionDataset(dataset_path,split='train',split_rate=0.8)
    test_dataset=VisionDataset(dataset_path,split='test',split_rate=0.8)
    train_loader=DataLoader(train_dataset,batch_size=NUM_batch_size,shuffle=Flag_shuffle,num_workers=NUM_workers)
    test_loader=DataLoader(test_dataset,batch_size=NUM_batch_size,shuffle=Flag_shuffle,num_workers=NUM_workers)
    
    
    network=ImageCNN(output_delta=7).cuda()
    optimizer=torch.optim.Adam(network.parameters(),lr=0.001)

    
    #2: Begin to train the network
    for epoch in range(0,NUM_train_epoch):

        #2.1 In training process
        sum_train_loss=0
        sum_test_loss=0
        network.train()
        for batch_idx,data in enumerate(train_loader):
            image,delta_pose=data
            image,delta_pose=image.cuda(),delta_pose.cuda()
            predict_delta_pose=network(image.float())
            
            optimizer.zero_grad()
            loss=F.mse_loss(predict_delta_pose,delta_pose)
            loss.backward()
            optimizer.step()
            sum_train_loss=sum_train_loss+loss.item()

            if batch_idx%10==0:
                batch_size_fmt="{:15}\t".format("Train:"+str(batch_idx*NUM_batch_size))
                all_logger_info(batch_size_fmt+'| Loss: {:.6f}'.format(loss.item()/len(image)))                

        #2.2 In test process
        network.eval()
        for batch_idx,data in enumerate(test_loader):
            image,delta_pose=data
            image,delta_pose=image.cuda(),delta_pose.cuda()
            predict_delta_pose=network(image.float())
            
            loss=F.mse_loss(predict_delta_pose,delta_pose)
            sum_test_loss=sum_test_loss+loss.item()

            if batch_idx%10==0:
                batch_size_fmt="{:15}\t".format("Test:"+str(batch_idx*NUM_batch_size))
                all_logger_info(batch_size_fmt+'| Loss: {:.6f}'.format(loss.item()/len(image)))


                

        #3: Save result
        average_train_loss=sum_train_loss/(len(train_loader)*NUM_batch_size)
        average_valid_loss=sum_test_loss/(len(test_loader)*NUM_batch_size)

        epoch_fmt="{:10}\t".format("Epoch:"+str(epoch))
        train_logger_info(epoch_fmt+"|Train loss: {:.6f} Test loss: {:.6f}".format(average_train_loss,average_valid_loss))
        all_logger_info(epoch_fmt+"|Train loss: {:.6f} Test loss: {:.6f}".format(average_train_loss,average_valid_loss))

        if Record_Flag:
            if average_valid_loss<NUM_lowest_loss:
                NUM_lowest_loss=average_valid_loss
                all_logger_info("!!!!Find new lowest loss in epoch{},loss is:{}!!!!!".format(epoch,average_valid_loss))
                bestsave_path=os.path.join(save_models_path,'best_model.pth')
                torch.save(network.state_dict(), bestsave_path)
    
            lastsave_path=os.path.join(save_models_path,'last_model.pth')
            torch.save(network.state_dict(), lastsave_path)


if __name__ == "__main__":
    train_vision_data()

