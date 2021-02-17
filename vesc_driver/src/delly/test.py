import torch
import torch.nn as nn
from dataloader import *
from model.GAIL import Actor,ActorMDN,Discriminator
from albumentations import *
import torch.distributions as td

device = 'cuda'
BATCH_SIZE = 1

#dataset load
aug = Compose([
             Resize(480//2, 640//2, p=1),
            ], p=1.0)
dataset = DellyDataset(transform = aug)
test_iter = torch.utils.data.DataLoader(dataset,batch_size=BATCH_SIZE,shuffle=False)

actor = ActorMDN().to(device).float()
actor.load_state_dict(torch.load("./ckpt/actor.pth"))
actor.eval()

action_list = list()
condition_list = list()
for sample in test_iter:
    expert_state, expert_action, expert_condition = sample
    batch_size = expert_state.size(0)
    expert_state = expert_state.permute(0,3,1,2).float()/255.

    # state -> image 
    # condition -> gps 
    pi,mu,sigma = actor(expert_state.to(device), expert_condition.to(device))
    max_idx = pi.argmax(dim=1)
    idx_gather = max_idx.unsqueeze(dim=-1).repeat(1, mu.shape[2]).unsqueeze(1)
    mu_sel = torch.gather(mu, dim=1, index= idx_gather).squeeze(dim=1)
    sigma_sel = torch.gather(sigma, dim=1, index= idx_gather).squeeze(dim=1)
    m = td.Normal(mu_sel, sigma_sel)
    action = m.sample() # steer
    # sigma & m is output
    # m is mu for steer
    # sigma_sel is for steer not Normal distributed

    #action = actor(expert_state.to(device), expert_condition.to(device))
    action_list.append(float(mu_sel.cpu().data))
    condition_list.append(float(expert_condition.cpu().data))

    print(max_idx)
    #cv2.imshow('img',expert_state.cpu()[0].permute(1,2,0).numpy())
    #cv2.waitKey(0)

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

action_list = np.array(action_list)
condition_list = np.array(condition_list)
label = np.array(dataset.y)

matplotlib.use('tkagg')
plt.subplot(3,1,1)
plt.title("straight")
plt.hist(label[condition_list==0], bins=50, color='b')
plt.hist(action_list[condition_list==0], bins=50, alpha=0.5,color='c')
plt.subplot(3,1,2)
plt.title("left")
plt.hist(label[condition_list==1], bins=50, color='b')
plt.hist(action_list[condition_list==1], bins=50, alpha=0.5,color='m')
plt.subplot(3,1,3)
plt.title("right")
plt.hist(label[condition_list==2], bins=50, color='b')
plt.hist(action_list[condition_list==2], bins=50, alpha=0.5,color='y')
plt.show()
