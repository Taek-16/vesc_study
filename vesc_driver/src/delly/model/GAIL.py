import torch
import torch.nn as nn
import torch.nn.functional as F

class MixtureOfLogits(nn.Module):
    def __init__(self,
                 in_dim     = 64,   # input feature dimension
                 y_dim      = 10,   # number of classes
                 k          = 5,    # number of mixtures
                 sig_min    = 1e-4, # minimum sigma
                 sig_max    = None, # maximum sigma
                 SHARE_SIG  = True  # share sigma among mixture
                 ):
        super(MixtureOfLogits,self).__init__()
        self.in_dim     = in_dim    # Q
        self.y_dim      = y_dim     # D
        self.k          = k         # K
        self.sig_min    = sig_min
        self.sig_max    = sig_max
        self.SHARE_SIG  = SHARE_SIG
        self.build_graph()

    def build_graph(self):
        self.h_dim = 256
        self.fc_pi      = nn.Sequential(
                                        nn.Linear(self.in_dim,self.h_dim),
                                        nn.ReLU(inplace=True),

                                        nn.Linear(self.h_dim,self.k))

        self.fc_mu      = nn.Sequential(
                                        nn.Linear(self.in_dim,self.h_dim),
                                        nn.ReLU(inplace=True),
                                        #nn.BatchNorm1d(self.h_dim),
                                        nn.Linear(self.h_dim,self.k*self.y_dim))
        if self.SHARE_SIG:
            self.fc_sigma   = nn.Sequential(
                                        nn.Linear(self.in_dim,self.h_dim),
                                        nn.ReLU(inplace=True),
                                        #nn.BatchNorm1d(self.h_dim),
                                        nn.Linear(self.h_dim,self.k))
        else:
            self.fc_sigma   = nn.Sequential(
                                        nn.Linear(self.in_dim,self.h_dim),
                                        nn.ReLU(inplace=True),
                                        #nn.BatchNorm1d(self.h_dim),
                                        nn.Linear(self.h_dim,self.k))

        self.init_param()

    def init_param(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight)

        self.fc_mu[0].bias.data.uniform_(0,1)
        self.fc_mu[-1].bias.data.uniform_(0,1)

    def forward(self,x):
        """
            :param x: [N x Q]
        """
        pi_logit        = self.fc_pi(x)                                 # [N x K]
        pi              = torch.softmax(pi_logit,dim=1)                 # [N x K]
        mu              = self.fc_mu(x)                                 # [N x KD]
        mu              = torch.reshape(mu,(-1,self.k,self.y_dim))      # [N x K x D]
        mu              = torch.clamp(mu, min=0, max=1)
        #mu              = F.sigmoid(mu)
        #mu              = F.softmax(mu, dim=-1)
        if self.SHARE_SIG:
            sigma       = self.fc_sigma(x)                              # [N x K]
            sigma       = sigma.unsqueeze(dim=-1)                       # [N x K x 1]
            sigma       = sigma.expand_as(mu)                           # [N x K x D]
        else:
            sigma       = self.fc_sigma(x)                              # [N x KD]
        sigma           = torch.reshape(sigma,(-1,self.k,self.y_dim))   # [N x K x D]
        if self.sig_max is None:
            sigma = self.sig_min + torch.exp(sigma)                     # [N x K x D]
        else:
            sig_range = (self.sig_max-self.sig_min)
            sigma = self.sig_min + sig_range*torch.sigmoid(sigma)       # [N x K x D]
        mol_out = {'pi':pi,'mu':mu,'sigma':sigma}
        return mol_out

class Actor(nn.Module):
    def __init__(self):
        super(Actor, self).__init__()

        self.feat= nn.Sequential(
            nn.Conv2d(3,32,5,2),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32,64,3,2),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.Conv2d(64,128,3,2),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Conv2d(128,256,3,1),
            nn.BatchNorm2d(256),
            nn.Conv2d(256,256,3,2),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.Flatten())
        self.fc = nn.Sequential(
            nn.Linear(52224,1000),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(1000,512),
            nn.ReLU())

        self.command = list()
        for i in range(3):
            self.command.append(MixtureOfLogits(
                in_dim      = 512,
                y_dim       = 1,
                k           = 3,
                sig_min     = 1e-4,
                sig_max     = 1e-2,
                SHARE_SIG   = True
            ))
            """
            self.command.append(
                nn.Sequential(
                    nn.Linear(512,256),
                    nn.ReLU(),
                    nn.Dropout(0.2),
                    nn.Linear(256,1),
                    nn.Sigmoid())
            )
            """

        self.command = nn.ModuleList(self.command)


    def forward(self, x, cmd):
        feat    = self.feat(x)
        fc      = self.fc(feat) # [B x 512]
        """
        action  = list()
        for idx,c in enumerate(cmd):
            out = self.command[c](fc[idx].unsqueeze(0))
            action.append(out)
        action = torch.cat(action, dim=0)
        return action
        """
        pi      = list()
        mu      = list()
        sigma   = list()
        for idx,c in enumerate(cmd):
            mln_out = self.command[c](fc[idx].unsqueeze(0))
            pi.append(mln_out['pi'])
            mu.append(mln_out['mu'])
            sigma.append(mln_out['sigma'])
        mu = torch.cat(mu, dim=0)
        sigma = torch.cat(sigma, dim=0)
        pi = torch.cat(pi, dim=0)
        return mu, sigma, pi


class ActorMDN(nn.Module):
    def __init__(self):
        super(ActorMDN, self).__init__()

        self.feat= nn.Sequential(
            nn.Conv2d(3,32,5,2),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32,64,3,2),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.Conv2d(64,128,3,2),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Conv2d(128,256,3,1),
            nn.BatchNorm2d(256),
            nn.Conv2d(256,256,3,2),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.Flatten())
        self.fc = nn.Sequential(
            nn.Linear(52224,1000),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(1000,512),
            nn.ReLU())

        self.command = MixtureOfLogits(
                in_dim      = 512*2,
                y_dim       = 1,
                k           = 3,
                sig_min     = 1e-4,
                sig_max     = 0.1,
                SHARE_SIG   = True
        )


    def forward(self, x, cmd):
        bs = cmd.size(0)
        feat    = self.feat(x)
        fc      = self.fc(feat) # [B x 512]

        cmd = cmd.view(bs,1)
        cmd_exp = cmd.expand_as(fc).float()

        feat_cmd_cat = torch.cat([fc,cmd_exp], axis=-1)

        mln_out = self.command(feat_cmd_cat)

        return mln_out['pi'], mln_out['mu'], mln_out['sigma']


class Discriminator(nn.Module):
    def __init__(self):
        super(Discriminator, self).__init__()

        self.feat= nn.Sequential(
            nn.Conv2d(3,32,5,2),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32,128,3,2),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.Conv2d(128,256,3,2),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.Conv2d(256,256,3,1),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.Conv2d(256,256,3,2),
            nn.BatchNorm2d(256),
            nn.ReLU(),
            nn.Flatten())

        self.fc = nn.Sequential(
            nn.Linear(52224,1024),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(1024,512),
            nn.ReLU())

        self.fc2 = nn.Sequential(
            nn.Linear(512*3,256),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(256,1))

    def forward(self, x, cmd, action):
        bs      = x.size(0)
        feat    = self.feat(x)
        fc      = self.fc(feat) # [B x 512]
        cmd_exp = cmd.view(bs,1).expand_as(fc).float()
        act_exp = action.view(bs,1).expand_as(fc).float()
        fc_cat  = torch.cat([fc, cmd_exp, act_exp], dim=-1)
        return torch.sigmoid(self.fc2(fc_cat))

if __name__=="__main__":
    x = torch.randn(4,3,480//2, 640//2)

    actor = Actor()
    disc  = Discriminator()

    out  = actor(x, 0 )
    out2 = disc(x, 0, 0.45)
    print(" Actor :: \n\tpi    : {}\n\tmu    : {}\n\tsigma : {}".format(out['pi'].shape,out['mu'].shape,out['sigma'].shape))
    #print(" Actor :: ",out.shape)

    print(" Discriminator :: ",out2.shape)
