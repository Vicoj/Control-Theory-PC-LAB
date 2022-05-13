from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output




class Simulation:
    def __init__(self,TSim,Ts,PVInit):
        self.TSim = TSim
        self.Ts = Ts
        self.N = int(TSim/Ts) + 1 
        self.PVInit = PVInit
        self.PV = []
        self.t = self.calc_t()

    def calc_t(self):
        t = []
        for i in range(0,self.N):
            t.append(i*self.Ts)

        return t

class Path:
    def __init__(self,S:Simulation,path):
        self.S = S
        self.path = path
        self.Signal = self.SelectPath_RT()

    def SelectPath_RT(self):
        signal = []
        for i in self.S.t:
            for timeKey in self.path:
                if(self.S.t[i-1] >= timeKey):
                    timeKeyPrevious = timeKey    
            
            value = self.path[timeKeyPrevious]
            signal.append(value)
        
        return signal

class FirstOrder:
    def __init__(self,S:Simulation,gain,Time,Theta,point_fct):
        self.S = S
        self.K = gain
        self.T = Time
        self.Theta = Theta
        self.point_fct = point_fct
        self.MV = []

    def FO_RT(self,input,method):
        MV = []
        for i in self.S.t:
            if (self.T != 0):
                KFO = self.S.Ts/self.T
                if len(self.MV) == 0:
                    self.MV.append(self.S.PVInit)
                else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
                    if method == 'EBD':
                        MV.append((1/(1+KFO))*MV[-1] + (KFO*self.K/(1+KFO))*input[-1])
                    elif method == 'EFD':
                        MV.append((1-KFO)*MV[-1] + KFO*self.K*input[-2])
                    elif method == 'TRAP':
                        MV.append((1/(2*self.T+self.S.Ts))*((2*self.T-self.S.Ts)*MV[-1] + self.K*self.S.Ts*(input[-1] + input[-2])))            
                    else:
                        MV.append((1/(1+KFO))*MV[-1] + (KFO*self.K/(1+KFO))*input[-1])
            else:
                MV.append(self.K*input[-1])
        self.MV = MV

class FeedForward:
    def __init__(self,P:FirstOrder ,D:FirstOrder ):
        self.P = P
        self.D = D
        self.T1p = P.T
        self.T2p = 1
        self.T1d = D.T
        self.T2d = 1

        self.MV = []
        self.MV_LL1 = []
        self.MV_LL2 = []
        self.MVFF_Delay = []

class PID_Controller:
    def __init__(self,Kc,Ti,Td,alpha,MVmin,MVmax,OLP):
            self.Kc = Kc
            self.Ti = Ti
            self.Td = Td
            self.alpha = alpha
            self.MVmin = MVmin
            self.MVmax = MVmax
            self.OLP = OLP

            self.MVMan = []

            self.MV = []
            self.MVP = []
            self.MVI = []
            self.MVD = []
            self.E = []

class Delay:
    def __init__(self,S:Simulation):
        self.S = S
        self.theta = 0
        self.MVInit = 0 # Par Default

        self.MV = []

    def Delay_RT(self,Input,theta):
        self.theta = theta
        input_delay = []

        for i in self.S.t:
            NDelay = int(np.ceil(theta/self.S.Ts))
            if NDelay > len(Input)-1:
                input_delay.append(self.MVInit)
            else:    
                input_delay.append(Input[-NDelay-1])

        self.MV = input_delay

class Graph:
    def __init__(self,S:Simulation):
        self.S = S
        self.fig, self.ax1 = plt.subplots()
        self.fig.set_figheight(10)
        self.fig.set_figwidth(15)

    def show(self,Signal,label,title):
        
        l1, = self.ax1.step(self.S.t,Signal,'g-',linewidth=2,label='label',where='post')
        self.ax1.set_ylabel(label)
        self.ax1.set_title(title)
        self.ax1.legend(loc='best')

        plt.show()