from signal import signal
from tokenize import Single
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches
from datetime import datetime
import os

from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output

import package_Advanced
from package_Advanced import *

import package_Class
from package_Class import *


#Simulation Instance
SIM = Simulation(3000,1,0)
G = Graph(SIM,'PID Control')

# Path for Every Signal
SP = Path(SIM,{0: 0, 5: 60 ,1000: 0, SIM.TSim: 0})
DV = Path(SIM,{0: 0, 500: 50, 1500:0, SIM.TSim: 0} )
MAN = Path(SIM, {0: 0, 800:1, 900:0,SIM.TSim: 0})
MAN_VAL = Path(SIM, {0: 80,SIM.TSim: 80})

# FO Process
P = FirstOrder(SIM,0.654997667761135,141.9367358894029,6.678212203596281,50)

# FO Disturbance
D = FirstOrder(SIM,0.0654997667761135,1114.9367358894029,6.678212203596281,50)

LL = LeadLag(SIM,10,10,10)

FF = FeedForward(SIM,P,D)

PID = PID_Controller(SIM,2,50,10,0.5,0,100,False,True)
Delay_SP = Delay(SIM)

t = []
for ti in SIM.t:
    t.append(ti)
    SelectPath_RT(SP.path,t,SP.Signal)
    SelectPath_RT(DV.path,t,DV.Signal)
    SelectPath_RT(MAN.path,t,MAN.Signal)
    # Feed Forward
    FF_RT(DV.Signal, D.K, P.K, FF.T1p, FF.T1d, FF.T2p, FF.T2d , FF.ThetaD, FF.ThetaP, SIM.Ts, D.point_fct,SIM.PVInit,FF.MVFF_Delay,FF.MV_LL1,FF.MV_LL2 , FF.MV)
    #PID
    PID_RT(SP.Signal, SIM.PV, MAN.Signal, PID.MVMan, FF.MV, PID.Kc, PID.Ti, PID.Td, PID.alpha, SIM.Ts, PID.MVMin, PID.MVMax, PID.MV, PID.MVP, PID.MVI, PID.MVD, PID.E,PID.OLP, PID.ManFF,SIM.PVInit, method='EBD-EBD')
    #P(s) Processus
    FO_RT(PID.MV,P.K,P.T,SIM.Ts,P.MV,SIM.PVInit,method='EBD')
    #D(s) Disturbance
    #Delay_RT(DV.Signal - D.point_fct*np.ones_like(DV.Signal),D.Theta,SIM.Ts,1)
    FO_RT(DV.Signal,D.K,D.T,SIM.Ts,D.MV,SIM.PVInit,method='EBD')
    
    
    SIM.PV.append(P.MV[-1]+D.MV[-1] + P.point_fct-P.K*50)
    

SP = Signal(SP.Signal,'Set Point','-m')
PIDMV = Signal(PID.MV,'MVFB','-c')
DS = Signal(D.MV,'Disturabnce','-k')
MVFF = Signal(FF.MV,'Feed Forward','-r')
PV = Signal(np.add(P.MV,D.MV),'Point Value','-g')
FO_MV = Signal(P.MV,'First Order','-y')
MAN = Signal(MAN.Signal,'Manual Mode','-b')
#MAN_VAL = Signal(MAN_VAL.Signal,'Manual Mode value','-b')

G.show([PV,FO_MV,DS,SP,PIDMV],[MAN])