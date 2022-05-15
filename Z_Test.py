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

FF = FeedForward(SIM)

PID = PID_Controller(SIM,2,60,10,0.6,0,100,True,False)
Delay_SP = Delay(SIM)

G.simList = [
    #Delay_SP.Delay_RT(SP.Signal,80),
    FF.FF_RT(P,D,'EBD'),
    PID.PID_RT(SP.Signal,DV.Signal,MAN.Signal,MAN_VAL.Signal,FF.MV,'EBD-EBD'),
    P.FO_RT(PID.MV,'EBD'),
    D.FO_RT(DV.Signal,'EBD'),
    #LL.LeadLag_RT(P.MV,'EBD'),
    
]


SP = Signal(SP.Signal,'Set Point','-m')
PIDMV = Signal(PID.MV,'MVFB','-c')
DS = Signal(D.MV,'Disturabnce','-k')
MVFF = Signal(FF.MV,'Feed Forward','-r')
PV = Signal(np.add(P.MV,D.MV),'Point Value','-g')
FO_MV = Signal(P.MV,'First Order','-y')
MAN = Signal(MAN.Signal,'Manual Mode','-b')
MAN_VAL = Signal(MAN_VAL.Signal,'Manual Mode value','-b')

G.show([PV,FO_MV,DS,SP,MAN_VAL,PIDMV],[MAN])