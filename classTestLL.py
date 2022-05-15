from signal import signal
from tokenize import Single
import numpy as np
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

# Graph Instance
G = Graph(SIM,'PID Control')

# Path
SP = Path(SIM,{0: 0, 5: 60 ,1000: 0, SIM.TSim: 0})
DV = Path(SIM,{0: 0, 5: 0, 1500:0, SIM.TSim: 0} )
MAN = Path(SIM,{0: 0,80:1,800:0, SIM.TSim: 0})

#Delay 
Delay1 = Delay(SIM,100)

# FO Process
P = FirstOrder(SIM,0.654997667761135,141.9367358894029,6.678212203596281,50)

# Lead-Lag

LL = LeadLag(SIM,1,200,10)

t = []
for ti in SIM.t:
    t.append(ti)

    SP.RT(t)
    MAN.RT(t)


    P.RT(SP.Signal,'EBD')
    Delay1.RT(P.PV)

    LL.RT(P.PV,'EBD')

SIM.PV = LL.PV


SPs = Signal(SP.Signal,'Set Point','-m')
MANs = Signal(MAN.Signal,'Manual Mode','-g')
P_PVs = Signal(P.PV,'Process FO','-y')
PVs = Signal(SIM.PV,'Point Value','-g')
DelayFOs = Signal(Delay1.PV,'Delay FO','-b')

G.show([SPs,PVs,P_PVs,DelayFOs],[MANs])