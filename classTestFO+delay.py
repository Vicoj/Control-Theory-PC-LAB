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
MAN = Path(SIM,{0: 0,80:1,800:0, SIM.TSim: 0})

#Delay 
Delay1 = Delay(SIM,100)

# FO Process
P = FirstOrder(SIM,0.654997667761135,141.9367358894029,6.678212203596281,50)

t = []
for ti in SIM.t:
    t.append(ti)

    SP.RT(t)
    MAN.RT(t)

    P.RT(SP.Signal,method='EBD')
    Delay1.RT(P.PV)

SIM.PV = Delay1.PV


SPs = Signal(SP.Signal,'Set Point','-m')
MANs = Signal(MAN.Signal,'Manual Mode','-g')
PVs = Signal(SIM.PV,'Point Value','-g')

G.show([SPs,PVs],[MANs])