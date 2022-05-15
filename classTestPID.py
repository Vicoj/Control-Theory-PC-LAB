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
SP = Path(SIM,{0: 50,2000: 0, SIM.TSim: 0})
DV = Path(SIM,{0: 50, 1500: 80, SIM.TSim: 80} )
MAN = Path(SIM,{0: 0,80:0,800:0, SIM.TSim: 0})

#Delay 
Delay1 = Delay(SIM,100)

# FO Process
P = FirstOrder(SIM,0.654997667761135,141.9367358894029,6.678212203596281,50)
D = FirstOrder(SIM,0.065,500,7,50)

# Lead-Lag
LL = LeadLag(SIM,1,200,10)

# Feed Forward
FF = FeedForward(SIM,P,D)

#PID
PID = PID_Controller(SIM,0.6,5,2,0.7,0,100,False,False)

t = []
for ti in SIM.t:
    t.append(ti)

    #Signals
    SP.RT(t)
    DV.RT(t)
    MAN.RT(t)

    FF.RT(DV.Signal) # FeedForward
    PID.RT(SP.Signal,SIM.PV,MAN.Signal,[80],FF.MVFF,'EBD-EBD')
    

    SIM.MV.append(PID.MVFB[-1]+ FF.MVFF[-1]) # Modified Value

    P.RT(SIM.MV,'EBD')
    D.RT(DV.Signal,'EBD')

    SIM.PV.append(P.PV[-1] + D.PV[-1]) # Point Value
    




SigVals = [
    Signal(SP.Signal,'Set Point','-r'),
    Signal(DV.Signal,'Disturbance Value','-k'),
    Signal(SIM.PV,'Point Value','-b'),

    Signal(P.PV,'P(s)','--b'),
    Signal(D.PV,'P(s)','--k'),

    Signal(FF.MVFF,'MVFF','-k'),
    Signal(PID.MVFB,'MVFB','-y'),
    #Signal(PID.E,'Erreur',':r'),
    #Signal(PID.MVP,'MVP',':b'),
    #Signal(PID.MVI,'MVI',':y'),
    #Signal(PID.MVD,'MVD',':m'),

]

SigValsBin = [
    Signal(MAN.Signal,'Manual Mode','-g')
]

G.show(SigVals,SigValsBin)