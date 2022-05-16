from signal import signal
from tokenize import Single
import numpy as np
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches
from datetime import datetime
import os
import tclab
import time


from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output

import package_Class
from package_Class import Simulation,Path,FirstOrder,SecondOrderPlusDelay,LeadLag,FeedForward,PID_Controller,Delay,Signal,Variable,Graph,LabValues

#Simulation Instance
SIM = Simulation(200,1,0)

#Tc Lab
LAB = tclab.TCLab()
LABVal = LabValues(SIM,LAB)

# Graph Instance
G = Graph(SIM,'PID Control')

# Path
SP = Path(SIM,{0: 0,50: 80, SIM.TSim: 80})
DV = Path(SIM,{0: 0, 1500: 0, SIM.TSim: 0} )
MAN = Path(SIM,{0: 0,1000:0,1500:0, SIM.TSim: 0})

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
PID = PID_Controller(SIM,1.69,141,10,0.7,0,100,False,False)

SIM.t = []
start = time.time()
delta = 0
totalTime = 0
last = time.time()

while totalTime < SIM.TSim:
    if delta > SIM.Ts:
        last = time.time()
        SIM.t.append(totalTime)
        #Signals
        SP.RT(SIM.t)
        DV.RT(SIM.t)
        MAN.RT(SIM.t)

        FF.RT(DV.Signal) # FeedForward
        PID.RT(SP.Signal,SIM.PV,MAN.Signal,[80],FF.MVFF,'EBD-EBD')
        SIM.MV.append(PID.MVFB[-1]+ FF.MVFF[-1]) # Modified Value
        LABVal.RT(SIM.MV,DV.Signal)
        delta = 0
        print(totalTime)

    else :
        totalTime = time.time() - start
        delta = time.time() - last
        


LAB.close()

SigVals1 = [
    Signal(SP.Signal,'SP','-r'),
    Signal(SIM.PV,'PV','-b'),
    #Signal(P.PV,'P(s)','--b'),
    #Signal(D.PV,'D(s)','--k'),

]
SigVals2 = [
    Signal(SIM.MV,'MV','-b'),
    Signal(FF.MVFF,'MVFF','-k'),
    Signal(PID.MVFB,'MVFB','-y'),
    #Signal(PID.E,'E',':r'),
    #Signal(PID.MVP,'MVP',':b'),
    #Signal(PID.MVI,'MVI',':y'),
    #Signal(PID.MVD,'MVD',':m'),
    #Signal(DV.Signal,'DV','-k'),

]
SigValsBin = [
    Signal(MAN.Signal,'Manual Mode','-g')
]
varVals = [
    Variable(PID.Kc,'Gain PID'),
    Variable(PID.Td,'Td PID'),
    Variable(PID.Ti,'Ti PID'),
]

G.show([SigVals1,SigVals2],SigValsBin,varVals)
#G.Bode(D,True)