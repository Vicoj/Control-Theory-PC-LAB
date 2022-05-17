from signal import signal
from tokenize import Single
import numpy as np
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches
import datetime
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
SIM = Simulation(1500,1,30,False)

# Graph Instance
G = Graph(SIM,'PID Control')

# Path
SP = Path(SIM,{0: 0,10: 40,800: 60, SIM.TSim: 60})
DV = Path(SIM,{0: 50, 600: 80, SIM.TSim: 80} )
MAN = Path(SIM,{0: 0,1000:0,1500:0, SIM.TSim: 0})
MANVal = Path(SIM,{0: 80, SIM.TSim: 80})

#Delay 
Delay1 = Delay(SIM,100)

# FO Process
P = FirstOrder(SIM,0.6522434279003099,245.9823790885576,0.649693920059717,50,SIM.PVInit)
D = FirstOrder(SIM,0.6156105636473335,387.0591022229922, 5.419428855220769,50,0)

# Lead-Lag
LL = LeadLag(SIM,1,200,10)

# Feed Forward
FF = FeedForward(SIM,P,D,1)

#PID
PID = PID_Controller(SIM,1.69,141,5,2,0,100,False,False)
PID.IMC_tuning(P,0.4,'H')



if(SIM.sim == True):
    t = []
    for ti in SIM.t:
        t.append(ti)
        #Signals
        SP.RT(t)
        DV.RT(t)
        MAN.RT(t)
        MANVal.RT(t)

        FF.RT(DV.Signal) # FeedForward
        PID.RT(SP.Signal,SIM.PV,MAN.Signal,MANVal,FF.MVFF,'EBD-EBD')
        SIM.MV.append(PID.MVFB[-1]+ FF.MVFF[-1]) # Modified Value
        P.RT(SIM.MV,'EBD')
        D.RT(DV.Signal,'EBD')
        SIM.PV.append(P.PV[-1]+D.PV[-1]) # Point Value
        SIM.updateBar()

if((SIM.sim == False)):
    #Tc Lab
    LAB = tclab.TCLab()
    LABVal = LabValues(SIM,LAB)

    SIM.t = []
    start = time.time()
    delta = 0
    totalTime = 0
    last = time.time()

    while totalTime < SIM.TSim:
        if delta > SIM.Ts:
            last = time.time()
            SIM.t.append(round(totalTime,4))
            #Signals
            SP.RT(SIM.t)
            DV.RT(SIM.t)
            MAN.RT(SIM.t)
            MANVal.RT(SIM.t)

            FF.RT(DV.Signal) # FeedForward
            PID.RT(SP.Signal,SIM.PV,MAN.Signal,MANVal,FF.MVFF,'EBD-EBD')
            SIM.MV.append(PID.MVFB[-1]+ FF.MVFF[-1]) # Modified Value
            LABVal.RT(SIM.MV,DV.Signal,D.point_fct)
            delta = 0
            SIM.updateBar()

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
    Signal(DV.Signal,'DV','-k'),
    #Signal(FF.MVFF,'MVFF','-k'),
    #Signal(PID.MVFB,'MVFB','-y'),
    ##Signal(PID.E,'E',':r'),
    #Signal(PID.MVP,'MVP',':b'),
    #Signal(PID.MVI,'MVI',':y'),
    #Signal(PID.MVD,'MVD',':m'),
    #Signal(DV.Signal,'DV','-k'),

]
SigValsBin = [
    Signal(MAN.Signal,'Manual Mode','-g')
]
varVals = [
    Variable(SIM.TSim,'Temps Sim [s]'),
    Variable(SIM.Ts,'Sampling [s]'),
    Variable(SIM.PVInit,'Pv Init [°C]'),

    Variable(PID.Kc,'Kc PID'),
    Variable(PID.Td,'Td PID'),
    Variable(PID.Ti,'Ti PID'),
]

G.show([SigVals1,SigVals2],SigValsBin,varVals)
#G.Bode(D,True)