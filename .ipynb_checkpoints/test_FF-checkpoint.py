import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode

import package_Advanced
from package_Advanced import *


TSim = 1000 #Temps de la simulation
Ts = 1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
#MVPath = {0: 0, 5: 40, 280:0, TSim: 55} # Chemin choisis
MVPath = {0: 20, 500: 20, TSim: 20} # Chemin choisis
DVPath = {0: 0, 5: 30, 900:0, TSim: 0} # Chemin choisis

# FO P Parametrers
#Final SSE Objective: 0.03787173811807361
Kp = 0.654997667761135
Tp = 141.9367358894029
ThetaP = 6.678212203596281

# FO D Parametres
Kd = 0.06
Td = 200
ThetaD = 6.678212203596281

# FF Parametres
T1p = 1
T1d = 1
T2p = 1
T2d = 1

# PID Parametrers

Man = 0
MVMan = []
MVFF = []

Kc = 1
Ti = 60
Td = 60
alpha = 1

MVMin = 0
MVMax = 100

OLP = False
  
#FF
MVFF = []
#PID
MV = []
MVP = []
MVI = []
MVD = []
E = []
#Ds
PV_D = []
#Ps
PV_P = []

PV = []
SP = []
DV = []
t= []


for i in range(0,N):
    PVInit = 10
    ManFF=False
    t.append(i*Ts)
    SelectPath_RT(MVPath,t,SP)
    SelectPath_RT(DVPath,t,DV)
    FF_RT(DV,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, MVFF)
    #D
    FO_RT(DV,Kd,Td,Ts,PV_D,PVInit,method='EBD')
    #PID
    PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVP, MVI, MVD, E,OLP, ManFF,PVInit, method='EBD-EBD')
    #P
    FO_RT(MV,Kp,Tp,Ts,PV_P,PVInit,method='EBD')
    
    PV.append(PV_P[-1]+PV_D[-1])

plt.figure(figsize = (15,12))

plt.subplot(2,1,1)
plt.step(t,SP,'b-',label='SP',where='post')
plt.step(t,DV,'b-',linewidth=0.5,label='MV plus delay',where='post')
plt.ylabel('Value of PV')
plt.title('PV signal')
plt.legend(loc='best')
plt.xlim([0, TSim])

plt.subplot(2,1,2)
plt.step(t,MV,'g-',label='MV signal',where='post')
plt.step(t,MVFF,'lime',label='MVFF',where='post')
plt.step(t,PV,'yellow',label='PV',where='post',linewidth=0.5)
plt.ylabel('Value of MV')
plt.xlabel('Time [s]')
plt.legend(loc='best')
plt.xlim([0, TSim])
plt.show()