import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches
from datetime import datetime
import os

import package_DBR
from package_DBR import *

import package_Advanced
from package_Advanced import *


TSim = 2000 #Temps de la simulation
Ts = 1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
#MVPath = {0: 0, 5: 40, 280:0, TSim: 55} # Chemin choisis
MVPath = {0: 0, 5: 60 ,1000: 60, TSim: 60} # Chemin choisis
DVPath = {0: 0, 5: 50, 1500:60, TSim: 60} # Chemin choisis

# FO P Parametrers
#Final SSE Objective: 0.03787173811807361
Kp = 0.654997667761135
Tp = 141.9367358894029
ThetaP = 6.678212203596281

PV0 = 50
MV0 = 50

# FO D Parametres
Kd = 0.06
Td = 200
ThetaD = 6.678212203596281
MVDelayD = 1

DV0 = 50

# FF Parametres
T1p = Tp
T1d = Td
T2p = 10
T2d = 10

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

SP = []
DV = []
  
def plotValues(Kp,Kc,Ti,Td,alpha,OLP):
    N = int(TSim/Ts) + 1 # nombres de samples 
    t = []
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
    MV_LL1 = []
    MV_LL2 = []
    MVFF_Delay = []
    MVDelayD = []

    #Ps
    PV_P = []

    PV = []
    SP = []
    DV = []
    
    
    for i in range(0,N):

        PVInit = 0
        ManFF=False
        t.append(i*Ts)
        SelectPath_RT(MVPath,t,SP)
        SelectPath_RT(DVPath,t,DV)

        # Feed Forward
        FF_RT(DV,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, DV0,PVInit,MVFF_Delay,MV_LL1,MV_LL2 , MVFF)

        #PID
        PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVP, MVI, MVD, E,OLP, ManFF,PVInit = 0, method='EBD-EBD')

        #P(s) Processus
        FO_RT(MV,Kp,Tp,Ts,PV_P,PVInit,method='EBD')

        #D(s) Disturbance
        Delay_RT(DV - DV0*np.ones_like(DV),ThetaD,Ts,MVDelayD,0)
        FO_RT(MVDelayD,Kd,Td,Ts,PV_D,PVInit,method='EBD')
        
        

        PV.append(PV_P[-1]+PV_D[-1] + PV0-Kp*MV0)
    
    
    return t,SP,PV,MV,DV,MVFF,[MVP,MVI,MVD]



# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
t,SP,PV,MV,DV,MVFF,PID = plotValues(Kp,Kc,Ti,Td,alpha,OLP)
PVg, = plt.plot(t,PV,color='b')
MVg, = plt.plot(t,MV,color='g')
SPg, = plt.plot(t,SP,color='r')
DVg, = plt.plot(t,DV,color='c')
MVFFg, = plt.plot(t,MVFF,color='k')
MVPg, = plt.plot(t,PID[0],color='#71FF33')
MVIg, = plt.plot(t,PID[1],color='#4F8C35')
MVDg, = plt.plot(t,PID[2],color='#36920F')

ax.set_xlabel('Time [s]')


# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.1, bottom=0.3)

###### Make a horizontal slider to control the Param #####
axKc = plt.axes([0.1, 0.05, 0.5, 0.03])
axTi = plt.axes([0.1, 0.1, 0.5, 0.03])
axTd = plt.axes([0.1, 0.15, 0.5, 0.03])
axalpha = plt.axes([0.1, 0.2, 0.5, 0.03])
 
Kc_slider = Slider(ax=axKc,label='Kc',valmin=0.1,valmax=20,valinit=Kc)
Ti_slider = Slider(ax=axTi,label='Ti',valmin=0.1,valmax=TSim,valinit=Ti)
Td_slider = Slider(ax=axTd,label='Td',valmin=0.1,valmax=TSim,valinit=Td)
alpha_slider = Slider(ax=axalpha,label='alpha',valmin=0.1,valmax=20,valinit=alpha)


# The function to be called anytime a slider's value changes
def update(val):
    t,SP,PV,MV,DV,MVFF,PID = plotValues(Kp,Kc_slider.val,Ti_slider.val,Td_slider.val,alpha_slider.val,check.get_status()[0])
    PVg.set_ydata(PV)
    MVg.set_ydata(MV)
    MVPg.set_ydata(PID[0])
    MVIg.set_ydata(PID[1])
    MVDg.set_ydata(PID[2])

    PVg.set_visible(check.get_status()[1])
    MVg.set_visible(check.get_status()[2])
    SPg.set_visible(check.get_status()[3])
    DVg.set_visible(check.get_status()[4])
    MVFFg.set_visible(check.get_status()[5])
    MVPg.set_visible(check.get_status()[6])
    MVIg.set_visible(check.get_status()[6])
    MVDg.set_visible(check.get_status()[6])

    fig.canvas.draw_idle()
    ming = min(min(MV),min(MVFF))
    maxg = MVMax
    ax.set_ylim(ming + ming/10,maxg + maxg/10)

# register the update function with each slider
Kc_slider.on_changed(update)
Ti_slider.on_changed(update)
Td_slider.on_changed(update)
alpha_slider.on_changed(update)

# Buttons
saveax = plt.axes([0.9, 0.15, 0.05, 0.04])
button_save = Button(saveax, 'Save', hovercolor='0.975')

namebox = plt.axes([0.75, 0.15, 0.15, 0.04])
text_box = TextBox(namebox, 'NAME :', initial='')

resetax = plt.axes([0.8, 0.1, 0.1, 0.04])
button_reset = Button(resetax, 'Default Values', hovercolor='0.975')
closefig = plt.axes([0.8, 0.05, 0.1, 0.04])
button_close = Button(closefig, 'Close', hovercolor='0.975')

# Check Box

rax = plt.axes([0.025, 0.75, 0.05, 0.1]) # gauche haut droit bas (relatif)
check = CheckButtons(rax, ['OLP','PV','MV','SP','DV','MVFF','MVFB'], [False,True,True,True,True,True,True])
check.on_clicked(update)

# Fuctions

def save(event):
    t,SP,PV,MV,DV,MVFF = plotValues(Kp,Kc,Ti,Td,alpha,OLP)
    fig.savefig("Output/"+text_box.text)
    now = datetime.now()
    date_time = now.strftime("%Y-%m-%d-%Hh%M")
    t = np.array(t)
    MV = np.array(MV)
    PV = np.array(PV)
    DV = np.array(DV)
    my_data = np.vstack((t.T,MV.T,PV.T,DV.T))
    my_data = my_data.T
    nameFile = 'Data/PID_Graph_' + text_box.text + '_' + date_time + '.txt'
    if not os.path.exists('Data'):
        os.makedirs('Data')
    np.savetxt(nameFile,my_data,delimiter=',',header='t,MV,PV,DV',comments='')
                   
button_save.on_clicked(save)

def reset(event):
    Kc_slider.reset()
    Ti_slider.reset()
    Td_slider.reset()
    alpha_slider.reset()
button_reset.on_clicked(reset)

def close(event):
    plt.close()
button_close.on_clicked(close)


#Full screen
manager = plt.get_current_fig_manager()
manager.full_screen_toggle()

# Creating legend with color box
PVl = mpatches.Patch(color='b', label='PV')
MVl = mpatches.Patch(color='g', label='MV')
SPl = mpatches.Patch(color='r', label='SP')
DVl = mpatches.Patch(color='c', label='DV')
MVFFl = mpatches.Patch(color='k', label='MVFF')
MVPl = mpatches.Patch(color='#71FF33', label='MVP')
MVIl = mpatches.Patch(color='#4F8C35', label='MVI')
MVDl = mpatches.Patch(color='#36920F', label='MVD')


ax.legend(handles=[PVl,MVl,SPl,DVl,MVFFl,MVPl,MVIl,MVDl])

ax.grid()
ax.set_title(label='PID Simulation')
plt.show()