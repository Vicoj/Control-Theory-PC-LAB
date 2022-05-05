import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches

import package_DBR
from package_DBR import *

import package_Advanced
from package_Advanced import *


TSim = 1000 #Temps de la simulation
Ts = 1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
#MVPath = {0: 0, 5: 40, 280:0, TSim: 55} # Chemin choisis
MVPath = {0: 0, 50: 1, 500: 2, 800: 3, TSim: 3} # Chemin choisis
DVPath = {0: 0, 400: 3, 500:0, TSim: 0} # Chemin choisis

# FO P Parametrers
#Final SSE Objective: 0.03787173811807361
Kp = 0.654997667761135
Tp = 141.9367358894029
ThetaP = 6.678212203596281

# FO D Parametres
Kd = 69809.39444564922
Td = 37604923.967538156
ThetaD = 18.000023725261713

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

t = []
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
    #Ps
    PV_P = []

    PV = []

    for i in range(0,N):

        
        t.append(i*Ts)
        SelectPath_RT(MVPath,t,SP)
        SelectPath_RT(DVPath,t,DV)


        #FF_RT(DV,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, MVFF)

        #D
        #FO_RT(DV,Kd,Td,Ts,PV_D,PVInit=0,method='EBD')
        #PID
        PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVP, MVI, MVD, E,OLP, ManFF=False, PVInit=0, method='EBD-EBD')
        #P
        FO_RT(MV,Kp,Tp,Ts,PV_P,PVInit=0,method='EBD')

        #PV = np.add(PV_P ,PV_D)
        PV = PV_P
    
    
    return t,SP,PV,MV,DV



# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
t,SP,PV,MV,DV = plotValues(Kp,Kc,Ti,Td,alpha,OLP)
PVg, = plt.plot(t,PV,color='b')
MVg, = plt.plot(t,MV,color='g')
SPg, = plt.plot(t,SP,color='r')
DVg, = plt.plot(t,DV,color='c')

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
    t,SP,PV,MV,DV = plotValues(Kp,Kc_slider.val,Ti_slider.val,Td_slider.val,alpha_slider.val,check.get_status()[0])
    PVg.set_ydata(PV)
    MVg.set_ydata(MV)

    PVg.set_visible(check.get_status()[1])
    MVg.set_visible(check.get_status()[2])
    SPg.set_visible(check.get_status()[3])
    DVg.set_visible(check.get_status()[4])

    fig.canvas.draw_idle()
    ming = min(min(MV),min(PV))
    maxg = max(max(MV),max(PV))
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
check = CheckButtons(rax, ['OLP','PV','MV','SP','DV'], [False,True,True,True,True])
check.on_clicked(update)

# Fuctions

def save(event):
    fig.savefig("Output/"+text_box.text)
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

ax.legend(handles=[PVl,MVl,SPl,DVl])

ax.grid()
ax.set_title(label='PID Simulation')
plt.show()