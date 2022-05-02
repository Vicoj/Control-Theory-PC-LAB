import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons

import package_DBR
from package_DBR import *

import package_Advanced
from package_Advanced import *

TSim = 60 #Temps de la simulation
Ts = 0.1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
MVPath = {0: 0, 5: 40, 20:0, TSim: 45} # Chemin choisis

# FO Parametrers
#Final SSE Objective: 0.03787173811807361
Kp = 0.654997667761135
T = 141.9367358894029
theta = 6.678212203596281
PV = []

# PID Parametrers

Man = 0
MVMan = []
MVFF = []

Kc = 0.1
Ti = 1
Td = 1
alpha = 1

MVMin = -100
MVMax = 100

MV = []
MVP = []
MVI = []
MVD = []
E = []

ManFF = 0
PVInit = 0

def plotValues(Kp,T,theta,Kc,Ti,Td,alpha,MVPath,TSim,Ts):
    
    N = int(TSim/Ts) + 1 # nombres de samples 

    # Variables a rempir
    t = []
    MVPID = []
    MVDelay = []
    SP_FO = []
    PV = []
    MVP = []
    MVI = []
    MVD = []
    E = []


    for i in range(0,N):
        t.append(i*Ts)
        SelectPath_RT(MVPath,t,PV)
        FO_RT(PV,Kp,T,Ts,SP_FO,PVInit=0,method='EBD')
        PID_RT(PV, SP_FO, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MVPID, MVP, MVI, MVD, E, ManFF=False, PVInit=0, method='EBD-EBD')
    
    return t,PV,MVPID



# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
t,PV,MVPID = plotValues(Kp,T,theta,Kc,Ti,Td,alpha,MVPath,TSim,Ts)
PID, = plt.plot(t,MVPID, lw=2,label='PID Control (EDB)')
default, = plt.plot(t,PV)

ax.set_xlabel('Time [s]')


# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.1, bottom=0.3)

###### Make a horizontal slider to control the Param #####
axKc = plt.axes([0.15, 0.05, 0.5, 0.03])
axTi = plt.axes([0.15, 0.1, 0.5, 0.03])
axTd = plt.axes([0.15, 0.15, 0.5, 0.03])
axalpha = plt.axes([0.15, 0.2, 0.5, 0.03])


Kc_slider = Slider(ax=axKc,label='Kc',valmin=0.1,valmax=2,valinit=Kc)
Ti_slider = Slider(ax=axTi,label='Ti',valmin=0.1,valmax=2,valinit=Ti)
Td_slider = Slider(ax=axTd,label='Td',valmin=0.1,valmax=2,valinit=Td)
alpha_slider = Slider(ax=axalpha,label='alpha',valmin=0,valmax=2,valinit=alpha)


# The function to be called anytime a slider's value changes
def update(val):
    t,PV,MVPID = plotValues(Kp,T,theta,Kc_slider.val,Ti_slider.val,Td_slider.val,alpha_slider.val,MVPath,TSim,Ts)
    PID.set_ydata(MVPID)
    fig.canvas.draw_idle()
    ax.set_ylim(0,max(max(PV),max(MVPID))+0.1)

# register the update function with each slider
Kc_slider.on_changed(update)
Ti_slider.on_changed(update)
Td_slider.on_changed(update)
alpha_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    Kc_slider.reset()
    Ti_slider.reset()
    Td_slider.reset()
    alpha_slider.reset()
button.on_clicked(reset)

#Full screen
#manager = plt.get_current_fig_manager()
#manager.full_screen_toggle()

plt.show()