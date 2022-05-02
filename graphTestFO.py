import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode

import package_Advanced
from package_Advanced import LeadLag_RT , PID_RT

TSim = 540 #Temps de la simulation
Ts = 0.1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
MVPath = {0: 0, 5: 40, 280:0, TSim: 45} # Chemin choisis

# FO Parametrers
#Final SSE Objective: 0.03787173811807361
Kp = 0.654997667761135
T = 141.9367358894029
theta = 6.678212203596281
PV = []

def plotValues(Kp,T,theta,MVPath,TSim,Ts):
    
    N = int(TSim/Ts) + 1 # nombres de samples 

    # Variables a rempir
    t = []
    MV = []
    MVDelay = []
    PV_FO = []


    for i in range(0,N):
        t.append(i*Ts)
        SelectPath_RT(MVPath,t,MV)
        Delay_RT(MV,theta,Ts,MVDelay) 
        FO_RT(MVDelay,Kp,T,Ts,PV_FO,PVInit=0,method='EBD')
    
    return t,MV,PV_FO



# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
t,MV,PV_FO = plotValues(Kp,T,theta,MVPath,TSim,Ts)
FO, = plt.plot(t,PV_FO, lw=2,label='FO Responce (EDB)')
default, = plt.plot(t,MV)

ax.set_xlabel('Time [s]')


# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.15, bottom=0.3)

###### Make a horizontal slider to control the Param #####
axKp = plt.axes([0.05, 0.25, 0.0225, 0.63])
axT = plt.axes([0.15, 0.1, 0.5, 0.03])
axtheta = plt.axes([0.15, 0.15, 0.5, 0.03])


Kp_slider = Slider(ax=axKp,label='Kp',valmin=0,valmax=10,valinit=Kp,orientation="vertical")
T_slider = Slider(ax=axT,label='T',valmin=0,valmax=TSim,valinit=T)
theta_slider = Slider(ax=axtheta,label='theta',valmin=0,valmax=TSim,valinit=theta)


# The function to be called anytime a slider's value changes
def update(val):
    t,MV,PV_FO = plotValues(Kp_slider.val,T_slider.val,theta_slider.val,MVPath,TSim,Ts)
    FO.set_ydata(PV_FO)
    fig.canvas.draw_idle()
    ax.set_ylim(0,max(max(MV),max(PV_FO))+0.1)

# register the update function with each slider
Kp_slider.on_changed(update)
T_slider.on_changed(update)
theta_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    Kp_slider.reset()
    T_slider.reset()
    theta_slider.reset()
button.on_clicked(reset)

#Full screen
#manager = plt.get_current_fig_manager()
#manager.full_screen_toggle()

plt.show()