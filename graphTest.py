import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode

import package_Advanced
from package_Advanced import LeadLag_RT , PID_RT

TSim = 100 #Temps de la simulation
Ts = 0.1 # Temps du samling
N = int(TSim/Ts) + 1 # nombres de samples 

# Path for MV
MVPath = {0: 0, 5: 1, 50: 2, 80: 3, TSim: 3} # Chemin choisis

# Define initial parameters
Kp = 1
T = 5
theta = 0
TLead = 1
TLag = 5

def plotValues(Kp,T,theta,TLead,TLag,MVPath,TSim,Ts):
    
    N = int(TSim/Ts) + 1 # nombres de samples 

    # Variables a rempir
    t = []
    MV = []
    MVDelay = []
    PV_EBD = []
    PV_EFD = []
    PV_TRAP = []

    for i in range(0,N):
        t.append(i*Ts)
        SelectPath_RT(MVPath,t,MV)
        Delay_RT(MV,theta,Ts,MVDelay) 
        LeadLag_RT(MVDelay,Kp,TLead,TLag,Ts,PV_EBD,PVInit=0,method='EDB')
    
    return t,MV,PV_EBD



# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
t,path,y = plotValues(Kp,T,theta,TLead,TLag,MVPath,TSim,Ts)
line, = plt.plot(t,y, lw=2,label='LeadLag Responce (EDB)')
plt.plot(t,path)
ax.set_xlabel('Time [s]')


# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.25, bottom=0.3)

# Make a horizontal slider to control the frequency.
axKp = plt.axes([0.1, 0.25, 0.0225, 0.63])
axTlead = plt.axes([0.25, 0.05, 0.5, 0.03])
axTLag = plt.axes([0.25, 0.1, 0.5, 0.03])
axtheta = plt.axes([0.25, 0.15, 0.5, 0.03])

Kp_slider = Slider(ax=axKp,label='Kp',valmin=0,valmax=10,valinit=Kp,orientation="vertical")
TLead_slider = Slider(ax=axTlead,label='TLead',valmin=0.1,valmax=10,valinit=TLead)
TLag_slider = Slider(ax=axTLag,label='TLag',valmin=0.1,valmax=10,valinit=TLag)
theta_slider = Slider(ax=axtheta,label='theta',valmin=0,valmax=10,valinit=theta)

# The function to be called anytime a slider's value changes
def update(val):
    t,path,y = plotValues(Kp_slider.val,T,theta_slider.val,TLead_slider.val,TLag_slider.val,MVPath,TSim,Ts)
    line.set_ydata(y)
    fig.canvas.draw_idle()
    ax.set_ylim(0,max(y))

# register the update function with each slider
Kp_slider.on_changed(update)
TLead_slider.on_changed(update)
TLag_slider.on_changed(update)
theta_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    Kp_slider.reset()
    TLead_slider.reset()
    TLag_slider.reset()
    theta_slider.reset()
button.on_clicked(reset)

plt.show()