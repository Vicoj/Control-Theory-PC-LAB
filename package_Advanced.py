from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output

import package_DBR
from package_DBR import *

#-----------------------------------
def LeadLag_RT(MV,Kp,TLead,TLag,Ts,PV,PVInit=0,method='EDB'):
    
    """
    :MV: input vector
    :Kp: process gain
    :TLead: lead time Constant [s]
    :TLag: lag time Constant [s]
    :Ts: sampling period [s]
    :PV: output vector
    :MVInit: (optional: default value is 0)    
    :PVInit: (optional: default value is 0)
    :method: discretisation method (optional: default value is 'EBD')
        EBD: Euler Backward difference
        EFD: Euler Forward difference
        TRAP: Trapezoïdal method
        
    This function apends a value to the output vector "PV"        
    
    """    
    
    if (TLag != 0):
        K = Ts/TLag
        if len(PV) == 0:
            PV.append(PVInit)
        else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
            if method == 'EBD':
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
            elif method == 'EFD':
                PV.append((1-K)*PV[-1] + (K*Kp)*((TLead/Ts)*MV[-1] +(1-(TLead/Ts))*MV[-2]))
            elif method == 'TRAP':
                "PV.append((1/(2*T+Ts))*((2*T-Ts)*PV[-1] + Kp*Ts*(MV[-1] + MV[-2])))"            
            else:
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
    else:
        PV.append(Kp*MV[-1])

    return None

def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVp, MVi, MVd, E, OLP, ManFF, PVInit, method='EBD-EBD'):
    """
    :SP: Set Point vector
    :PV: Process Value vector
    :Man: Manual control vector (T/F)
    :MVMan: Man value vector
    :MVFF: Feed Forward vector

    :Kc: controller gain
    :Ti: integral time costant [s]
    :Td: derivative time costant [s]
    :alpha: Tdf = alpha*Td :Tdf: derivative filter time constant
    :Ts: Sampling period [s]


    :MV: Manipulated value vector
    :MVP: Proportionnal part vector
    :MVI: Integral part vector
    :MVD: Derivative part vector
    :E: Control Error vector

    :ManFF: Activated FF (T/F)
    :Pvinit: Initial Value PV


    :method: discretisation method (optional: default value is 'EBD')
        EBD-EDB: EDB for integral and EDB for derivative action
        EBD-TRAP: EDB for integral and TRAP for derivative action
        TRAP-EDB: TRAP for integral and EDB for derivative action
        TRAP-TRAP: TRAP for integral and TRAP for derivative action
        
        
    This function apends new values to the output vector "MV", "MVP", "MVI", "MVD".
     """
    
    #calcul de l'erreur SP-PV
    
    if(not OLP):
        if(len(PV)==0):
            E.append(SP[-1]-PVInit)
        else :
            E.append(SP[-1]-PV[-1])
    else:
        E.append(SP[-1])
    
    #calcul de MVp
    
    MVp.append(Kc*E[-1])
    
    #calcul MVi
    
    if(len(MVi)>0):
        MVi.append(MVi[-1]+(Kc*Ts*E[-1])/Ti)
    else :
        MVi.append(PVInit)
    
    #calcul MVd
    
    Tfd = alpha*Td
    if(Td>0):
        if(len(MVd)!=0):
            if(len(E)==1):
                MVd.append((Tfd/(Tfd+Ts))*MVd[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1]))
            else:
                MVd.append(( Tfd / (Tfd+Ts) )*MVd[-1] + ( (Kc*Td) / (Tfd+Ts) ) *(E[-1]-E[-2]))
        else :
            MVd.append(PVInit)
        
    #calcul saturation, anti emballement, reset saturation integrateur
    
    #mode automatique
    if(not Man[-1]):
        #saturation
        if(MVp[-1]+MVi[-1]+MVd[-1] <MVMin) :
            MVi[-1] = MVMin - MVp[-1] - MVd[-1] #ecrasement valeur de MV
        elif (MVp[-1]+MVi[-1]+MVd[-1] >=MVMax) :
            MVi[-1] = MVMax - MVp[-1] - MVd[-1]
        MV.append(MVp[-1]+MVi[-1]+MVd[-1])
    
    #mode manuel
    else :
        if(ManFF):
            MVi[-1]=MVMan[-1]-MVp[-1]-MVd[-1]
        else:
            MVi[-1]=MVMan[-1]-MVp[-1]-MVd[-1]-MVFF[-1]
            
        MV.append(MVp[-1]+MVi[-1]+MVd[-1])
    return None

def FF_RT(DV,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, DV0,PVInit,MVFF_Delay,MV_LL1,MV_LL2 ,MVFF):
    
    #Gain
    KFF = -(Kd/Kp) 

    #Dephasage
    thetaFF = np.max([ThetaD-ThetaP,0])
    PVFF = DV-DV0*np.ones_like(DV)
    Delay_RT(PVFF,thetaFF,Ts,MVFF_Delay,PVInit) 

    LeadLag_RT(MVFF_Delay,KFF,T1p,T1d,Ts,MV_LL1,PVInit,method='EDB')
    LeadLag_RT(MV_LL1,1,T2p,T2d,Ts,MV_LL2,PVInit,method='EDB')
    

    MVFF.append(MV_LL2[-1])

    return None

def IMC_Tuning():
    return None

class Simulation:
    def __init__(self,TSim,Ts,PVInit):
        self.TSim = TSim
        self.Ts = Ts
        self.N = int(TSim/Ts) + 1 
        self.PVInit = PVInit
        self.PV = []
        self.t = self.calc_t()

    def calc_t(self):
        t = []
        for i in range(0,self.N):
            t.append(i*self.Ts)

        return t

class Path:
    def __init__(self,S:Simulation,path):
        self.S = S
        self.path = path
        self.Signal = SelectPath_RT()

    def SelectPath_RT(self):
        signal = []
        for i in self.S.t:
            for timeKey in self.path:
                if(self.S.t[i-1] >= timeKey):
                    timeKeyPrevious = timeKey    
            
            value = self.path[timeKeyPrevious]
            signal.append(value)

class FirstOrder:
    def __init__(self,gain,Time,Theta,point_fct):
        self.K = gain
        self.T = Time
        self.Theta = Theta
        self.point_fct = point_fct
        self.PV = []

class FeedForward:
    def __init__(self,P:FirstOrder ,D:FirstOrder ):
        self.P = P
        self.D = D
        self.T1p = P.T
        self.T2p = 1
        self.T1d = D.T
        self.T2d = 1

        self.MV = []
        self.MV_LL1 = []
        self.MV_LL2 = []
        self.MVFF_Delay = []

class PID_Controller:
    def __init__(self,Kc,Ti,Td,alpha,MVmin,MVmax,OLP):
            self.Kc = Kc
            self.Ti = Ti
            self.Td = Td
            self.alpha = alpha
            self.MVmin = MVmin
            self.MVmax = MVmax
            self.OLP = OLP

            self.MVMan = []

            self.MV = []
            self.MVP = []
            self.MVI = []
            self.MVD = []
            self.E = []
