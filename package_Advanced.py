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
                #PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
                PV.append((1-K)*PV[-1] + (K*Kp)*((TLead/Ts)*MV[-1] +(1-(TLead/Ts))*MV[-2]))
            elif method == 'TRAP':
                "PV.append((1/(2*T+Ts))*((2*T-Ts)*PV[-1] + Kp*Ts*(MV[-1] + MV[-2])))"            
            else:
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
    else:
        PV.append(Kp*MV[-1])

def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MVPID, MVP, MVI, MVD, E, ManFF=False, PVInit=0, method='EBD-EBD'):

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

    #Initialisation de E
    if(len(PV)==0):
        E.append(SP[-1]-PVInit)
    else :
        E.append(SP[-1]-PV[-1])
    
    #Action Proportionelle
    MVP.append(Kc*E[-1])

    #Action integral
    if(len(MVI)>0):
        MVI.append(MVI[-1]+(Kc*Ts*E[-1])/Ti)
    else :
        MVI.append(0)
    
    #Action derivé
    Tfd = alpha*Td
    if(Td>0):
        if(len(MVD)!=0):
            if(len(E)==1):
                MVD.append((Tfd/(Tfd+Ts))*MVD[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1]))

            else:
                MVD.append(( Tfd / (Tfd+Ts) )*MVD[-1] + ( (Kc*Td) / (Tfd+Ts) ) *(E[-1]-E[-2]))
        else : MVD.append(0)

    result = MVP[-1]+MVI[-1]+MVD[-1]
    if (result > MVMax) :
        result = MVMax
    elif (result < MVMin) :
        result = MVMin
    else :
        result = result

    MVPID.append(result)
    
    return None


def FF_RT(Dv0,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, PV_FF, ManFF=False, PVInit=0, method='EBD-EBD'):

    PV_LL1 = []
    PV_LL2 = []
    PV_Delay = []

    KFF = -(Kd/Kp)

    Dv0_gain = Dv0*KFF

    thetaF = max(0,ThetaD-ThetaP)
    thetaFF = round(thetaF/Ts)

    LeadLag_RT(Dv0_gain,0,T1p,T1d,Ts,PV_LL1,PVInit=0,method='EDB')
    LeadLag_RT(PV_LL1,0,T2p,T2d,Ts,PV_LL2,PVInit=0,method='EDB')
    Delay_RT(PV_LL2,thetaFF,Ts,PV_Delay,MVInit=0)

    PV_FF = PV_Delay

    return None

    

