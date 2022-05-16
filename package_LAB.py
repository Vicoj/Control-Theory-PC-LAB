import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode



def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVp, MVi, MVd, E, OLP, ManFF=False, PVInit=0, method='EBD-EBD'):
    
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
        MVi.append(0)
    
    #calcul MVd
    
    Tfd = alpha*Td
    if(Td>0):
        if(len(MVd)!=0):
            if(len(E)==1):
                MVd.append((Tfd/(Tfd+Ts))*MVd[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1]))
            else:
                MVd.append(( Tfd / (Tfd+Ts) )*MVd[-1] + ( (Kc*Td) / (Tfd+Ts) ) *(E[-1]-E[-2]))
        else :
            MVd.append(0)
        
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

def IMC_tuning(MV,Kp,Ts, T1p ,Theta, gamma, case="H", T1=1, T2=1, T3=1):
    #theta process
    #Kp gain process
    #T1p = time constant process
    #gamma for desired closed loop time constant
    #
    
    Tc = gamma*T1p # 0.2 <gamma< 0.9
    
    if case == "G" :
        Kc = T1p/(Kp*Tc+Theta)
        Ti = T+Theta/2
        Td = 0
    if case == "H" :
        Kc = (T1p+Theta/2)/(Kp*Tc+Theta/2)
        Ti = T1p+Theta/2
        Td = (T1p*Theta)/(2*Tc+Theta)
    #if case == "I" :
        #Kc = (T1+T2-T3)/(Kp*Tc+Theta)
        #Ti = T1+T2-T3
        #Td = (T1*T2-(T1+T2-T3)*T3) /(T1+T2-T3)
    
    
    return (Kc, Ti, Td)

def FF_RT(DV,Kd, Kp, T1p, T1d, T2p, T2d , ThetaD, ThetaP, Ts, DV0,PVInit,MVFF_Delay,MV_LL1,MV_LL2 ,MVFF):

    KFF = -(Kd/Kp) #Gain

    thetaFF = np.max([ThetaD-ThetaP,0]) #Dephasage
    PVFF = DV-DV0*np.ones_like(DV)
    Delay_RT(PVFF,thetaFF,Ts,MVFF_Delay,PVInit) 

    LeadLag_RT(MVFF_Delay,KFF,T1p,T1d,Ts,MV_LL1,PVInit,method='EDB')
    LeadLag_RT(MV_LL1,1,T2p,T2d,Ts,MV_LL2,PVInit,method='EDB')
    

    MVFF.append(MV_LL2[-1])

    return None

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
