import numpy as np
import matplotlib.pyplot as plt
import logging
import math
import time

def x_dot(t, x, w_):
    # State vector
    # x = [ w r_xy v_xy phi omega]' \in R^8
    ## Parâmetros
    m = 0.25 # massa
    g = 9.81 # aceleração da gravidade
    l = 0.1 # tamanho
    kf = 1.744e-08 # constante de força
    Iz = 2e-4 # momento de inércia
    tal = 0.005
    Fg = np.array([[0],[-m*g]])
    ## Estados atuais
    w = x[0:2]
    r = x[2:4]
    v = x[4:6]
    phi = x[6]
    ome = x[7]

    # forças
    f1 = kf * w[0]**2
    f2 = kf * w[1]**2
    # Torque
    Tc = l * (f1 - f2)
    # Força de controle
    Fc_B = np.array( [[0], [(f1 + f2)]])
    # Matriz de atitude
    D_RB = np.array([ [ np.cos(phi), -np.sin(phi)], [ np.sin(phi), np.cos(phi)]])
    ## Derivadas
    w_dot = (-w + w_)/tal
    r_dot = v
    v_dot = (1/m)*(D_RB @ Fc_B + Fg)
    v_dot = v_dot.reshape(2,)
    phi_dot = np.array([ome])
    ome_dot = np.array([Tc/Iz])
    xkp1 = np.concatenate([ w_dot,r_dot,v_dot,phi_dot,ome_dot])

    return xkp1

def rk4(tk, h, xk, uk):
    k1 = x_dot(tk , xk , uk)
    k2 = x_dot(tk + h/2.0, xk + h*k1/2.0, uk)
    k3 = x_dot(tk + h/2.0, xk + h*k2/2.0, uk)
    k4 = x_dot(tk + h , xk + h*k3 , uk)
    xkp1 = xk +(h/6.0)*(k1 + 2*k2 + 2*k3 + k4)
    return xkp1

def simulate(time):
    h = 5e-4 # passo da simulação de tempo continuo
    Ts = 0.005 # intervalo de atuação do controlador
    fTh = Ts/h
    maxT = time
    tc = np.arange(0,maxT,h) # k
    td = np.arange(0,maxT,Ts) # j
    tam = len(tc)
    j = 0
    # x = [ w r_xy v_xy phi omega]' \in R^8
    x = np.zeros([8, tam])
    #x[:,0] = np.array([0.,1.,2,3,4,5,6,7,])
    x[:,0] = np.array([0., 0., 0., 0., 0., .0, 0*np.pi/180., 0*np.pi/180.])
    u = np.zeros([2,len(td)]) # comando de controle
    
    eP_ = np.zeros([2,len(td)])
    m = 0.25 # massa
    g = 9.81 # aceleração da gravidade
    l = 0.1 # tamanho
    kf = 1.744e-08 # constante de força
    Iz = 2e-4 # momento de inércia
    
    tal = 0.05
    Fe = np.array([-m*g])
    # Restrições do controle
    phi_max = 15*np.pi/180. # ângulo máximo
    w_max = 15000
    Fc_max = kf*w_max**2 # Força de controle máximo
    Tc_max = l*kf*w_max**2

    # Waypoints
    r_ = np.array([ [0.,20.], \
    [15.,20.], \
    [-50.,4.], \
    [-20., 30.], \
    [ 0., 0.]]).transpose()
    r_ID = 0
    r_IDN = 4

    ## Execution

    for k in range(tam-1):
        # Sistema de controle
        if (k % fTh) == 0:
            # Extrai os dados do vetor
            r_k = x[2:4,k]
            v_k = x[4:6,k]
            phi_k = x[6,k]
            ome_k = x[7,k]
            # Comando de posição
            v_ = np.array([0,0])
            #####################
            # Controle de Posição
            kpP = np.array([.075])
            kdP = np.array([0.25])
            eP = r_[:,r_ID] - r_k
            eV = v_ - v_k
            eP_[:,j] = eP
            # Definição do próximo waypoint
            if np.linalg.norm(eP) < .1 and r_ID < (r_IDN):
                r_ID += 1
            
            Fx = kpP * eP[0] + kdP * eV[0]
            Fy = kpP * eP[1] + kdP * eV[1] - Fe
            Fy = np.maximum(0.2*Fc_max, np.minimum(Fy, 0.8*Fc_max))
            
            #####################
            # Controle de Atitude
            phi_ = np.arctan2(-Fx, Fy)
            if np.abs(phi_) > phi_max:
                signal = phi_/np.absolute(phi_)
                phi_ = signal * phi_max
                # Limitando o ângulo
                Fx = Fy * np.tan(phi_)
            
            Fxy = np.array([Fx, Fy])
            Fc = np.linalg.norm(Fxy)
            f12 = np.array([Fc/2.0, Fc/2.0])
            # Constantes Kp e Kd
            kpA = np.array([.75])
            kdA = np.array([0.05])
            ePhi = phi_ - phi_k
            eOme = 0 - ome_k
            Tc = kpA * ePhi + kdA * eOme
            Tc = np.maximum(-0.4*Tc_max, np.minimum(Tc, 0.4*Tc_max))
            # Delta de forças
            df12 = np.absolute(Tc)/2.0
            if (Tc >= 0.0):
                f12[0] = f12[0] + df12
                f12[1] = f12[1] - df12
            else:
                f12[0] = f12[0] - df12
                f12[1] = f12[1] + df12
            ################
            # Limitadores
            
            w1_ = np.sqrt(f12[0]/(kf))
            w2_ = np.sqrt(f12[1]/(kf))
            # Limitando o comando do motor entre 0 - 15000 rpm
            w1 = np.maximum(0., np.minimum(w1_, w_max))
            w2 = np.maximum(0., np.minimum(w2_, w_max))
            # Determinação do comando de entrada
            u[:,j] = np.array([w1, w2])
            j = j+1
        # Simulação um passo a frente
        x[:,k+1] = rk4(tc[k], h, x[:,k], u[:,j-1])

    # Processaento de variáveis intermediárias
    # obtem a força aplicada por cada rotor
    f = np.zeros([3, tam])
    for k in range(tam):
        w = x[0:2,k]
        f[0:2,k] = np.array([kf*w[0]**2, kf*w[1]**2])
        f[2,k] = f[0,k] + f[1,k] # Força total em B

    return x

class IteractiveSimulator:
    def __init__(self,timewindow):
        self.total_ticks=0
        self.total_time=0
        self.last_delta_tick=0
        self.h = 5e-4 # passo da simulação de tempo continuo
        self.Ts = 0.005 # intervalo de atuação do controlador
        self.fTh = self.Ts/self.h
        self.maxT = timewindow
        self.tc = np.arange(0,self.maxT,self.h) # k
        self.td = np.arange(0,self.maxT,self.Ts) # j
        self.tam = len(self.tc)
        self.j = 0
        # x = [ w r_xy v_xy phi omega]' \in R^8
        self.x = np.zeros([8, self.tam])
        #x[:,0] = np.array([0.,1.,2,3,4,5,6,7,])
        self.x[:,0] = np.array([0., 0., 0., 0., 0., .0, 0*np.pi/180., 0*np.pi/180.])
        self.u = np.zeros([2,len(self.td)]) # comando de controle

        self.eP_ = np.zeros([2,len(self.td)])
        self.m = 0.25 # massa
        self.g = 9.81 # aceleração da gravidade
        self.l = 0.1 # tamanho
        self.kf = 1.744e-08 # constante de força
        self.Iz = 2e-4 # momento de inércia

        self.tal = 0.05
        self.Fe = np.array([-self.m*self.g])
        # Restrições do controle
        self.phi_max = 15*np.pi/180 # ângulo máximo
        self.w_max = 15000
        self.Fc_max = self.kf*self.w_max**2 # Força de controle máximo
        self.Tc_max = self.l*self.kf*self.w_max**2
        # Waypoints
        self.r_ = np.array([0.,0.]).transpose()

    ## Execution
    def fillTimeWindow(self):
        j=0
        for k in range(self.tam-1):
            # Sistema de controle
            if (k % self.fTh) == 0:
                # Extrai os dados do vetor
                r_k = self.x[2:4,k]
                v_k = self.x[4:6,k]
                phi_k = self.x[6,k]
                ome_k = self.x[7,k]
                # Comando de posição
                v_ = np.array([0,0])
                #####################
                # Controle de Posição
                kpP = np.array([.075])
                kdP = np.array([0.25])
                eP = self.r_ - r_k
                eV = v_ - v_k
                self.eP_[:,j] = eP
                # Definição do próximo waypoint

                Fx = kpP * eP[0] + kdP * eV[0]
                Fy = kpP * eP[1] + kdP * eV[1] - self.Fe
                Fy = np.maximum(0.2*self.Fc_max, np.minimum(Fy, 0.8*self.Fc_max))

                #####################
                # Controle de Atitude
                phi_ = np.arctan2(-Fx, Fy)
                if np.abs(phi_) > self.phi_max:
                    signal = phi_/np.absolute(phi_)
                    phi_ = signal * self.phi_max
                    # Limitando o ângulo
                    Fx = Fy * np.tan(phi_)

                Fxy = np.array([Fx, Fy])
                Fc = np.linalg.norm(Fxy)
                f12 = np.array([Fc/2.0, Fc/2.0])
                # Constantes Kp e Kd
                kpA = np.array([.75])
                kdA = np.array([0.05])
                ePhi = phi_ - phi_k
                eOme = 0 - ome_k
                Tc = kpA * ePhi + kdA * eOme
                Tc = np.maximum(-0.4*self.Tc_max, np.minimum(Tc, 0.4*self.Tc_max))
                # Delta de forças
                df12 = np.absolute(Tc)/2.0
                if (Tc >= 0.0):
                    f12[0] = f12[0] + df12
                    f12[1] = f12[1] - df12
                else:
                    f12[0] = f12[0] - df12
                    f12[1] = f12[1] + df12
                ################
                # Limitadores

                w1_ = np.sqrt(f12[0]/(self.kf))
                w2_ = np.sqrt(f12[1]/(self.kf))
                # Limitando o comando do motor entre 0 - 15000 rpm
                w1 = np.maximum(0., np.minimum(w1_, self.w_max))
                w2 = np.maximum(0., np.minimum(w2_, self.w_max))
                # Determinação do comando de entrada
                self.u[:,j] = np.array([w1, w2])
                j = j+1
            # Simulação um passo a frente
            self.x[:,k+1] = rk4(self.tc[k], self.h, self.x[:,k], self.u[:,j-1])

        # Processaento de variáveis intermediárias
        # obtem a força aplicada por cada rotor
        #self.f = np.zeros([3, self.tam])
        #for k in range(self.tam):
        #    w = self.x[0:2,k]
        #    self.f[0:2,k] = np.array([self.kf*w[0]**2, self.kf*w[1]**2])
        #    self.f[2,k] = self.f[0,k] + self.f[1,k] # Força total em B
        self.prev_time=time.time()
        self.total_ticks+=self.tam
        self.total_time+=self.maxT

    def nextStep(self):
        now = time.time()
        delta_time = now - self.prev_time
        self.prev_time = now
        # one tick is step in the simulation
        delta_ticks =math.ceil(delta_time/self.h)
        delta_observed =math.ceil(delta_time/self.Ts)
        self.last_delta_tick=delta_ticks
        

        self.x=np.delete(self.x,np.s_[0:delta_ticks],axis=1)
        to_insert = np.zeros([8, delta_ticks])
        self.x=np.append(self.x,to_insert,axis=1)

        self.u=np.delete(self.u,np.s_[0:delta_observed],axis=1)
        to_insert = np.zeros([2, delta_observed])
        self.u=np.append(self.u,to_insert,axis=1)
        
        self.eP_=np.delete(self.eP_,np.s_[0:delta_observed],axis=1)
        to_insert = np.zeros([2, delta_observed])
        self.eP_=np.append(self.eP_,to_insert,axis=1)
        
        self.tc = np.arange(self.total_time+delta_time-self.maxT,self.total_time+delta_time,self.h) # k
        self.td = np.arange(self.total_time+delta_time-self.maxT,self.total_time+delta_time,self.Ts) # k

        j=len(self.td)-delta_observed-1
        for n in range(delta_ticks):
            k=len(self.tc)-delta_ticks+n-1

            if (k % self.fTh) == 0:
                # Extrai os dados do vetor
                r_k = self.x[2:4,k]
                v_k = self.x[4:6,k]
                phi_k = self.x[6,k]
                ome_k = self.x[7,k]
                # Comando de posição
                v_ = np.array([0,0])
                #####################
                # Controle de Posição
                kpP = np.array([.075])
                kdP = np.array([0.25])
                eP = self.r_ - r_k
                eV = v_ - v_k
                self.eP_[:,j] = eP
                # Definição do próximo waypoint

                Fx = kpP * eP[0] + kdP * eV[0]
                Fy = kpP * eP[1] + kdP * eV[1] - self.Fe
                Fy = np.maximum(0.2*self.Fc_max, np.minimum(Fy, 0.8*self.Fc_max))

                #####################
                # Controle de Atitude
                phi_ = np.arctan2(-Fx, Fy)
                if np.abs(phi_) > self.phi_max:
                    signal = phi_/np.absolute(phi_)
                    phi_ = signal * self.phi_max
                    # Limitando o ângulo
                    Fx = Fy * np.tan(phi_)

                Fxy = np.array([Fx, Fy])
                Fc = np.linalg.norm(Fxy)
                f12 = np.array([Fc/2.0, Fc/2.0])
                # Constantes Kp e Kd
                kpA = np.array([.75])
                kdA = np.array([0.05])
                ePhi = phi_ - phi_k
                eOme = 0 - ome_k
                Tc = kpA * ePhi + kdA * eOme
                Tc = np.maximum(-0.4*self.Tc_max, np.minimum(Tc, 0.4*self.Tc_max))
                # Delta de forças
                df12 = np.absolute(Tc)/2.0
                if (Tc >= 0.0):
                    f12[0] = f12[0] + df12
                    f12[1] = f12[1] - df12
                else:
                    f12[0] = f12[0] - df12
                    f12[1] = f12[1] + df12
                ################
                # Limitadores

                w1_ = np.sqrt(f12[0]/(self.kf))
                w2_ = np.sqrt(f12[1]/(self.kf))
                # Limitando o comando do motor entre 0 - 15000 rpm
                w1 = np.maximum(0., np.minimum(w1_, self.w_max))
                w2 = np.maximum(0., np.minimum(w2_, self.w_max))
                # Determinação do comando de entrada
                self.u[:,j] = np.array([w1, w2])
                j = j+1
            # Simulação um passo a frente
            self.x[:,k+1] = rk4(self.tc[k], self.h, self.x[:,k], self.u[:,j-1])

        self.total_ticks += delta_ticks
        self.total_time += delta_time

if __name__ == '__main__':
    logger=logging.basicConfig(encoding='utf-8', level=logging.DEBUG)
    simulate(60)