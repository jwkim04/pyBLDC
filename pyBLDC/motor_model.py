import numpy as np
import json

class Motor():
    
    def __init__(self):
        
        #Motor Constants
        self.Kt = 0.0    #Torque Constant           Nm/A
        self.R = 0.0     #Phase Resistance          Ohm
        self.L = 0.0     #Phase Inductance          H
        self.M = 0.0     #Phase Mutual Inductance   H
        self.I = 0.0     #Inertia                   kg*m^2
        self.J = 0.0     #Friction                  Nm/(rad/s)
        self.pole = 2    #Motor Pole pair
        
        
        
        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0        
        self.back_emf_a = 0.0
        self.back_emf_b = 0.0
        self.back_emf_c = 0.0
        self.phase_voltage_a = 0.0
        self.phase_voltage_b = 0.0
        self.phase_voltage_c = 0.0
        self.omega = 0.0
        self.tau = 0.0 
        self.theta = 0.0
        self.electrical_theta = 0.0
        
        self.outputs = {}
        
        self.outputs['ia'] = 0.0
        self.outputs['ib'] = 0.0
        self.outputs['ic'] = 0.0
        self.outputs['back_emf_a'] = 0.0
        self.outputs['back_emf_b'] = 0.0
        self.outputs['back_emf_c'] = 0.0
        self.outputs['phase_voltage_a'] = 0.0
        self.outputs['phase_voltage_b'] = 0.0
        self.outputs['phase_voltage_c'] = 0.0
        self.outputs['omega'] = 0.0
        self.outputs['tau'] = 0.0
        self.outputs['theta'] = 0.0
        self.outputs['electrical_theta'] = 0.0
        
        
    def reset(self):
        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0        
        self.back_emf_a = 0.0
        self.back_emf_b = 0.0
        self.back_emf_c = 0.0
        self.phase_voltage_a = 0.0
        self.phase_voltage_b = 0.0
        self.phase_voltage_c = 0.0
        self.omega = 0.0
        self.tau = 0.0 
        self.theta = 0.0
        self.electrical_theta = 0.0
        
        self.outputs['ia'] = 0.0
        self.outputs['ib'] = 0.0
        self.outputs['ic'] = 0.0
        self.outputs['back_emf_a'] = 0.0
        self.outputs['back_emf_b'] = 0.0
        self.outputs['back_emf_c'] = 0.0
        self.outputs['phase_voltage_a'] = 0.0
        self.outputs['phase_voltage_b'] = 0.0
        self.outputs['phase_voltage_c'] = 0.0
        self.outputs['omega'] = 0.0
        self.outputs['tau'] = 0.0
        self.outputs['theta'] = 0.0
        self.outputs['electrical_theta'] = 0.0
    
    def loadPreset(self, path):        
        file = open(path, 'r') 
        
        preset = json.load(file)

        self.Kt = preset['Kt']
        self.R = preset['R']
        self.L = preset['L']
        self.M = preset['M']
        self.I =preset['I']
        self.J = preset['J']
        self.pole = preset['pole']
        
        file.close()
    
    def savePreset(self, path):        
        preset = {}
        preset['Kt'] = self.Kt
        preset['R'] = self.R
        preset['L'] = self.L
        preset['M'] = self.M
        preset['I'] = self.I
        preset['J'] = self.J
        preset['pole'] = self.pole
        
        file = open(path, 'w')        
        file.write(json.dumps(preset))
        file.close()
    
    def step(self, t, dt, inputs):           
        va = inputs['va']
        vb = inputs['vb']
        vc = inputs['vc']
        
        load_torque = inputs['load_torque']
        load_inertia = inputs['load_inertia']
        
        v_avg = (va + vb + vc) / 3.0
        self.phase_voltage_a = va - v_avg
        self.phase_voltage_b = vb - v_avg
        self.phase_voltage_c = vc - v_avg

        self.ia += (self.phase_voltage_a - self.back_emf_a - self.ia * self.R) / (self.L - self.M) * dt
        self.ib += (self.phase_voltage_b - self.back_emf_b - self.ib * self.R) / (self.L - self.M) * dt
        self.ic += (self.phase_voltage_c - self.back_emf_c - self.ic * self.R) / (self.L - self.M) * dt
        
        i_avg  =  (self.ia + self.ib + self.ic) / 3.0
        self.ia -= i_avg
        self.ib -= i_avg
        self.ic -= i_avg
        
        self.electrical_theta = self.theta * self.pole / 2.0
        sin_a = np.sin(self.electrical_theta)
        sin_b = np.sin(self.electrical_theta - (np.pi * 2.0 / 3.0))
        sin_c = np.sin(self.electrical_theta - (np.pi * 4.0 / 3.0))
        
        self.tau = -self.Kt * (self.ia * sin_a + self.ib * sin_b + self.ic * sin_c)
        
        self.omega += (self.tau - self.omega * self.J - load_torque) / (self.I + load_inertia) * dt
        self.theta = self.theta + self.omega * dt
        
        self.back_emf_a = self.omega * self.Kt * sin_a
        self.back_emf_b = self.omega * self.Kt * sin_b
        self.back_emf_c = self.omega * self.Kt * sin_c
        
        K_CP = np.sqrt(2.0 / 3.0) * np.array([[np.cos(self.electrical_theta),       np.cos(self.electrical_theta - (2.0 * np.pi / 3.0)),    np.cos(self.electrical_theta + (2.0 * np.pi / 3.0))],
                                             [-np.sin(self.electrical_theta),       -np.sin(self.electrical_theta - (2.0 * np.pi / 3.0)),   -np.sin(self.electrical_theta + (2.0 * np.pi / 3.0))],
                                             [np.sqrt(2.0) / 2.0,                   np.sqrt(2.0) / 2.0,                                     np.sqrt(2.0) / 2.0]]) 
        
        i_abc = np.array([[self.ia], [self.ib], [self.ic]])
        u_DQZ = np.dot(K_CP, i_abc)
        
        self.outputs['id'] = u_DQZ[0, 0]
        self.outputs['iq'] = u_DQZ[1, 0]
        self.outputs['ia'] = self.ia
        self.outputs['ib'] = self.ib
        self.outputs['ic'] = self.ic
        self.outputs['back_emf_a'] = self.back_emf_a
        self.outputs['back_emf_b'] = self.back_emf_b
        self.outputs['back_emf_c'] = self.back_emf_c
        self.outputs['phase_voltage_a'] = self.phase_voltage_a
        self.outputs['phase_voltage_b'] = self.phase_voltage_b
        self.outputs['phase_voltage_c'] = self.phase_voltage_c
        self.outputs['omega'] = self.omega
        self.outputs['tau'] = self.tau
        self.outputs['theta'] = self.theta
        self.outputs['electrical_theta'] = self.electrical_theta