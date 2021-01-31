import numpy as np
from pyBLDC.controller import PID

class FOCController():
    
    def __init__(self):
        self.PID_id = PID.PIDController()
        self.PID_iq = PID.PIDController()
        
        self.voltage = 0.0
        
        self.outputs = {}
        
        self.outputs['va'] = 0.0
        self.outputs['vb'] = 0.0
        self.outputs['vc'] = 0.0
        
    def reset(self):
        self.PID_id.reset()
        self.PID_iq.reset()
        
        self.outputs['va'] = 0.0
        self.outputs['vb'] = 0.0
        self.outputs['vc'] = 0.0
    
    def step(self, t, dt, inputs, feedback):
        
        theta = feedback['theta']
        ia = feedback['ia']
        ib = feedback['ib']
        ic = feedback['ic']
        
        K_CP = np.sqrt(2.0 / 3.0) * np.array([[np.cos(theta),       np.cos(theta - (2.0 * np.pi / 3.0)),    np.cos(theta + (2.0 * np.pi / 3.0))],
                                             [-np.sin(theta),       -np.sin(theta - (2.0 * np.pi / 3.0)),   -np.sin(theta + (2.0 * np.pi / 3.0))],
                                             [np.sqrt(2.0) / 2.0,   np.sqrt(2.0) / 2.0,                     np.sqrt(2.0) / 2.0]]) 
        
        K_CP_inv = np.sqrt(2.0 / 3.0) * np.array([[np.cos(theta),                           -np.sin(theta),                         np.sqrt(2.0) / 2.0],
                                                 [np.cos(theta - (2.0 * np.pi / 3.0)),      -np.sin(theta - (2.0 * np.pi / 3.0)),   np.sqrt(2.0) / 2.0],
                                                 [np.cos(theta + (2.0 * np.pi / 3.0)),      -np.sin(theta + (2.0 * np.pi / 3.0)),   np.sqrt(2.0) / 2.0]])
        
        i_abc = np.array([[ia], [ib], [ic]])
        u_DQZ = np.dot(K_CP, i_abc)
        
        id_inputs = {}
        iq_inputs = {}
        id_inputs['setpoint'] = 0.0
        iq_inputs['setpoint'] = inputs['setpoint']
        
        id_feedback = {}
        iq_feedback = {}        
        id_feedback['feedback'] = u_DQZ[0, 0]
        iq_feedback['feedback'] = u_DQZ[1, 0]
        
        self.PID_id.step(t, dt, id_inputs, id_feedback)
        self.PID_iq.step(t, dt, iq_inputs, iq_feedback)
        
        vd = self.PID_id.outputs['out']
        vq = self.PID_iq.outputs['out']    
        
        u_DQZ = np.array([[vd], [vq], [0.0]])        
        u_DQZ = np.clip(u_DQZ, -1.0, 1.0)
        
        v_abc = np.dot(K_CP_inv, u_DQZ)                
        
        self.outputs['va'] = v_abc[0, 0] * self.voltage / 2.0
        self.outputs['vb'] = v_abc[1, 0] * self.voltage / 2.0
        self.outputs['vc'] = v_abc[2, 0] * self.voltage / 2.0
    
    