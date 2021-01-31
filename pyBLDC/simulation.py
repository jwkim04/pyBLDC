from pyBLDC import motor_model
from pyBLDC.controller import FOC
from pyBLDC.controller import PID
from tqdm import tqdm
import numpy as np

CONTROL_MODE_CURRENT = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3

class Simulation():
    
    def __init__(self):         
        self.motor = motor_model.Motor()
        
        self.load_torque = 0.0
        self.load_inertia = 0.0
        self.sim_dt = 0.0        
        self.control_mode = CONTROL_MODE_CURRENT            
        
        self.PositionController = PID.PIDController()
        self.VelocityController = PID.PIDController()
        self.CurrentController = FOC.FOCController()          
        
        self.position_controller_inputs = {}
        self.position_controller_inputs['setpoint'] = 0.0
        
        self.velocity_controller_inputs = {}
        self.velocity_controller_inputs['setpoint'] = 0.0
        
        self.current_controller_inputs = {}
        self.current_controller_inputs['setpoint'] = 0.0        
        
        self.voltage = 0.0
        
    def reset(self):
        self.motor.reset()
        
        self.PositionController.reset()
        self.VelocityController.reset()
        self.CurrentController.reset()
    
    def setVoltage(self, voltage):
        self.voltage = voltage
    
    def setLoadTorque(self,load_torque):
        self.load_torque = load_torque
        
    def setLoadInertia(self,load_inertia):
        self.load_inertia = load_inertia        
    
    def setSimFreq(self, hz):        
        self.sim_dt = 1.0 / hz

    def setControlMode(self,control_mode):        
        self.control_mode = control_mode    
    
    def simpleAnalysis(self, end_time, call_back=None, process_bar=True):        
        DQZ = []
        position = []
        velocity = []        
        phase_voltage = []
        phase_current = []
        back_EMF = []                
        
        self.CurrentController.voltage = self.voltage
        self.CurrentController.pole = self.motor.pole
        
        if(process_bar):        
            range_gen = tqdm(np.arange(0.0, end_time, self.sim_dt))
        else:
            range_gen = np.arange(0.0, end_time, self.sim_dt)
        
        for t in range_gen:              
            if(call_back != None):
                call_back(t, self.sim_dt)
            
            position_controller_feedback = {}
            velocity_controller_feedback = {}
            current_controller_feedback = {}                                    
            
            position_controller_feedback['feedback'] = self.motor.outputs['theta']
            velocity_controller_feedback['feedback'] = self.motor.outputs['omega']
            
            current_controller_feedback['theta'] = self.motor.outputs['electrical_theta']
            current_controller_feedback['ia'] = self.motor.outputs['ia']
            current_controller_feedback['ib'] = self.motor.outputs['ib']
            current_controller_feedback['ic'] = self.motor.outputs['ic']
            
            if(self.control_mode >= CONTROL_MODE_POSITION):
                self.PositionController.step(t, self.sim_dt, self.position_controller_inputs, position_controller_feedback)
                self.velocity_controller_inputs['setpoint'] = self.PositionController.outputs['out']
            
            if(self.control_mode >= CONTROL_MODE_VELOCITY):
                self.VelocityController.step(t, self.sim_dt, self.velocity_controller_inputs, velocity_controller_feedback)
                self.current_controller_inputs['setpoint'] = self.VelocityController.outputs['out']
            
            if(self.control_mode >= CONTROL_MODE_CURRENT):
                self.CurrentController.step(t, self.sim_dt, self.current_controller_inputs, current_controller_feedback)
                
                motor_inputs = {}                                
                
                motor_inputs['load_torque'] = self.load_torque
                motor_inputs['load_inertia'] = self.load_inertia
                
                motor_inputs['va'] = self.CurrentController.outputs['va']
                motor_inputs['vb'] = self.CurrentController.outputs['vb']
                motor_inputs['vc'] = self.CurrentController.outputs['vc']
                
                self.motor.step(t, self.sim_dt, motor_inputs)
            
            DQZ.append([self.motor.outputs['id'], self.motor.outputs['iq'], self.current_controller_inputs['setpoint']])
            position.append([self.motor.outputs['theta'], self.position_controller_inputs['setpoint']])
            velocity.append([self.motor.outputs['omega'], self.velocity_controller_inputs['setpoint']])
            phase_voltage.append([self.motor.outputs['phase_voltage_a'], self.motor.outputs['phase_voltage_b'], self.motor.outputs['phase_voltage_c']])
            phase_current.append([self.motor.outputs['ia'], self.motor.outputs['ib'], self.motor.outputs['ic']])
            back_EMF.append([self.motor.outputs['back_emf_a'], self.motor.outputs['back_emf_b'], self.motor.outputs['back_emf_c']])
        
        result = {}
        
        result['time'] = np.linspace(0.0, end_time, int(end_time / self.sim_dt))
                
        result['position'] = position
        result['velocity'] = velocity
        result['DQZ'] = DQZ
        result['phase_voltage'] = phase_voltage
        result['phase_current'] = phase_current       
        result['back_EMF'] = back_EMF
                
        return result