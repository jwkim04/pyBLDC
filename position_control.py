import numpy as np
from scipy import signal
from pyBLDC import simulation
import plot

sim = simulation.Simulation()

sim.reset()

sim.motor.loadPreset('motor_preset.json')
sim.setControlMode(simulation.CONTROL_MODE_POSITION)
sim.setSimFreq(1.0e+4)

sim.setVoltage(24.0)    #V
sim.setLoadTorque(0.0)  #Nm
sim.setLoadInertia(0.0) #kg*m^2

sim.position_controller_inputs['setpoint'] = 0.0

sim.PositionController.Kp = 20.0
sim.PositionController.Ki = 0.0

sim.VelocityController.Kp = 2.0
sim.VelocityController.Ki = 1.0

sim.CurrentController.PID_id.Kp = 2.0
sim.CurrentController.PID_iq.Kp = 2.0
sim.CurrentController.PID_id.Ki = 1.0
sim.CurrentController.PID_iq.Ki = 1.0

TIME = 1.0 #sec

AMPLITUDE = np.pi / 4.0 #rad
FREQUNCY = 5.0 #Hz

def call_back(t, dt):
    sim.position_controller_inputs['setpoint'] = np.sin(2 * np.pi * t * FREQUNCY) * AMPLITUDE
    #sim.position_controller_inputs['setpoint'] = signal.square(2 * np.pi * t * 1.0) * AMPLITUDE
    
result = sim.simpleAnalysis(TIME, call_back)

plot.plotData(result)