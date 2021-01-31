import numpy as np

class PIDController():
    
    def __init__(self):
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.Kf = 0.0
        self.Kw = 0.0
        self.limiter_max = 0.0
        self.limiter_min = 0.0
        
        self.outputs = {}
        
        self.outputs['out'] = 0.0
        
        self.out = 0.0
        self.error = 0.0
        self.error_prev = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.w = 0.0
        self.f = 0.0
    
    def reset(self):
        self.outputs['out'] = 0.0
        
        self.out = 0.0
        self.error = 0.0
        self.error_prev = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.w = 0.0
        self.f = 0.0        
    
    def limiter(self, value, limit_min, limit_max):
        if(limit_min != 0.0 or limit_max != 0.0):
            return np.clip(value, limit_min, limit_max)
        
        else:
            return value
        
    def step(self, t, dt, inputs, feedback):        
        setpoint = inputs['setpoint']
        feedback = feedback['feedback']
        
        self.error = setpoint - feedback
        
        self.f = setpoint * self.Kf
        self.p = self.error * self.Kp
        self.i += (self.error - self.w) * self.Ki * dt
        self.d = (self.error - self.error_prev) / dt * self.Kd
        self.error_prev = self.error
        
        self.out = self.limiter((self.p + self.i + self.d + self.f), self.limiter_min, self.limiter_max)
        
        self.w = ((self.p + self.i + self.d + self.f) - self.out) * self.Kw
        
        self.outputs['out'] = self.out