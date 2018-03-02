#!/usr/bin/env python
theta = {
    'left': {'min': 300, 'max': 40},
    'right': {'min': 120, 'max': 220},
    'shift': 35
}

class Control:
    def __init__(self, config_dict):
        self.config_dict = config_dict
        
    def fx(self, id, angle, speed, speed_ref):
        dth = (speed/speed_ref)*theta['shift']
        
        theta_min = theta[id]['min'] - dth
        theta_max = theta[id]['max'] - dth
        
        # check if angle in range (theta_min, theta_max)
        if theta_min <= angle and angle <= theta_max:
            return 1
        elif theta[id]['min'] > theta[id]['max']:
            if angle <= theta_min and angle <= theta_max:
                if theta_min <= angle + 360 and angle <= theta_max:
                    return 1
            elif angle >= theta_min and angle >= theta_max:
                if theta_min <= angle and angle <= theta_max + 360:
                    return 1
                    
        # return 0 otherwise
        return 0
    
    def g(self, error):
        Kp = 1/float(5000)
        Ki = 1/float(100000)
        
        # If there is a change of signal, reset
        if ((error[-2] >= 0) and (error[-1] < 0)) or ((error[-2] < 0) and (error[-1] >= 0)):
            errorTemp = [0 for x in range(len(error))]
            errorTemp[-1] = error[-1]
            error = errorTemp
        
        signal = 0.5 + Kp*error[-1]+Ki*sum(error)
        
        # saturation
        if signal > 1:
            signal = 1
            error[-1] = 0
        elif signal < 0:
            signal = 0
            error[-1] = 0
        
        return signal
            
    def calculate(self, angle, speed, speed_ref, speed_err):
        pw_max = 500
        
        fx_left = self.fx('left', angle, speed, speed_ref)
        fx_right = self.fx('right', angle, speed, speed_ref)
        
        g = self.g(speed_err)
        
        pw_left = fx_left*g*pw_max
        pw_right = fx_right*g*pw_max
        
        # print g, pw_left, pw_right
        # print [round(x) for x in speed_err[-10:]]
        
        return pw_left, pw_right
