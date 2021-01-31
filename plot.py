from matplotlib import pyplot as plt
import numpy as np

def plotData(data):
    t_axis = data['time']
    
    data_len = len(data) - 1
    i = 1
          
    for key, value in data.items():
        value = np.array(value)
        
        if(key != 'time'):
            plt.subplot(data_len * 100 + 10 + i)
            plt.title(key)
            plt.xlabel('t')
            
            if(key == 'position'):
                plt.plot(t_axis, value[:, 0], label='position')
                plt.plot(t_axis, value[:, 1], '--', label='setpoint')
            
            elif(key == 'velocity'):
                plt.plot(t_axis, value[:, 0], label='velocity')
                plt.plot(t_axis, value[:, 1], '--', label='setpoint')
            
            elif(key == 'DQZ'):
                plt.plot(t_axis, value[:, 0], label='id')
                plt.plot(t_axis, value[:, 1], label='iq')
                plt.plot(t_axis, value[:, 2], '--', label='iq setpoint')                
            
            elif(key == 'phase_voltage'):
                plt.plot(t_axis, value[:, 0], label='phase voltage a')
                plt.plot(t_axis, value[:, 1], label='phase voltage b')
                plt.plot(t_axis, value[:, 2], label='phase voltage c')  
            
            elif(key == 'phase_current'):
                plt.plot(t_axis, value[:, 0], label='ia')
                plt.plot(t_axis, value[:, 1], label='ib')
                plt.plot(t_axis, value[:, 2], label='ic') 
                
            elif(key == 'back_EMF'):
                plt.plot(t_axis, value[:, 0], label='BEMF phase a')
                plt.plot(t_axis, value[:, 1], label='BEMF phase b')
                plt.plot(t_axis, value[:, 2], label='BEMF phase c')                 
            
            else:
                plt.plot(t_axis, value)
            
            plt.legend(loc='upper right')
            i += 1
        
    plt.show()