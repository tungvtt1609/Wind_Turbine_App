import numpy as np
from numpy.ctypeslib import ndpointer
import matplotlib.pyplot as plt
import ctypes as ct
import _ctypes
from contextlib import contextmanager
#from DTU10MW_control_settings import settings

DELTAT = 0.01
N_ITER = 1000

def main():
    pass
    # filename = '../src/dtu_we_controller/x64/Release/dtu_we_controller.dll'
    # out = np.empty([N_ITER, 4])
    # update = np.array([0.0, 1.005, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 10000000.0, 0, 0.0, 0.0])
    # 
    # with open_DTUController(filename) as dll:
    #     dll.init(settings)
    # 
    #     for i in range(N_ITER):
    #         update[0] += DELTAT
    #         #update[1] = 1.005 + np.random.normal()*0.005
    #         print(f'\riteration {i}...', end='')
    # 
    #         out[i, :] = dll.update(update)[:4]
    # 
    #     np.savez('test_python_interface.npz', out)
    # 

     
        

    # make plot
    fig, axes = plt.subplots(4, sharex=True)
    labels = ['Gen Torque', 'pitch 1', 'pitch 2', 'pitch 3']
    for i, ax in enumerate(axes):
        
        ax.plot(out[:, i], label=labels[i])
        ax.legend()
    plt.show()
    
    
    
@contextmanager
def open_DTUController(filename):
    dll = fortran_dll(filename)
    yield dll
    dll.close()  
    
    
        
class fortran_dll(object):
    def __init__(self, filename):
        self.lib = ct.CDLL(filename)
        

    def init(self, array):
        f = self.lib.init_regulation_advanced
        
        f.argtypes = [ndpointer(shape=100, dtype=ct.c_double, flags='FORTRAN'),
                      ndpointer(shape=1, dtype=ct.c_double, flags='FORTRAN')]
        f.restype = None
        
        array = zeropad_to_length(array, 100)
        arg1 = np.array(array, dtype=ct.c_double, order='F')
        arg2 = np.zeros(1, dtype=ct.c_double, order='F')
    
        f(arg1, arg2)
        
        return(arg2)
        
        
    def update(self, array):
        f = self.lib.update_regulation
        
        f.argtypes = [ndpointer(shape=100, dtype=ct.c_double, flags='FORTRAN'),
                      ndpointer(shape=100, dtype=ct.c_double, flags='FORTRAN')]
        f.restype = None
        
        array = zeropad_to_length(array, 100)
        arg1 = np.array(array, dtype=ct.c_double, order='F')
        arg2 = np.zeros(100, dtype=ct.c_double, order='F')
        f(arg1, arg2)
        
        return(arg2)
        
        
    def close(self):
            _ctypes.FreeLibrary(self.lib._handle)


    
def zeropad_to_length(array, n):
    zeros = np.zeros(n-len(array))
    return np.concatenate([array, zeros])        

        
        

if __name__ == '__main__':    
    main()
    



    
