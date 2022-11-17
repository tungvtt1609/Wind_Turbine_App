import os
#from numpy import testing as npt
#from scipy import stats

import numpy as np
from numpy.testing import assert_almost_equal
from .DTUControllerDLL import open_DTUController
from .DTU10MW_control_settings import settings

test_filepath = os.path.dirname(__file__) + '/test_files/'
dll_filepath = os.path.dirname(__file__) + '/../../' + 'src/dtu_we_controller/x64/Release/dtu_we_controller_64.dll'
dll_filepath = os.path.abspath(dll_filepath)

        
        
def test_python_interface():
    out = np.empty([1000, 4])
    update = np.array([0.0, 1.005, 0.0, 0.0, 0.0, 0.0, 10, 0.0, 10000000.0, 0, 0.0, 0.0])
    
    with open_DTUController(dll_filepath) as dll:
        dll.init(settings)
        for i in range(1000):
            update[0] += 0.01
            #update[1] = 1.005 + np.random.normal()*0.005
            #print(f'\riteration {i}...', end='')
            
            out[i, :] = dll.update(update)[:4]
        
    # compare to reference
    ref = np.load(test_filepath + 'test_python_interface.npz')['arr_0']
    assert_almost_equal(ref, out)
            

    
