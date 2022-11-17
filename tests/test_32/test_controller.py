import os
from wetb.hawc2.htc_file import HTCFile
from wetb.hawc2 import Hawc2io
from numpy import testing as npt
from scipy import stats


h2_path = os.path.dirname(__file__) + '/../../' + 'hawc2-win32/hawc2mb.exe'
h2_path = os.path.abspath(h2_path)
m_path = os.path.dirname(__file__) + '/../../' + 'examples/HAWC2/DTU10MW'
m_path = os.path.abspath(m_path)
f_path = os.path.join(m_path, 'htc/DTU_10MW_RWT_short.htc')
res_path = os.path.join(m_path, 'tmp/dtu_10mw_rwt_short')
ref_path = os.path.join(os.path.dirname(__file__), 'test_files', 'DTU_10MW_RWT_short')
htc = HTCFile(f_path, m_path)


def test_10mw_short():
    stdout, log = htc.simulate(h2_path)
    if "error" in log.lower():
        raise Exception(log.replace("\r\n", '\n'))
    res = Hawc2io.ReadHawc2(res_path)
    ref = Hawc2io.ReadHawc2(ref_path)
    ress = stats.describe(res())._asdict()
    refs = stats.describe(ref())._asdict()
    for ref, res in zip(refs['mean'], ress['mean']):
        npt.assert_approx_equal(ref, res, significant=4)
        
        

            

    
