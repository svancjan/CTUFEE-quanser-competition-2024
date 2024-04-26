import scipy.io
import numpy as np

mat = scipy.io.loadmat('Qpath1.mat')
np.save('Qpath.npy', mat['QPath'])
