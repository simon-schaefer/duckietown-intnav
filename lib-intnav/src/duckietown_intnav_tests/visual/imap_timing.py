#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test imap performance i.e. the timing required for imap calls such as 
# initialization, map coordinate transformation, etc. 
###############################################################################
import matplotlib.pyplot as plt
import numpy as np
import time

from duckietown_intnav.imap import IMap

# Test imap initialization time. 
resolutions = np.arange(0.001, 2, 0.01)
times = []
for r in resolutions: 
    start_time = time.time()
    imap = IMap("4", resolution=r)
    times.append((time.time() - start_time)*1000)
plt.plot(resolutions, times)
plt.xlabel("Resolutions [m]")
plt.ylabel("Runtime [ms]")
plt.show()
