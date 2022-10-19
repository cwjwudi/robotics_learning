import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

import spatialmath.base as base

puma = rtb.models.Puma560()
t = np.arange(0, 2, 0.010)
T0 = SE3(0.0, 0.0, 0.6)
T1 = SE3(0.0, 0.0, 0.0)
Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))

sol = puma.ikine_LM(Ts)

puma.plot(sol.q)
