#https://docs.scipy.org/doc/scipy/tutorial/optimize.html#constrained-minimization-of-multivariate-scalar-functions-minimize

import numpy as np
import scipy.optimize as opt

def cost(x):
    return x[0]**2+x[1]**2+x[2]**2+x[3]**2+x[4]**2

limits = opt.Bounds([0.3,-np.inf,-np.inf,-np.inf,-np.inf],
                    [np.inf, np.inf, 5, np.inf, np.inf])

ineq_cons = {'type': 'ineq',
             'fun' : lambda x: np.array([5 - x[3]**2 - x[4]**2])
             }
eq_cons = {'type': 'eq',
           'fun' : lambda x: np.array([x[0]+x[1] + x[2] - 5,
                                       x[2]**2+x[3]-2])
           }

x0 = np.array([1, 1, 1, 2, 1])
res = opt.minimize(cost, x0, method='SLSQP', 
               constraints=[eq_cons, ineq_cons], options={'ftol': 1e-9, 'disp': True},
               bounds=limits)
print(res.x)
