import scipy.optimize as opt
import numpy as np

def cost(x):
    return x[0]**2+x[1]**2+x[2]**2+x[3]**2+x[4]**2

def nonlinear_fn(x):
    return [ 5 - x[3]**2 - x[4]**2, x[2]**2+x[3]-2 ]

limits = opt.Bounds([0.3,-np.inf,-np.inf,-np.inf,-np.inf],
                    [np.inf, np.inf, 5, np.inf, np.inf])

linear_constraint = opt.LinearConstraint([ [1,1,1,0,0] ], [5], [5])
nonlinear_constraint = opt.NonlinearConstraint(nonlinear_fn, [0, 0], [np.inf, 0])

x0 =  np.array([1, 1, 1, 2, 1])
res = opt.minimize(cost, x0, method='trust-constr',
               constraints=[linear_constraint, nonlinear_constraint],
               options={'verbose': 1}, bounds=limits)

print(res.x)
