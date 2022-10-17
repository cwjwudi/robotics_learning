#Unconstrained optimization using BFGS
import scipy.optimize as opt

def fun(paramt):
    x1,x2 = paramt
    return 100*(x2-x1**2)**2 + (1-x1)**2


x0 = [1, 2]
#res = optimize.minimize(fun,x0)
res = opt.minimize(fun,x0,method='BFGS')

print(res.x)
