import sympy as sy

def fun(x,y):
    f = sy.Matrix([x**2+y**2, 2*x+3*y+5])
    return f

x,y  = sy.symbols('x y', real=True)

# f = sy.Matrix([x**2+y**2, 2*x+3*y+5]); #f can also be defined outside
f = fun(x,y)
z = sy.Matrix([x, y])
J = f.jacobian(z)
print(J)

z0 = [1, 2]
J_sym = J.subs([ (x,z0[0]), (y,z0[1])])
print(J_sym)
