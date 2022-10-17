import sympy as sy
#In terminal type pip install sympy

x  = sy.symbols('x', real=True) # defining the variables
f0 = x**2+2*x+1
print(f0)
df0dx = sy.diff(f0,x) # Gives the first derivative of the function
print(df0dx)

xval = 1
print(df0dx.subs(x,xval))
