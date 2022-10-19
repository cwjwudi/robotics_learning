import roboticstoolbox as rtb
import spatialmath.base as base

puma = rtb.models.DH.Puma560(symbolic=True)
q = base.sym.symbol("q_:6") # q = (q_1, q_2, ... q_5)
T = puma.fkine(q)
print(T.t[0])
