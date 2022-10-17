import numpy as np

def cos(theta):
    return np.cos(theta)

def sin(theta):
    return np.sin(theta)


theta = 0.5;
B = np.array([[np.cos(theta), -sin(theta), sin(theta), cos(theta)]])
# print(B)


A = np.array([[2,4],[5,-6]])
b = np.array([1,2])
D = A.dot(b)
print(D)
#
# print(A.transpose())
# print(np.transpose(A))

Ainv = np.linalg.inv(A)
# print(Ainv)

print(np.matmul(Ainv,A))

print(np.identity(3))
