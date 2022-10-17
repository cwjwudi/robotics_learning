%%
syms l1 l2 l3 ...
     q1 q2 q3 x y z real
T01 = [cos(q1)   -sin(q1)     0    0;
       sin(q1)    cos(q1)     0    0;
          0         0         1    0;
          0         0         0    1];
T12 = [cos(q2)   -sin(q2)     0    l1;
       sin(q2)    cos(q2)     0    0;
          0         0         1    0;
          0         0         0    1];
T23 = [cos(q3)   -sin(q3)     0    l2;
       sin(q3)    cos(q3)     0    0;
          0         0         1    0;
          0         0         0    1];
T34 = [1    0      0    l3;
       0    1      0    0;
       0    0      1    0
       0    0      0    1];
T04 = T01*T12*T23*T34;
xyz = T04(1:3,4:4);
Jac = jacobian(T04(1:2,4:4),[q1;q2;q3]);
%%
%l1 = 1m, l2 = 0.8m, l3 = 0.6m, q1 = 30°, q2 = 45°, q3 = 75°
%末端速度
w1 = [1; 0.5];
J1 = eval(subs(Jac, ...
          {l1,l2,l3, ...
           q1,q2,q3},...
          {1, 0.8, 0.6, deg2rad(30), deg2rad(45), deg2rad(75) ...
           }));
%第2关节速度=0
w2 = 0;
J2 = [0  1  0];
N1 = eye(3,3) - pinv(J1)*J1;
q_desired = pinv(J1)*w1 +  pinv(J2*N1)*(w2 - J2*pinv(J1)*w1);
error = norm((q_desired - (pinv(J1)*w1 +  N1*pinv(J2*N1)*(w2 - J2*pinv(J1)*w1))), 2);
error1 = norm((J1*q_desired - w1), 2);
error2 = norm((J2*q_desired - w2), 2);
fprintf('the error of add N1 or not is: %e\n',error)
fprintf('the error of endpoint_vel is: %e\n',error1)
fprintf('the error of add second joint vel is: %e\n',error2)