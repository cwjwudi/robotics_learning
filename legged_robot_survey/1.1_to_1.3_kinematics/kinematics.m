%%
syms hipy_to_hipx knee_to_hipy foot_to_knee ...
     q0 q1 q2 x y z real
%hipx rt. hipx固定系(与hipx关节坐标系共原点，各坐标轴与身体坐标系平行)
T01 = [1       0       0       0;
       0    cos(q0)  -sin(q0)  0;
       0    sin(q0)   cos(q0)  0;
       0       0        0      1];
%hipy rt. hipx   
T12 = [cos(q1)    0     sin(q1)    0;
          0       1        0       hipy_to_hipx;
       -sin(q1)   0     cos(q1)    0;
          0       0        0       1];
%knee rt. hipy   
T23 = [cos(q2)    0      sin(q2)    0;
          0       1         0       0;
       -sin(q2)   0      cos(q2)    -knee_to_hipy;
          0       0         0       1];
%foot rt. knee      
T34 = [1    0      0    0;
       0    1      0    0;
       0    0      1    -foot_to_knee;
       0    0      0    1];
   
T04 = T01*T12*T23*T34;
xyz = T04(1:3,4:4) % 取1-3行的第4-4列
%%
%forward kinematics test
endpoint = eval(subs(xyz, ...
          {hipy_to_hipx,knee_to_hipy,foot_to_knee, ...
           q0,q1,q2},...
          {0.037,0.25, 0.25,deg2rad(2.45), deg2rad(37.8), deg2rad(51.4) ...
           }))
%%
%-----------------------Jacobian------------------------------------%
% 3维雅可比
Jac = jacobian(T04(1:3,4:4),[q0;q1;q2]);
% 6维雅可比
T02 = T01*T12;
T03 = T01*T12*T23;
Jw_trans = [[T01(1:3, 1:1)]';[T02(1:3, 2:2)]';[T03(1:3, 2:2)]'];
Jac_6D = [jacobian(T04(1:3,4:4),[q0;q1;q2]);
           Jw_trans'];


%%
%-------------------inverse kinematics---------------------%
simplify(xyz(1,1)^2 + xyz(2,1)^2 + xyz(3,1)^2)
q2 = acosd((x*x + y*y + z*z - hipy_to_hipx^2 - knee_to_hipy^2 - foot_to_knee^2)/(2*knee_to_hipy*foot_to_knee));
%机器人学导论88页公式
q0 = 2*atand((z+(y*y+z*z-hipy_to_hipx*hipy_to_hipx)^0.5)/(y+hipy_to_hipx));
a = foot_to_knee*sind(q2);
b = knee_to_hipy+foot_to_knee*cosd(q2);
c = -x;
q1 = 2*atand((b-(b*b+a*a-c*c)^0.5)/(a+c));
ikine = [q0;q1;q2];

eval(subs(ikine, ...
          {hipy_to_hipx,knee_to_hipy,foot_to_knee, ...
           x,y,z},...
          {0.037,0.25,0.25, endpoint(1,1), endpoint(2,1), endpoint(3,1)...
           }))




