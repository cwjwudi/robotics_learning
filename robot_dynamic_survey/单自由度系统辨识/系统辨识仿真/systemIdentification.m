clear
clc
load('1dofRobot.mat')

N=length(P);
dt=1/150;
t=0:dt:(N-1)*dt;
t=t';
fs=1/dt;
omega=10/(fs/2);
N=2;
[b,a]=butter(N,omega); %设置滤波器参数

hipMea=P';
velMea=(hipMea(2:end)-hipMea(1:end-1))/dt; % 差分求解速度
velMea=[velMea(1);velMea];  % 扩充第一个缺失的数据
filterdVel=filtfilt(b,a,velMea); % 离线滤波器
accMea=(filterdVel(2:end)-filterdVel(1:end-1))/dt; % 差分求解加速度
accMea=[accMea(1);accMea];  % 扩充第一个缺失的数据
filterdAcc=filtfilt(b,a,accMea);
filterdTor=filtfilt(b,a,tor); % tor是实际测得的
filterdTor=filterdTor';

plot(filterdAcc(20:end)-A(20:end)')
plot(filterdVel(20:end)-V(20:end)')

phi=[filterdAcc,1*filterdVel,1*sign(filterdVel),cos(hipMea),sin(hipMea)];

j=1;
for i=1:length(filterdVel)
  if(abs(filterdVel(i))>0.01)
    phiFinal(j,:)= phi(i,:);
    torFinal(j,:)= filterdTor(i,:);
    j=j+1;
  end
end

theta=pinv(phi'*phi)*phi'*filterdTor;  % pinv 求广义逆矩阵
estTor=phi*theta;
err=estTor-filterdTor;

figure(1)
subplot(2,1,1)
plot(filterdTor,'r','LineWidth',2)
hold on
plot(estTor,'LineWidth',2)
subplot(2,1,2)
plot(err,'LineWidth',2)



