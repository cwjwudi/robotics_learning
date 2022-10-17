clear
clc
load('tra.mat')
P=qF;
V=dqF;
A=ddqF;


Ja=0.21;Ba=14.55;Ga=12;f=4;%实际系统的参数
for k=1:length(P)
    tor(k)=Ja*A(k)+Ba*V(k)+Ga*sin(P(k))+f*sign(V(k))+2*normrnd(0,1);
end


save('1dofRobot.mat','P','V','A','tor')
