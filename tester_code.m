
close all; 

dh = [1 0 0 0 0 ; 1 0 0 0 0 ; 1 0 0 0 0];
qdes = [-0.5;1;0.75];
xdes = [1; 0.5; 0];
syms time;
xdes_sym = [2*exp(-time) ; 1*cos(pi*time/5) ; 0];
fdes_sym = [10*sin(pi*time/2) ; 15*cos(pi*time/10); 0 ; 0; 0; 0];
fdes = [0;0;0;0;0;0];
% xdes_sym = [1.3; 0.2; 0];
% fdes_sym = [10; 5; 2; 0; 0; 0];
q0 = [-pi/4;pi/2;pi/3]; qd0 = [0; 0; 0];
T = 10; 
% For compliance control: Kp~1000 , Kd~800 
% For impedance control: Kp~40, Kd~20, Md~10
Kp = 40; Kd = 20; Md = 10;
g0 = [0; 0; 0];
mL = [50 50 50]/10; Im = [0.03 0.03 0.03]; kr = [110 110 110];
Il = zeros([3,3,3]);
il = [0 0 0 ; 0 0 0 ; 0 0 11]; Il(:,:,1) = il; Il(:,:,2) = il; Il(:,:,3) = il;
Fv = zeros(3,1); Fs = zeros(3,1); mm = zeros(3,1);

% [t,q,qd] =  mjctrl_PDGrav(dh,qdes,q0,qd0,T,Kp,Kd,g0,mL,mm,Il,Im,kr,Fv,Fs);

% [t,q,qd,qdd,xe] = moctrl_PDGrav(dh,xdes,q0,qd0,T,Kp,Kd,g0,mL,mm,Il,Im,kr,Fv,Fs);

% [t,q,qd,qdd,xe] = moctrl_InvDyn(dh,xdes_sym,q0,qd0,T,Kp,Kd,g0,mL,mm,Il,Im,kr,Fv,Fs);

[t,q,qd,qdd,xe,fe,tau] = foctrl_Imp(dh,xdes_sym,fdes_sym,q0,qd0,T,Kp,Kd,Md,g0,mL,mm,Il,Im,kr,Fv,Fs);

% [t,q,qd,qdd,xe,fe,tau] = foctrl_Comp(dh,xdes,fdes,q0,qd0,T,Kp,Kd,g0,mL,mm,Il,Im,kr,Fv,Fs);

figure; subplot(3,1,1); plot(t,q); subplot(3,1,2); plot(t,qd); subplot(3,1,3); plot(t,qdd);

figure; plot(t,xe,'b-');
hold on; 
fplot(xdes_sym,[0 T],'r--');
% plot(t,xdes.*ones([3,length(t)]),'r--');

figure; plot(t,fe);

figure; plot(t,tau);

