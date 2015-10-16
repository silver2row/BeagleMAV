%% messing around with converting 3DR Iris PID constants
%% into more useful discrete time filter with similar dynamics

clear
close all
clc

%% iris gains
kp = .05;
ki = 0.001;
kd = .004;
dt = .005;

P = tf([kp],[1])
I = tf([0,ki],[1,0])
D = tf([kd,0],[1])
PIDc = P + I + D



%% discrete controller
kp = .05;
ki = 0.00;
kd = .004;
Tf = .015;

if(ki==0)
num = [(kp*Tf+kd)/Tf, -(((ki*dt-kp)*(dt-Tf))+kd)/Tf];
den = [1, -(Tf-dt)/Tf];
else
num = [(kp*Tf+kd)/Tf, (ki*dt*Tf + kp*(dt-Tf) - kp*Tf - 2*kd)/Tf, (((ki*dt-kp)*(dt-Tf))+kd)/Tf];
den = [1, (dt-2*Tf)/Tf, (Tf-dt)/Tf];
end

PIDd = tf(num,den, dt)
 PIDm = pid(kp,ki,kd,Tf,dt);
 
 
 tf(PIDm)




figure()
hold on
bode(PIDm)
bode(PIDc)
bode(PIDd)
handler = gcr; % Assign the plot object root to handler
               % Plot object root 'gcr' holds the properties of the plot being produced
%handler.AxesGrid.Xunits = 'Hz'; % Change rad s-1 to Hz
handler.AxesGrid.Yunits = {'abs','deg'}; % Set mag axis to abs &amp; phase axis to deg


legend('built in pid gen','continuous','discrete')
% 
% step = ones(1,50);
% response = filter(num,den,step);
% plot(response)
% axis([0,50,-.2, 2])
% response



