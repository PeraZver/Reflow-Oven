% From http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID


clear all
close all
clc

% Process parameters
T = 200;        % Time constant of the oven, sec
tau = 0;      % Delay time, sec
K = 0.37;        % Process gain, K/W

s = tf('s');
P = K/(T*s+1)*exp(-tau*s);
step(P);

%% Ziegler-Nichols method
% PID controller

ni = tau/T;
a = ni*K;           % Z-N parameter
Kp = 0.9/a;         % 0.9/a;    % PI regulator
Ki = Kp/2/tau;      % Kp/3/tau; % PI regulator
Kd = Kp*tau/2;      % 0;        % PI regulator
C = pid(Kp,Ki,Kd);
Loop = feedback(C*P,1);

t = 0:1:600;
step(Loop,t)
%% Cohen-Coon method
% PID controller

Kp = 1/K*(0.25+1.35/ni);                    
Ki = Kp/((2.5+0.46*ni)*tau/(1+0.61*ni));
Kd = Kp*(0.37*tau/(1+0.19*ni));

% PI controller
% Kp = 1/K*(0.083+0.9/ni);                    
% Ki = Kp/((3.3+0.31*ni)*tau/(1+2.2*ni));
% Kd = 0; 

C = pid(Kp,Ki,Kd);
Loop = feedback(C*P,1);

t = 0:1:600;
step(Loop,t)

%% PIDTool
%pidtool(P,C)
%% Optimization
opts = pidtuneOptions('CrossoverFrequency',10,'PhaseMargin',90);
[C, info] = pidtune(P, 'pid', opts);

PID = pid(C.Kp,C.Ki/2,C.Kd);
Loop = feedback(PID*P,1);

t = 0:1:200;
step(Loop,t)
