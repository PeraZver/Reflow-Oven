% Pero 2015
% Interpolating reflow oven measurement results

clear all
close all
clc

TempTest = importdata('TempTest.txt');
time = TempTest(:,1);
temp = TempTest(:,2);

% Filtering
a = 1;
N = 10;
b = ones(1,N)/N;
Ty = filter(b,a, temp);   % Moving average filter
plot(time, temp, time, Ty);  
grid;

%% Interpolating signal
t1 = temp(1);
Y0 = max(Ty)-t1;
Ts = 200;
Td = 0;
yt = Y0*(1-exp(-(time-Td)/Ts)) + t1;

plot(time, Ty, time, yt);
ylim([min(temp) max(Ty)+10]);
grid

%% Transfer function
s = tf('s');
K = Y0/660/0.5;  % 10 % Duty cycle, power 660W

G = K/(Ts*s+1)*exp(-Td*s);
[y, t]=step(G);
fig = figure;
plot(t,y);
grid;
% hold on
% [x_infl, y_infl] = getpts(fig);
% plot(x_infl,y_infl,'ro');


%% PID regulator by Ziegler Nichols

Kc = 1.2*Ts/Td/K;
Ti = 2*Td;
TD = 0.5*Td;

P = Kc;
I = Kc/Ti;
D = Kc*Td;

PID = pid(3000, 100, 0);
T = feedback(PID*G,1);
1
t = 0:0.1:120;
step(T,t)