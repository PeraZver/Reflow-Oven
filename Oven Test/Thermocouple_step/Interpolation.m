% Pero 2015
% Interpolating reflow oven measurement results

clear all
close all
clc

TempTest = importdata('TempTest_dc10percent.txt');
time = TempTest(:,1);
temp = TempTest(:,2);

% Filtering
a = 1;
N = 30;
b = ones(1,N)/N;
Ty = filter(b,a, temp);   % Moving average filter
plot(time, temp, time, Ty);  
grid;

%% Interpolating signal
Y0 = 88-temp(1);
Ts = 470;
Td = 70;
yt = Y0*(1-exp(-(time-Td)/Ts)) + temp(1);

plot(time, Ty, time, yt);
ylim([28 110]);
grid

%% Transfer function
s = tf('s');
K = Y0/660/0.1;  % 10 % Duty cycle, power 660W

G = K/(Ts*s+1)*exp(-Td*s);
[y, t]=step(G);
fig = figure;
plot(t,y);
grid;
hold on
[x_infl, y_infl] = getpts(fig);
plot(x_infl,y_infl,'ro');


%% PID regulator by Ziegler Nichols

Kc = 1.2*Ts/Td/K;
Ti = 2*Td;
TD = 0.5*Td;

P = Kc;
I = Kc/Ti;
D = Kc*Td;

PID = pid(P,I,D);
T = feedback(PID*G,1);

t = 0:0.1:1000;
step(T,t)