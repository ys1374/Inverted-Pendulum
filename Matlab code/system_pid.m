clear; close all; clc;
m=.039;
L_R=.28;
L_P=.11;
g=9.81;
J=.0201;
KK=0.588/7.8;
Ra=3;
b0=.1;

A1=g*(J+((L_R^2)*m))/(L_P*J);    
A2=L_R*((Ra*b0)+(KK^2))/(L_P*Ra*J);
A3=-g*m*L_R/J;
A4=-((Ra*b0)+(KK^2))/(Ra*J);
B1=KK*L_R/(L_P*Ra*J);
B2=-KK/(Ra*J);

A = [0 1 0 0;
    A1 0 0 A2; 
    0  0 0 1;
    A3 0 0 A4];

B = [0; B1; 0; B2;];

% C=[1 0 0 0];
% D=0;
C_theta = [1 0 0 0];
D = 0;
% X1=theta
% X2=theta_dot
% X3=beta
% X4=beta_dot
states = {'theta' 'theta_dot' 'beta' 'beta_dot'};
inputs = {'V'};
outputs = {'theta'};
sys_ss_theta = ss(A,B,C_theta,D,'statename',states,'inputname',inputs,'outputname',outputs);
%%  Design PID controller
theta_pid = pidtune(sys_ss_theta,'PID');
Kp=theta_pid.Kp;
Kd=theta_pid.Kd;
Ki=theta_pid.Ki;

C= pid(Kp,Ki,Kd);
T = feedback(sys_ss_theta,C);


step(T);
pause(5);
t=0:0.001:.5;
impulse(T,t)