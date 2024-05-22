clear; close all; clc;
m=.039;
L_R=.28;
L_P=.11;
g=9.81;
J=.0201;
KK=0.588/7.8;
Ra=3;
b0=.1;

% X1=theta
% X2=theta_dot
% X3=beta
% X4=beta_dot

A1=g*(J+((L_R^2)*m*m))/(L_P*J);    
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
C=[1 0 0 0];
D=0;
sys1=ss(A,B,C,D);
%%
rlocus(sys1);
pause(30);
%%
Q = [1 0 0 0;
    0 0 0 0;
    0 0 1 0;
    0 0 0 0];
R = 10;
K=lqr(A,B,Q,R);

A1=A-(B*K);
sys2=ss(A1,B,C,D);
rlocus(sys2);
pause(5);
%%
theta_pid = pidtune(sys1,'PID');
Kp=theta_pid.Kp;
Kd=theta_pid.Kd;
Ki=theta_pid.Ki;

C= pid(Kp,Ki,Kd);
sys3 = feedback(sys1,C);
rlocus(sys3);