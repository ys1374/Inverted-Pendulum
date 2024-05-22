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
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
% X1=theta
% X2=theta_dot
% X3=beta
% X4=beta_dot
states = {'theta' 'theta_dot' 'beta' 'beta_dot'};
inputs = {'V'};
outputs = {'theta'; 'beta'};

%%  Design LQR controller
Q = [1 0 0 0;
    0 0 0 0;
    0 0 1 0;
    0 0 0 0];
R = 10;
K=lqr(A,B,Q,R);

A1=A-(B*K);
sys_ss = ss(A1,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
%% Simulate closed-loop system
tspan = 0:.0001:2;
x0 = [pi+.5; 0; 0; 0];  % initial condition 
wr = [pi; 0; 0; 0];      % reference position
Ea=@(x)-K*(x - wr);       % control law
[t,x] = ode45(@(t,x)noneliniar_sys(x,m,J,L_R,L_P,g,KK,Ra,b0,Ea(x)),tspan,x0);

for k=1:100:length(t)
    print_sys(x(k,:),1,5,2,L_R);
end

%%
plot(t,x,'LineWidth',2);
l1 = legend('\theta','\omega','\beta','\beta-dot');
xlabel('Time')
ylabel('State')
grid on
pause(5);
step(sys_ss);
pause(5);
t=0:0.01:10;
impulse(sys_ss,t)