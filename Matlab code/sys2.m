clear; close all; clc;
m=.039;
L_R=.28;
L_P=.11;
g=9.81;
J=.0201;
KK=0.588/7.8;
Ra=3;
b0=.1;

n=0;

% X1=theta
% X2=theta_dot
% X3=beta
% X4=beta_dot

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
C=[1 0 0 0];
D=0;
sys=ss(A,B,C,D);
%%  Design LQR controller
C_pid = pidtune(sys,'PID');
Kp=C_pid.Kp;
Kd=C_pid.Kd;
Ki=C_pid.Ki;

C = pid(Kp,Ki,Kd);
T = feedback(sys,C);

% Q = [1 0 0 0;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% R = 100;
% K=lqr(A,B,Q,R);

%% Simulate closed-loop system
dt=0.0001;
tspan = 0:dt:2;
x0 = [pi+.5; 0; 0;  0];  % initial condition 
wr = [pi; 0; 0; 0];      % reference position
Ea=@(x)-K*(x - wr);       % control law

% 
% for i=1:length(tspan)
%     t(1)=i*dt;
%     t(2)=i*dt+dt;
[t,x] = ode45(@(t,x)noneliniar_sys(x,t,m,J,L_R,L_P,g,KK,Ra,b0,Ea(x)),tspan,x0);
% end







% previous_error= 0;
% integral= 0;
% 
% 
%     error = setpoint - measured_value;
%     integral = integral + error * dt;
%     derivative = (error - previous_error) / dt;
%     output = Kp * error + Ki * integral + Kd * derivative;
%     previous_error = error;
%     wait(dt);




















for k=1:100:length(t)
    print_sys(x(k,:),1,5,2,L_R);
end

%%
plot(t,x,'LineWidth',2);
l1 = legend('\theta','\omega','x','v');
xlabel('Time')
ylabel('State')
grid on