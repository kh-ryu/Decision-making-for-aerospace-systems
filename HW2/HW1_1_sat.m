clear all; close all; clc;

% simulation setting
tspan = 0.01;
t = 0:tspan:60;
N = size(t,2);

% variable
mu = 1;
xi = 2;
w_n = 2;

a = xi*w_n;
b = w_n^2;

% desired theta
% theta = sin(w*t)
w = 0.1;
theta = sin(w*t)+5;
theta_dot = w*cos(w*t);
theta_ddot = -w^2*sin(w*t);
theta_dddot = -w^3*cos(w*t);

x = zeros(3,N);
y = zeros(1,N);
y_dot = zeros(1,N);
y_ddot = zeros(1,N);
y_dddot = zeros(1,N);
int_y = zeros(1,N);
u = zeros(1,N);
s = zeros(1,N);
f = zeros(1,N);
g = zeros(1,N);

% control parameter
k = 10;
lambda = 3;
phi = 2;

for i = 1:N
    y(i) = x(1,i) - theta(i);
    y_dot(i) = mu*sin(x(2,i)) - theta_dot(i);
    y_ddot(i) = mu*x(3,i)*cos(x(2,i)) - theta_ddot(i);
    s(i) = y_ddot(i) + 2*lambda*y_dot(i) + lambda^2*y(i);
    f(i) = -a*mu*x(3,i)*cos(x(2,i)) - b*mu*x(2,i)*cos(x(2,i)) - mu*x(3,i)^2*sin(x(2,i)) - theta_dddot(i) + 2*lambda*y_ddot(i) + lambda^2*y_dot(i);
    g(i) = b*mu*cos(x(2,i));
    
    if s(i)>phi
        u(i) = (1/g(i))*(-f(i) - k);
    else
        if s(i)<-phi
            u(i) = (1/g(i))*(-f(i) + k);
        else
            u(i) = (1/g(i))*(-f(i) - k*s(i)/phi);
        end
    end
    if u(i)>10
        u(i) = 10;
    else if u(i)<-10
            u(i) = -10;
        end
    end
    
    % update
    x(3,i+1) = x(3,i) + tspan*(-a*x(3,i) - b*x(2,i) + b*u(i));
    x(2,i+1) = x(2,i) + tspan*x(3,i);
    x(1,i+1) = x(1,i) + tspan*(mu*sin(x(2,i)));
    
end

%% plots
figure(1);
plot(t,u)
title('Control input');
xlabel('time [sec]');
ylabel('u');

figure(2);
subplot(2,1,1)
hold on
plot(t,theta,'--')
plot(t,x(1,1:end-1))
grid on
hold off
title('Roll angle');
xlabel('time [sec]');
ylabel('theta');

subplot(2,1,2)
plot(t,x(2,1:end-1))
grid on
title('Thruster angle');
xlabel('time [sec]');
ylabel('delta');
