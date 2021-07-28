clear all; close all; clc;

% simulation setting
tspan = 0.01;
t = 0:tspan:500;
N = size(t,2);

% variable
gamma = 1;
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
u_bar = zeros(1,N);
s = zeros(1,N);
del_s = zeros(1,N);
f = zeros(1,N);
g = zeros(1,N);
gamma_hat = 0.8*ones(1,N);

% control parameter
k = 10;
k_gamma = 0.1;
lambda = 3;
phi = 1;

for i = 1:N
    y(i) = x(1,i) - theta(i);
    y_dot(i) = (1/gamma_hat(i))*sin(x(2,i)) - theta_dot(i);
    y_ddot(i) = (1/gamma_hat(i))*x(3,i)*cos(x(2,i)) - theta_ddot(i);
    if i~=1
        for j = 2:i
            int_y(i) = int_y(i) + tspan*y(j-1);
        end
    end
    s(i) = y_ddot(i) + 3*lambda*y_dot(i) + 3*lambda^2*y(i) + lambda^3*int_y(i);
    f(i) = -a*(1/gamma_hat(i))*x(3,i)*cos(x(2,i)) - b*(1/gamma_hat(i))*x(2,i)*cos(x(2,i)) - (1/gamma_hat(i))*x(3,i)^2*sin(x(2,i)) - theta_dddot(i) + 3*lambda*y_ddot(i) + 3*lambda^2*y_dot(i) + lambda^3*y(i);
    g(i) = b*(1/gamma_hat(i))*cos(x(2,i));
    
    u(i) = 1/g(i)*(-f(i) - k*sat(s(i)/phi));
    
    if u(i)>10
        u(i) = 10;
    else
        if u(i)<-10
            u(i) = -10;
        end
    end
    u_bar(i) = u(i)*g(i);
    
    % update
    x(3,i+1) = x(3,i) + tspan*(-a*x(3,i) - b*x(2,i) + b*u(i));
    x(2,i+1) = x(2,i) + tspan*x(3,i);
    x(1,i+1) = x(1,i) + tspan*(gamma*sin(x(2,i)));
    
    gamma_hat(i+1) = gamma_hat(i) + tspan*(-k_gamma*s(i)*u_bar(i));
    if gamma_hat(i+1)>1.5
        gamma_hat(i+1) = 1.5;
    else
        if gamma_hat(i+1)<0.5
            gamma_hat(i+1)=0.5;
        end
    end
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

figure(3)
hold on
plot(t,1./gamma_hat(1:end-1))
plot(t,ones(1,size(t,2)),'--')
title('mu estimation');
xlabel('time [sec]');
ylabel('mu');

%% functions
function output = sat(q)
output = zeros(size(q,1),1);
for i = 1:size(q,1)
    if q(i,1) > 1
        output(i,1) = 1;
    elseif q(i,1) < -1
        output(i,1) = -1;
    else
        output(i,1) = q(i,1);
    end
end
end