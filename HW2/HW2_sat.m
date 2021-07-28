clear all; close all; clc;

% simulation setting
tspan = 0.01;
t = 0:tspan:60;
N = size(t,2);

% desired location
w = 0.1;
r = 10;
y_r = [r*cos(w*t); r*sin(w*t)];
y_r_dot = [-w*r*sin(w*t); w*r*cos(w*t)];
y_r_2dot = [-w^2*r*cos(w*t); -w^2*r*sin(w*t)];

x = zeros(3,N);
x(:,1) = [0; 0; 0];
y = x(1:2,:);
y_dot = zeros(2,N);

e = zeros(2,N);
e_dot = zeros(2,N);
s = zeros(2,N);
v = zeros(2,N);
B = zeros(2,2,N);
u1 = zeros(1,N);
u1(1) = 10;
u2 = zeros(1,N);
u1_dot = zeros(1,N);

%parameter setting
k1 = 5;
k2 = 5;
K = [k1, 0; 0, k2];
lambda = 3;

for i = 1:N
    e(:,i) = y(:,i) - y_r(:,i);
    y_dot(:,i) = [u1(i)*cos(x(3,i)); u1(i)*sin(x(3,i))];
    e_dot(:,i) = y_dot(:,i) - y_r_dot(:,i);
    s(:,i) = e_dot(:,i) + lambda*e(:,i);
    
    v(:,i) = -y_r_2dot(:,i) + lambda*e_dot(:,i);
    B(:,:,i) = [cos(x(3,i)), -u1(i)*sin(x(3,i))
                          sin(x(3,i)), u1(i)*cos(x(3,i))]; 
            
    u = inv(B(:,:,i))*(-v(:,i) - K*[sat(s(1,i)); sat(s(2,i))]);
    u1_dot(i) = u(1); 
    u2(i) = u(2);
   
    if u1_dot(i)>5
        u1_dot(i) = 5;
    else
        if u1_dot(i)<-5
            u1_dot(i) = -5;
        end
    end
    
    if u2(i)>1
        u2(i) = 1;
    else
        if u2(i)<-1
            u2(i) = -1;
        end
    end
    
    % update
    u1(i+1) = u1(i) + tspan*u1_dot(i);
    x(1,i+1) = x(1,i) + tspan*u1(i)*cos(x(3,i));
    x(2,i+1) = x(2,i) + tspan*u1(i)*sin(x(3,i));
    x(3,i+1) = x(3,i) + tspan*(u2(i));
    y(1,i+1) = x(1,i+1);
    y(2,i+1) = x(2,i+1);
    
    if u1(i+1)>10
        u1(i+1) = 10;
    else
        if u1(i+1)<-10
            u1(i+1) = -10;
        end
    end
end

%% plots
figure(1);
plot(t,u1(1:end-1))
hold on
plot(t,u1_dot)
plot(t,u2)
title('Control input');
xlabel('time [sec]');
ylabel('u');
legend('u1','u1 dot','u2');

figure(2);
hold on
plot(y_r(1,:),y_r(2,:),'--')
plot(y(1,:),y(2,:))
grid on
hold off
title('Trajectory');
xlabel('x');
ylabel('y');
xlim([-15 15]); ylim([-15 15]);
legend('reference','unicycle');

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