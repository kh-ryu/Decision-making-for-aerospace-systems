close all; clear all; clc;

% Setting
N = 10;
R = 50;
dt = 0.01;
tmax = 60;
t = 0:dt:tmax;
drone=cell(1,10);

%% (b) - (i)

% (i)
for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if j ~= i
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
                A(i,j) = 1;
                D(i,j) = 0;
            else
                A(i,j) = 0;
                D(i,j) = N-1;
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_i_traj_first');

L = D-A;
fprintf('Laplacian matrix (b)-(i) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

for i = 1:N
    XYCoords(i,:) = drone{i}.position(:,1).'; 
end

figure;
gplot(A,XYCoords,'-*')
figSave('b_i_graph_first');

save L_bi1.mat L
%% (b) - (ii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end
%{
while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 5
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end
%}

A = [0 1 1 1 1 1 0 0 0 0
     1 0 1 1 1 1 0 0 0 0
     1 1 0 1 1 1 0 0 0 0
     1 1 1 0 1 1 0 0 0 0
     1 1 1 1 0 1 0 0 0 0
     1 1 1 1 1 0 0 0 0 0
     0 0 0 0 1 1 0 1 1 1
     0 0 0 0 1 1 1 0 1 1
     0 0 0 0 1 1 1 1 0 1
     0 0 0 0 1 1 1 1 1 0];

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
%figSave('b_ii_traj_first');

fprintf('Laplacian matrix (b)-(ii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');
%{
figure;
gplot(A,XYCoords,'-*')
figSave('b_ii_graph_first');

save L_bii1.mat L
%}
%% (b) - (iii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 3
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_iii_traj_first');

fprintf('Laplacian matrix (b)-(iii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

figure;
gplot(A,XYCoords,'-*')
figSave('b_iii_graph_first');

save L_biii1.mat L

%% second
close all; clear all;

% Setting
N = 10;
R = 50;
dt = 0.01;
tmax = 60;
t = 0:dt:tmax;
drone=cell(1,10);

%% (b) - (i)

% (i)
for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if j ~= i
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
                A(i,j) = 1;
                D(i,j) = 0;
            else
                A(i,j) = 0;
                D(i,j) = N-1;
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_i_traj_second');

L = D-A;
fprintf('Laplacian matrix (b)-(i) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

for i = 1:N
    XYCoords(i,:) = drone{i}.position(:,1).'; 
end

figure;
gplot(A,XYCoords,'-*')
figSave('b_i_graph_second');

save L_bi2.mat L
%% (b) - (ii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 5
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_ii_traj_second');

fprintf('Laplacian matrix (b)-(ii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

figure;
gplot(A,XYCoords,'-*')
figSave('b_ii_graph_second');

save L_bii2.mat L
%% (b) - (iii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 3
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_iii_traj_second');

fprintf('Laplacian matrix (b)-(iii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

figure;
gplot(A,XYCoords,'-*')
figSave('b_iii_graph_second');

save L_biii2.mat L

%% Third
close all; clear all;

% Setting
N = 10;
R = 50;
dt = 0.01;
tmax = 60;
t = 0:dt:tmax;
drone=cell(1,10);

%% (b) - (i)

% (i)
for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if j ~= i
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
                A(i,j) = 1;
                D(i,j) = 0;
            else
                A(i,j) = 0;
                D(i,j) = N-1;
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_i_traj_third');

L = D-A;
fprintf('Laplacian matrix (b)-(i) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

for i = 1:N
    XYCoords(i,:) = drone{i}.position(:,1).'; 
end

figure;
gplot(A,XYCoords,'-*')
figSave('b_i_graph_third');

save L_bi3.mat L
%% (b) - (ii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 5
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_ii_traj_third');

fprintf('Laplacian matrix (b)-(ii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

figure;
gplot(A,XYCoords,'-*')
figSave('b_ii_graph_third');

save L_bii3.mat L

%% (b) - (iii)

for i = 1:N
    drone{i}.position = zeros(2,length(t));
    drone{i}.position(:,1) = [20*rand(1)-10;20*rand(1)-10]; 
    drone{i}.velocity = zeros(2,length(t)-1);
end

while 1
    A = randi([0,1],10,10);
    for i = 1:N % adjacent
        for j = 1:N
            if j > i
                A(j,i) = A(i,j);
            elseif j == i
                A(i,j) = 0;
            end
        end
    end
    n = zeros(1,10);
    for i = 1:N
        if sum(A(i,:)) == 3
            n(i) = 1;
        end
    end
    if prod(n) == 1
        break;
    end
end

for i = 1:N
    d(i,1) = sum(A(i,:));
end
D = diag(d);

L = D-A;

for k = 1:length(t)-1
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                drone{i}.velocity(:,k) = drone{i}.velocity(:,k)...
                    -((2*R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))/(R-norm(drone{i}.position(:,k)-drone{j}.position(:,k)))^2)*(drone{i}.position(:,k)-drone{j}.position(:,k));
            end
        end
    end
    for i = 1:N
        drone{i}.position(:,k+1) = drone{i}.position(:,k)+dt*drone{i}.velocity(:,k);
    end
end

figure;
for i = 1:N
    plot(drone{i}.position(1,:),drone{i}.position(2,:))
    hold on
end
legend({'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10'},'FontSize',6,'Location','northEast','NumColumns',5);
legend('boxoff');
figSave('b_iii_traj_third');

fprintf('Laplacian matrix (b)-(iii) \n');
for i = 1:N
    for j = 1:N
        fprintf('%d\t',L(i,j));
    end
    fprintf('\n');
end
fprintf('\n');

figure;
gplot(A,XYCoords,'-*')
figSave('b_iii_graph_third');

save L_biii3.mat L


%% Functions
function figSave(Title)
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    print(Title,'-djpeg','-r1200')
end