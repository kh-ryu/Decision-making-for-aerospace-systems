%% clear
clear var
clear
clc

%% Draw a map
map_mat = [0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 1 1
           0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1
           0 0 0 0 1 0 0 0 1 0 0 1 1 1 1 1 1 0 0 0 0 1
           0 0 0 0 1 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 0 1
           0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1
           0 0 0 1 1 1 0 0 0 0 0 1 0 0 0 1 1 0 0 0 0 1
           0 1 1 1 1 1 0 0 0 0 0 1 0 0 0 1 1 0 0 0 0 1
           0 0 0 1 1 1 0 0 1 0 0 1 0 0 0 1 1 1 1 1 1 1
           0 0 0 1 0 0 0 0 1 0 0 1 0 0 0 1 1 0 0 0 0 0
           0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 1 1 0 0 0 0 0
           0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 1 1 1 1 1 0 0
           0 0 0 1 0 0 0 1 1 0 0 1 0 0 0 0 0 1 0 0 0 0
           0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0
           0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0
           0 0 0 1 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0];
       
map = binaryOccupancyMap(map_mat);
show(map)
hold on
% 시작, 목표
start = [1.5,14.5];
goal = [18.5,6.5]; 

n = 1000;
V = struct('coord',[1.5 14.5], 'parent',0);
q_new = struct('coord',[1.5 14.5], 'parent',0);
del_theta = zeros(1);

%% Kinematic model
kinematicModel = unicycleKinematics;
% start and goal state
initialState = [start,-pi/2];
V(1).coord = initialState;
q_goal.coord = [goal, -pi/2];

%time setting for ODE solver
tspan = 0:0.05:1;


%% RRT

iteration = 1;
while(1)

    % pick a point randomly
    q_rand = [22*rand(1) 15*rand(1) 2*pi*rand-pi];
    
    % bias for fast convergence
    % when tree gets near to the goal, q_rand is set near the goal
    if 17 < q_new.coord(1) && 5 < q_new.coord(2) && q_new.coord(2) < 7
        q_rand = [17+5*rand(1), 5+2*rand(1), 2*pi*rand-pi];
    end
            
    exit = 0;    
    if checkOccupancy(map, q_rand(1:2))==0   %check q_rand is in free configuration
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
        drawnow
        
        %find nearest node
        ndist = [];
        for j = 1:size(V,2)
            n = V(j).coord(1:3);
            tmp = norm(n(1:2)-q_rand(1:2)) + norm(sin(n(3)-q_rand(3)));
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = V(idx).coord;
        
        % calculate angle
        if q_rand(1)-q_near(1)>0
            angle = atan((q_rand(2)-q_near(2))/(q_rand(1)-q_near(1)));
        elseif q_rand(1)-q_near(1)<0 && q_rand(2)-q_near(2)<0
            angle = atan((q_rand(2)-q_near(2))/(q_rand(1)-q_near(1))) - pi;
        elseif q_rand(1)-q_near(1)<0 && q_rand(2)-q_near(2)>0
            angle = atan((q_rand(2)-q_near(2))/(q_rand(1)-q_near(1))) + pi;
        elseif q_rand(1)-q_near(1)==0 && q_rand(2)-q_near(2)>=0
            angle = pi/2;
        elseif q_rand(1)-q_near(1)==0 && q_rand(2)-q_near(2)<0
            angle = -pi/2;
        end
        del_theta(iteration) = angle-q_near(3);
        
        % steer (try to extend tree to q_rand)
        % vehicle의 시야에서 q_rand의 각도에 따라 input 달라짐
        if cos(del_theta(iteration)) > 1/sqrt(2)
            inputs = [10 0]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_near);
            q_new.coord = y(21,:);
            q_new.parent = idx;
            if any(checkOccupancy(map,y(:,1:2))) ~=0
                exit = 1;
            end
        elseif cos(del_theta(iteration)) > -1/sqrt(2) && cos(del_theta(iteration)) < 1/sqrt(2) && sin(del_theta(iteration))>0 
            inputs = [10 pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_near);
            q_new.coord = y(21,:);
            q_new.parent = idx;
            if any(checkOccupancy(map,y(:,1:2))) ~=0
                exit = 1;
            end
        elseif -1<cos(del_theta(iteration)) && cos(del_theta(iteration)) < -1/sqrt(2) && sin(del_theta(iteration))>0
            inputs = [0 pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_near);
            q_new.coord = y(21,:);
            q_new.parent = idx;
            if any(checkOccupancy(map,y(:,1:2))) ~=0
                exit = 1;
            end
        elseif cos(del_theta(iteration)) > -1/sqrt(2) && cos(del_theta(iteration)) < 1/sqrt(2) && sin(del_theta(iteration))<0
            inputs = [10 -pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_near);
            q_new.coord = y(21,:);
            q_new.parent = idx;
            if any(checkOccupancy(map,y(:,1:2))) ~=0
                exit = 1;
            end
        elseif -1<cos(del_theta(iteration)) && cos(del_theta(iteration)) < -1/sqrt(2) && sin(del_theta(iteration))<0
            inputs = [0 -pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_near);
            q_new.coord = y(21,:);
            q_new.parent = idx;
            if any(checkOccupancy(map,y(:,1:2))) ~=0
                exit = 1;
            end            
        end
        
        % obstacle check and add
        if exit ==0
            V = [V q_new];
            plot(y(:,1),y(:,2), 'Color',  'g', 'Linewidth',1)
            drawnow
        end
   
    end
    if exit==0
        if q_new.coord(1) < q_goal.coord(1)+0.5 && q_new.coord(1) > q_goal.coord(1)-0.5 && q_new.coord(2) < q_goal.coord(2)+0.5 && q_new.coord(2) > q_goal.coord(2)-0.5
            break
        end
    end
    iteration = iteration +1;
end

%% Find path
% distance to goal
D = [];
for j = 1:1:size(V,2)
    tmpdist = dist(V(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = V(idx);
q_goal.parent = idx;
q_end = q_goal;
V = [V q_goal];

while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), V(start).coord(1)], [q_end.coord(2), V(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    q_end = V(start);
end