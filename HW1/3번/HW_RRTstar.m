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
q_start.coord = [1.5 14.5 -pi/2];
q_start.cost = 0;
q_start.parent = 0;
q_start.input = [0 0];
q_goal.coord = [18.5 6.5 -pi/2];
q_goal.cost = 0;

nodes(1) = q_start;

q_new = struct('coord',[0 0 0],'cost',0,'input',[0 0]);
del_theta = zeros(1);
possible_inputs = [10 pi/2; 10 0; 10 -pi/2];
select_input = 0;

%% Kinematic model
kinematicModel = unicycleKinematics;
% start and goal state
initialState = q_start.coord;
finalState = q_goal.coord;

%time setting for ODE solver
tspan = 0:0.05:1;


%% RRT star
iteration = 1;
while(1)
    
    % pick a point randomly
    q_rand = [22*rand(1) 15*rand(1) 2*pi*rand-pi];
    
    % bias for fast convergence
    % when tree gets near to the goal, q_rand is set near the goal
    if 17 < q_new.coord(1) && 5 < q_new.coord(2) && q_new.coord(2) < 7
        q_rand = [17+5*rand(1), 5+2*rand(1), 2*pi*rand-pi];
    end
        
    if checkOccupancy(map, q_rand(1:2))==0 %check q_rand is in free configuration
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
        drawnow
        
        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:length(nodes)
            n = nodes(j);
            % consider attitude when finding closest node
            tmp = dist(n.coord(1:2), q_rand(1:2)) + norm(sin(q_rand(3)-n.coord(3)));
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_nearest = nodes(idx);
        q_new.parent = idx;
        
        % calculate angle between random point and nearest node
        if q_rand(1)-q_nearest.coord(1)>=0
            angle = atan((q_rand(2)-q_nearest.coord(2))/(q_rand(1)-q_nearest.coord(1)));
        elseif q_rand(1)-q_nearest.coord(1)<0 && q_rand(2)-q_nearest.coord(2)<0
            angle = atan((q_rand(2)-q_nearest.coord(2))/(q_rand(1)-q_nearest.coord(1))) - pi;
        elseif q_rand(1)-q_nearest.coord(1)<0 && q_rand(2)-q_nearest.coord(2)>0
            angle = atan((q_rand(2)-q_nearest.coord(2))/(q_rand(1)-q_nearest.coord(1))) + pi;
        elseif q_rand(1)-q_nearest.coord(1)==0 && q_rand(2)-q_nearest.coord(2)>=0
            angle = pi/2;
        elseif q_rand(1)-q_nearest.coord(1)==0 && q_rand(2)-q_nearest.coord(2)<0
            angle = -pi/2;
        end
        del_theta(iteration) = angle-q_nearest.coord(3);
        
        % steer (try to extend tree to q_rand)
        % vehicle의 시야에서 q_rand의 각도에 따라 input 달라짐
        if cos(del_theta(iteration)) > 1/sqrt(2)
            inputs = [10 0]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_nearest.coord);
        elseif cos(del_theta(iteration)) > -1/sqrt(2) && cos(del_theta(iteration)) < 1/sqrt(2) && sin(del_theta(iteration))>0 
            inputs = [10 pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_nearest.coord);
        elseif -1<cos(del_theta(iteration)) && cos(del_theta(iteration)) < -1/sqrt(2) && sin(del_theta(iteration))>=0
            inputs = [0 pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_nearest.coord);
        elseif cos(del_theta(iteration)) > -1/sqrt(2) && cos(del_theta(iteration)) < 1/sqrt(2) && sin(del_theta(iteration))<0
            inputs = [10 -pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_nearest.coord);
        elseif -1<cos(del_theta(iteration)) && cos(del_theta(iteration)) < -1/sqrt(2) && sin(del_theta(iteration))<0
            inputs = [0 -pi/2]; 
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,q_nearest.coord);
        end
        q_new.coord = y(21,:);
        q_nearest.input = inputs;
        q_new.cost = dist(q_new.coord, q_nearest.coord) + q_nearest.cost; 

            
        if all(checkOccupancy(map,y(:,1:2))==0) % ObstacleFree(q_nearest,q_new)
                        
            % Within a radius of r, find all existing nodes
            q_near = [];
            r = 3;
            neighbor_count = 0;
            for j = 1:length(nodes)
                if dist(nodes(j).coord, q_new.coord) <= r
                    neighbor_count = neighbor_count+1;
                    q_near(neighbor_count).coord = nodes(j).coord;
                    q_near(neighbor_count).cost = nodes(j).cost;
                    q_near(neighbor_count).parent = nodes(j).parent;
                    q_near(neighbor_count).input = nodes(j).input;
                end
            end
            
            % Append to nodes
            nodes = [nodes q_new];

            % Initialize cost to currently known value
            q_min = q_nearest;
            C_min = q_new.cost;
            
            % Iterate through all nearest neighbors to find alternate lower
            %  cost paths
            
            if neighbor_count ~= 0
                for k = 1:1:neighbor_count
                    for m = 1:3
                        try_inputs = possible_inputs(m,:); 
                        [t,y_near] = ode45(@(t,y_near)derivative(kinematicModel,y_near,try_inputs),tspan,q_near(k).coord);
                        if dist(y_near(21,1:2),q_new.coord(1:2))<0.5
                            if all(checkOccupancy(map,y_near(:,1:2))==0)
                                if q_near(k).cost + dist(q_near(k).coord(1:2),q_new.coord(1:2))  < C_min %+ norm(q_near(k).coord(3)-q_new.coord(3))/2
                                    q_min = q_near(k);
                                    C_min = q_near(k).cost + dist(q_near(k).coord(1:2), q_new.coord(1:2)); %+ norm(q_near(k).coord(3)-q_new.coord(3))/2;
                                    q_min.input = possible_inputs(m,:);
                                end
                            end
                        end
                        
                    end
                end
            end
            
            % plot minimum cost path
            [t,y] = ode45(@(t,y)derivative(kinematicModel,y,q_min.input),tspan,q_min.coord);
            plot(y(:,1),y(:,2), 'Color',  'g', 'Linewidth',1)
            drawnow
            
            % Rewire the tree if find lower cost path than original parent
            % node
            if neighbor_count ~= 0
                for k = 1:1:neighbor_count
                    for m = 1:3
                        try_inputs = possible_inputs(m,:); 
                        [t,y_rewire] = ode45(@(t,y_rewire)derivative(kinematicModel,y_rewire,try_inputs),tspan,q_new.coord);
                        if dist(y_rewire(21,1:2),q_near(k).coord(1:2))<0.5
                            if all(checkOccupancy(map,y_rewire(:,1:2))==0)
                                if q_new.cost + dist(q_near(k).coord(1:2),q_new.coord(1:2))  < q_near(k).cost %+ norm(q_near(k).coord(3)-q_new.coord(3))/2
                                    q_near(k).parent = iteration + 1;
                                    plot(y_rewire(:,1),y_rewire(:,2), 'Color',  'g', 'Linewidth',1)
                                    drawnow
                                end
                            end
                        end
                    end
                end
            end
                                   
        end
    end
    
    % if get in goal grid, end the tree
    if q_new.coord(1) < finalState(1)+0.5 && q_new.coord(1) > finalState(1)-0.5 && q_new.coord(2) < finalState(2)+0.5 && q_new.coord(2) > finalState(2)-0.5
        break
    end
    

    iteration = iteration + 1;
end

%% Find path
% distance to goal
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
% follow the parent node
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_goal.input = [0 0];
q_end = q_goal;
nodes = [nodes q_goal];


while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    q_end = nodes(start);
end
