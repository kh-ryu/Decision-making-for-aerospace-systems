%% clear
clear var
clear
clc

%% Draw a map
map_sparse = [0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 1 1
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
       
map = binaryOccupancyMap(map_sparse);
show(map)
hold on
% 시작, 목표
start = [1.5,14.5,-pi/2];
goal = [18.5,6.5,-pi/2]; 
reedsConnObj = reedsSheppConnection('MinTurningRadius',1);
V(1) = struct('coord',[1.5 14.5,-pi/2], 'cost',0, 'parent', 0, 'idx',1);
r = 3;
%% RRT*
tic
n = 5000;
for i = 1:n
    x_rand.coord = SampleFree(map);
    %{
    plot(x_rand.coord(1), x_rand.coord(2), 'x', 'Color',  [0 0.4470 0.7410])
    drawnow
    %}
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:size(V,2)
        % consider attitude when finding closest node
        tmp = norm(V(j).coord(1:2) - x_rand.coord(1:2))  + 0.2*norm(V(j).coord(3) - x_rand.coord(3));
        ndist = [ndist tmp];
    end
    [~, idx] = min(ndist);
    x_nearest = V(idx);
    
    x_new = Steer(x_nearest,x_rand,reedsConnObj);
    x_new.parent = idx;
    x_new.cost = x_nearest.cost + cost(x_nearest,x_new,reedsConnObj);
    x_new.idx = size(V,2)+1;
    
    if CollisionFree(x_nearest,x_new,map,reedsConnObj)
        X_near = near(V,x_new,r);
        idx = size(V,2);
        V(x_new.idx) = x_new;
        
        x_min = x_nearest;
        c_min = x_nearest.cost + cost(x_nearest,x_new,reedsConnObj);
        
        for j = 1:size(X_near,2)
            x_near = X_near(j);
            if CollisionFree(x_near,x_new,map,reedsConnObj)
                if (x_near.cost + cost(x_near,x_new,reedsConnObj))<c_min
                    x_min = x_near;
                    c_min = (x_near.cost + cost(x_near,x_new,reedsConnObj));
                end
            end
        end
        %{
        startPose = x_min.coord;
        goalPose = x_new.coord;
        [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
        length = pathSegObj{1}.Length;
        poses = interpolate(pathSegObj{1},0:0.1:length);
        plot(poses(:,1),poses(:,2),'g');
        drawnow
        %}
        for j = 1:size(X_near,2)
            x_near = X_near(j);
            if CollisionFree(x_new,x_near,map,reedsConnObj)
                if (x_new.cost + cost(x_new,x_near,reedsConnObj))<x_near.cost
                    V(x_near.idx).parent = size(V,2);
                    %{
                    startPose = x_new.coord;
                    goalPose = x_near.coord;
                    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
                    length = pathSegObj{1}.Length;
                    poses = interpolate(pathSegObj{1},0:0.1:length);
                    plot(poses(:,1),poses(:,2),'g');
                    drawnow
                    %}
                end
            end
        end
    end
end
toc
%% Find path
% distance to goal
D = [];
for j = 1:1:size(V,2)
    tmpdist = norm(V(j).coord(1:2) - goal(1:2));
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
% follow the parent node
[val, idx] = min(D);

%% plot
q_final = V(idx);
q_goal.parent = idx;
q_goal.idx = size(V,2)+1;
q_goal.coord = goal;
q_goal.cost = q_final.cost + cost(q_final,q_goal,reedsConnObj)
q_end = q_goal;
V = [V q_goal];


while q_end.parent ~= 0
    start = q_end.parent;
    startPose = V(start).coord;
    goalPose = q_end.coord;
    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
    length = pathSegObj{1}.Length;
    poses = interpolate(pathSegObj{1},0:0.1:length);
    plot(poses(:,1),poses(:,2),'LineWidth',2,'Color','r');
    drawnow
    q_end = V(start);
end

%% Functions
function N = near(V, z, r)
    n = size(V,2);
    j = 1;
    coord = z.coord;
    for i = 1:n
        dist = norm(coord(1:2) - V(i).coord(1:2)) + 0.2*norm(coord(3) - V(i).coord(3));
        if (dist < r)&&(dist~=0)
            N(j) = V(i);
            N(j).idx = i;
            j = j+1;
        end
    end
    return
end


function free = CollisionFree(y_min, x, map, reedsConnObj)
    startPose = y_min.coord;
    goalPose = x.coord;
    
    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
    length = pathSegObj{1}.Length;
    poses = interpolate(pathSegObj{1},0:0.1:length);
        
    %checkoccupancy: obstacle free:0 occupied:1
    if any(checkOccupancy(map,poses(:,1:2))) %if collision
        free = false;
        return
    else
        free = true;
        return
    end
end

function check = goalCheck(goal,x,V)
    x_coord = V(x).coord;
    dist = norm(goal-x_coord);
    if dist<0.5
        check = true;
        return
    else
        check = false;
        return
    end
end

function x_rand = SampleFree(map)
    x_rand = [22*rand(1) 15*rand(1) pi*rand(1)];
    check = (checkOccupancy(map, x_rand(1:2))==0);
    while check == false
        x_rand = [22*rand(1) 15*rand(1) pi*rand(1)];
        check = (checkOccupancy(map, x_rand(1:2))==0);    
    end
end

function x_new = Steer(x_nearest,x_rand,reedsConnObj)
    startPose = x_nearest.coord;
    goalPose = x_rand.coord;
    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
    length = pathSegObj{1}.Length;
    poses = interpolate(pathSegObj{1},0:0.1:length);
    if size(poses,1)>10
        x_new.coord = poses(10,:);
    else
        x_new.coord = poses(end,:);
    end
end

function c = cost(x,y,reedsConnObj)
    x_coord = x.coord;
    y_coord = y.coord;
    [~,pathCosts] = connect(reedsConnObj,x_coord,y_coord);
    c = pathCosts;
    return
end