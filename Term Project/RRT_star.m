%% clear
clear var
clear
clc

%% Draw a map
map_mat = zeros(15,22);

       
map = binaryOccupancyMap(map_mat);
h = figure;
filename = 'RRT_sparse.gif';
show(map)
hold on
% 시작, 목표
start = [1.5,14.5];
goal = [18.5,6.5]; 

V(1) = struct('coord',[1.5 14.5], 'cost',0, 'parent', 0, 'idx',1);
r = 3;

%% RRT*
n = 300;
tic
for i = 1:n
    x_rand.coord = SampleFree(map);
    
    %{
    plot(x_rand.coord(1), x_rand.coord(2), 'x', 'Color',  [0 0.4470 0.7410])
    drawnow
    %}
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:length(V)
        % consider attitude when finding closest node
        tmp = norm(V(j).coord(1:2) - x_rand.coord(1:2));
        ndist = [ndist tmp];
    end
    [~, idx] = min(ndist);
    x_nearest = V(idx);
    
    x_new = Steer(x_nearest,x_rand);
    x_new.parent = idx;
    x_new.cost = x_nearest.cost + cost(x_nearest,x_new);
    x_new.idx = size(V,2)+1;
    
    if CollisionFree(x_nearest,x_new,map)
        X_near = near(V,x_new,r);
        idx = length(V);
        V(idx+1) = x_new;
        
        x_min = x_nearest;
        c_min = x_nearest.cost + cost(x_nearest,x_new);
        
        for j = 1:length(X_near)
            x_near = X_near(j);
            if CollisionFree(x_near,x_new,map)
                if (x_near.cost + cost(x_near,x_new))<c_min
                    x_min = x_near;
                    c_min = (x_near.cost + cost(x_near,x_new));
                end
            end
        end
        %{
        x_value = [x_min.coord(1), x_new.coord(1)];
        y_value = [x_min.coord(2), x_new.coord(2)];
        plot(x_value,y_value,'LineWidth',0.1,'Color','g')
        drawnow
        %}
        for j = 1:length(X_near)
            x_near = X_near(j);
            if CollisionFree(x_new,x_near,map)
                if (x_new.cost + cost(x_new,x_near))<x_near.cost
                    V(x_near.idx).parent = x_new.idx;
                    %{
                    x_value = [x_new.coord(1), x_near.coord(1)];
                    y_value = [x_new.coord(2), x_near.coord(2)];
                    plot(x_value,y_value,'LineWidth',0.1,'Color','g')
                    drawnow
                    %}
                end
            end
        end
    end
    % gif
    %{
    frame = getframe(h);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if i == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    end 
    %}
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
q_final = V(idx);
q_goal.parent = idx;
q_goal.idx = size(V,2)+1;
q_goal.coord = goal;
q_goal.cost = q_final.cost + cost(q_final,q_goal)
q_end = q_goal;
V = [V q_goal];


while q_end.parent ~= 0
    start = q_end.parent;
    startPose = V(start).coord;
    goalPose = q_end.coord;
    x_value = [startPose(1), goalPose(1)];
    y_value = [startPose(2), goalPose(2)];
    plot(x_value,y_value,'LineWidth',2,'Color','r')
    drawnow
    q_end = V(start);
    
    % gif
    %{
    frame = getframe(h);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    %}
end

%% Functions
function N = near(V, z, r)
    n = size(V,2);
    j = 1;
    coord = z.coord;
    for i = 1:n
        dist = norm(coord - V(i).coord);
        if (dist < r)&&(dist~=0)
            N(j) = V(i);
            j = j+1;
        end
    end
    return
end

function c = cost(x,y)
    x_coord = x.coord;
    y_coord = y.coord;
    c = norm(x_coord - y_coord);
    return
end

function free = CollisionFree(y_min, x, map)
    loc1 = y_min.coord;
    loc2 = x.coord;
    
    edge = [linspace(loc1(1), loc2(1), 100)', linspace(loc1(2), loc2(2), 100)'];
    
    %checkoccupancy: obstacle free:0 occupied:1
    if any(checkOccupancy(map,edge(:,:))) %if collision
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
    x_rand = [22*rand(1) 15*rand(1)];
    check = (checkOccupancy(map, x_rand)==0);
    while check == false
        x_rand = [22*rand(1) 15*rand(1)];
        check = (checkOccupancy(map, x_rand)==0);    
    end
end

function x_new = Steer(x_nearest,x_rand)
    del_x = x_rand.coord - x_nearest.coord;
    del_x = 1*del_x/norm(del_x);
    
    x_new.coord = x_nearest.coord + del_x;
end
