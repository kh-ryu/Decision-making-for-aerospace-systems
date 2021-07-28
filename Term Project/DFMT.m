%% clear
clear var
clear
clc

%% Draw a map
map_mat = zeros(15,22);

map = binaryOccupancyMap(map_mat);
show(map)
hold on
% 시작, 목표
start = [1.5, 14.5, -pi/2];
goal = [18.5, 6.5, -pi/2]; 

n = 300;
%unvisited:0 open:1 closed:2
V(1) = struct('coord',[1.5 14.5, -pi/2],'category',1, 'cost',0, 'parent', 0);
E = [];
r = 5;
%% Sample node
i = 2;
tic
while(1)
    x_rand = [22*rand(1) 15*rand(1) pi*rand(1)];
    if checkOccupancy(map, x_rand(1:2))==0
        V(i).coord = x_rand;
        V(i).category = 0;
        V(i).cost = 0;
        V(i).parent = 0;
        %{
        plot(V(i).coord(1),V(i).coord(2),'xc')
        drawnow
        %}
        i = i+1;
    end
    if i == n+1
        V(i).coord = goal;
        V(i).category = 0;
        V(i).cost = 0;
        V(i).parent = 0;
        %{
        plot(V(i).coord(1),V(i).coord(2),'xc')
        drawnow
        %}
        break
    end
end

z = 1;
% Reeds Shepp car
reedsConnObj = reedsSheppConnection('MinTurningRadius',1);

%% FMT

iteration = 1;
% while x is not in goal
while(1)
    
    V_open_new = [];
    %get X_near
    N_z = near(V, z, r);
    X_near = [];
    for i = 1:size(N_z,2)
        idx = N_z(i);
        if (V(idx).category == 0)     % if in V_unvisited
            X_near = [X_near idx];
        end
    end
    
    % for x in X_near
    for i = 1:size(X_near,2)
        x = X_near(i);
        
        %find N_x
        N_x = near(V, x, r);
        
        % Set Y_near
        Y_near = [];
        for j = 1:size(N_x,2)
            idx = N_x(j);
            if (V(idx).category == 1)     %if in V_open
                Y_near = [Y_near idx];
            end
        end
        
        % get y_min
        total_cost = [];
        for j = 1:size(Y_near,2)
            idx = Y_near(j);
            total_cost = [total_cost; [idx, V(idx).cost + cost(idx,x,V,reedsConnObj)]];
        end
        [M, idx] = min(total_cost(:,2));
        y_min = total_cost(idx,1);
                
        % if CollisionFree
        if CollisionFree(V, y_min, x, map, reedsConnObj)
            %{
            startPose = V(y_min).coord;
            goalPose = V(x).coord;
            [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
            length = pathSegObj{1}.Length;
            poses = interpolate(pathSegObj{1},0:0.1:length);
            plot(poses(:,1),poses(:,2),'g');
            drawnow
            %}
            V(x).parent = y_min;
            E = [E; [y_min x]];
            V(x).category = 1;
            V(x).cost = V(y_min).cost + cost(y_min,x,V,reedsConnObj);
        end
    end
    
    V(z).category = 2;
       
    % reset z
    V_open = [];
    for i = 1:n+1
        if (V(i).category == 1)
            V_open = [V_open; [i, V(i).cost]];
        end
    end
    % Failure check
    if size(V_open,2) == 0
        fprintf('Failure')
        break
    end
    
    [M, idx] = min(V_open(:,2));
    z = V_open(idx,1);
    

    
    if goalCheck(goal, z, V)
        break
    end
    iteration = iteration + 1;
end
toc
%% Find path

son = z;
parent = V(z).parent;
while (parent ~= 0)
    startPose = V(parent).coord;
    goalPose = V(son).coord;
    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
    length = pathSegObj{1}.Length;
    poses = interpolate(pathSegObj{1},0:0.1:length);
    plot(poses(:,1),poses(:,2),'r','LineWidth',2);
    drawnow;
    son = parent;
    parent = V(parent).parent;
end
V(z).cost

%% Functions
function N = near(V, z, r)
    n = size(V,2);
    N = [];
    coord = V(z).coord;
    for i = 1:n
        dist = norm(coord(1:2) - V(i).coord(1:2));
        if (dist < r)&&(dist~=0)
            N = [N, i];
        end
    end
    return
end

function c = cost(x,y,V,reedsConnObj)
    x_coord = V(x).coord;
    y_coord = V(y).coord;
    [~,pathCosts] = connect(reedsConnObj,x_coord,y_coord);
    c = pathCosts;
    return
end

function free = CollisionFree(V, y_min, x, map, reedsConnObj)
    startPose = V(y_min).coord;
    goalPose = V(x).coord;
    
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
