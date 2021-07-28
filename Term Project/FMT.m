%% clear
clear var
clear
clc

%% Draw a map
map_mat =  zeros(15,22);
       
map = binaryOccupancyMap(map_mat);
h = figure;
show(map)
hold on
% 시작, 목표
start = [1.5,14.5];
goal = [18.5,6.5]; 

n = 300;
%unvisited:0 open:1 closed:2
V(1) = struct('coord',[1.5 14.5],'category',1, 'cost',0, 'parent', 0);
E = [];
r = 5;
filename = 'FMT_sparse.gif';
%% Sample node
i = 2;
tic
while(1)
    x_rand = [22*rand(1) 15*rand(1)];
    if checkOccupancy(map, x_rand)==0
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
            total_cost = [total_cost; [idx, V(idx).cost + cost(idx,x,V)]];
        end
        [M, idx] = min(total_cost(:,2));
        y_min = total_cost(idx,1);
                
        % if CollisionFree
        if CollisionFree(V, y_min, x, map)
            x_value = [V(y_min).coord(1), V(x).coord(1)];
            y_value = [V(y_min).coord(2), V(x).coord(2)];
            %{
            plot(x_value,y_value,'LineWidth',0.1,'Color','g')
            drawnow
            %}
            V(x).parent = y_min;
            E = [E; [y_min x]];
            V(x).category = 1;
            V(x).cost = V(y_min).cost + cost(y_min,x,V);
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
    
    % gif
    %{
    frame = getframe(h);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if iteration == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    end     
    %}
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
    x_value = [V(son).coord(1), V(parent).coord(1)];
    y_value = [V(son).coord(2), V(parent).coord(2)];
    plot(x_value,y_value,'LineWidth',2,'Color','r')
    son = parent;
    parent = V(parent).parent;
    
    % gif
    %{
    frame = getframe(h);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    %}
end
V(z).cost

%% Functions
function N = near(V, z, r)
    n = size(V,2);
    N = [];
    coord = V(z).coord;
    for i = 1:n
        dist = norm(coord - V(i).coord);
        if (dist < r)&&(dist~=0)
            N = [N, i];
        end
    end
    return
end

function c = cost(x,y,V)
    x_coord = V(x).coord;
    y_coord = V(y).coord;
    c = norm(x_coord - y_coord);
    return
end

function free = CollisionFree(V, y_min, x, map)
    loc1 = V(y_min).coord;
    loc2 = V(x).coord;
    
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
    if dist==0
        check = true;
        return
    else
        check = false;
        return
    end
end
