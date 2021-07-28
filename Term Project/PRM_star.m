%% clear
clear var
clear
clc

%% Draw a map
map_mat = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 1 0 0 1 1 0 0 0 0 0 0 0 0 0
           0 0 0 0 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0
           0 0 0 1 1 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0
           0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0
           0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 1 1 0 0 0 0 0
           0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
       
map = binaryOccupancyMap(map_mat);
h = figure;
filename = 'PRM.gif';
show(map)
hold on
% 시작, 목표
start = [1.5,14.5];
goal = [18.5,6.5]; 

n = 300;
%unvisited:0 open:1 closed:2
V(1) = struct('coord',[1.5 14.5], 'cost',0, 'parent', 0);
G = zeros(n+1);
r = 5;
%% Sample node
i = 2;
tic
while(1)
    x_rand = [22*rand(1) 15*rand(1)];
    if checkOccupancy(map, x_rand)==0
        V(i).coord = x_rand;
        V(i).cost = 0;
        V(i).parent = 0;
        
        plot(V(i).coord(1),V(i).coord(2),'xc')
        drawnow
        
        i = i+1;
    end
    if i == n+1
        V(i).coord = goal;
        V(i).cost = 0;
        V(i).parent = 0;
        
        plot(V(i).coord(1),V(i).coord(2),'xc')
        drawnow
                
        break
    end
end


%% PRM*

% while x is not in goal
for v = 1:n
    

    %get V_near
    N_v = near(V, v, r);

    for i = 1:size(N_v,2)
        u = N_v(i);
        if CollisionFree(V, v, u, map)
            x_value = [V(v).coord(1), V(u).coord(1)];
            y_value = [V(v).coord(2), V(u).coord(2)];
            
            plot(x_value,y_value,'LineWidth',0.1,'Color','g')
            drawnow
            %}
            G(u,v) = cost(v,u,V);
            V(u).cost = V(v).cost + cost(v,u,V);
        end
    end

    % gif
    %{
    frame = getframe(h);
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if v == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    end
   %}
end
toc
%% Dijkstra
for i = 1:n+1
    V(i).dist = 999;
    Q(i).optimized = 0;
end
V(1).dist = 0;

while(1)
    dist = [];
    for i = 1:n+1
        if Q(i).optimized ==0
            dist = [dist; [i,V(i).dist]];
        end
    end
    if size(dist,1) == 0
        break
    end
    [~,idx] = min(dist(:,2));
    u = dist(idx,1);
    Q(u).optimized = 1;
    
    for v = 1:n+1
        if G(v,u)~=0 && Q(v).optimized == 0
            alt = V(u).dist + G(v,u);
            if alt<V(v).dist
                V(v).dist = alt;
                V(v).parent = u;
            end
        end
    end
end

%% Plot path

son = n+1;
parent = V(n+1).parent;
while (parent ~= 0)
    x_value = [V(son).coord(1), V(parent).coord(1)];
    y_value = [V(son).coord(2), V(parent).coord(2)];
    plot(x_value,y_value,'LineWidth',2,'Color','r')
    drawnow;
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
V(n+1).dist

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
    if dist<0.5
        check = true;
        return
    else
        check = false;
        return
    end
end
