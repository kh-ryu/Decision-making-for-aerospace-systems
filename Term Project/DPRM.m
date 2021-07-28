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
start = [1.5,14.5, -pi/2];
goal = [18.5,6.5,-pi/2]; 

% Reeds Shepp car
reedsConnObj = reedsSheppConnection('MinTurningRadius',1);

n = 300;

V(1) = struct('coord',[1.5 14.5,-pi/2], 'cost',0, 'parent', 0);
E = [];
r = 5;

% Dijkstra graph
G = zeros(n+1);

%% Sample node
tic
i = 2;
while(1)
    x_rand = [22*rand(1) 15*rand(1) pi*rand(1)];
    if checkOccupancy(map, x_rand(1:2))==0
        V(i).coord = x_rand;
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
        V(i).cost = 0;
        V(i).parent = 0;
        %{
        plot(V(i).coord(1),V(i).coord(2),'xc')
        drawnow
        %}
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
        if CollisionFree(V, u, v, map, reedsConnObj)
            %{
            startPose = V(u).coord;
            goalPose = V(v).coord;
            [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
            length = pathSegObj{1}.Length;
            poses = interpolate(pathSegObj{1},0:0.1:length);
            plot(poses(:,1),poses(:,2),'g');
            drawnow
            %}
            G(u,v) = cost(v,u,V,reedsConnObj);
            V(u).cost = V(v).cost + cost(v,u,V,reedsConnObj);
        end
    end
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
V(n+1).dist


%% A*
%{
for i = 1:n+1
    node(i).score = 999;
    node(i).heuristicScore = 999;
    node(i).visited = 0;
end
node(1).score = 0;
node(1).heuristicScore = 0;

score = zeros(1,n+1);
while (1)
    
    for i =1:n+1
        score(i) = node(i).score;
    end
    [~,idx] = min(score);
    currentNode = idx;
    node(currentNode).visited = 1;
    
    for i=1:n+1
        if G(currentNode,i)~=0
            if node(i).visited ==0
                newScore = G(currentNode,i);
                if newScore<node(i).score
                    node(i).score = newScore;
                    node(i).heuristicScore = newScore + norm(V(i).coord(1:2)-goal(1:2));
                    node(i).routeToNode = currentNode;
                end
            end
        end
    end
    
    if currentNode == n+1
        break
    end
    if node(currentNode).score == 999
        fprintf('No Path Found');
        break
    end
end
    
%}
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
