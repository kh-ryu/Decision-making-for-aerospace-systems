%% clear
clear
clear all
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

% start point: [1,2]
% end point: [9,19]
map_mat(9,19) = 2;

%% Wave-front propagation
% 주변 8개 node 중 빈 것이 있으면 현재 node value +1을 저장
while(1)
    for i = 1:15
        for j = 1:22
            if (map_mat(i,j)~=0)&&(map_mat(i,j)~=1)
                if (i-1~=0)&&(j-1~=0)&&(map_mat(i-1,j-1)==0)
                    map_mat(i-1,j-1) = map_mat(i,j)+1; end
                if (i-1~=0)&&(map_mat(i-1,j)==0)
                    map_mat(i-1,j) = map_mat(i,j)+1; end
                if (i-1~=0)&&(j+1~=23)&&(map_mat(i-1,j+1)==0)
                    map_mat(i-1,j+1) = map_mat(i,j)+1; end
                if (j-1~=0)&&(map_mat(i,j-1)==0)
                    map_mat(i,j-1) = map_mat(i,j)+1; end
                if (j+1~=23)&&(map_mat(i,j+1)==0)
                    map_mat(i,j+1) = map_mat(i,j)+1; end
                if (i+1~=16)&&(j-1~=0)&&(map_mat(i+1,j-1)==0)
                    map_mat(i+1,j-1) = map_mat(i,j)+1; end
                if (i+1~=16)&&(map_mat(i+1,j)==0)
                    map_mat(i+1,j) = map_mat(i,j)+1; end
                if (i+1~=16)&&(j+1~=23)&&(map_mat(i+1,j+1)==0)  
                    map_mat(i+1,j+1) = map_mat(i,j)+1; end
            end
        end
    end
    if map_mat(1,2) ~= 0
        break
    end
end

% Wave front map
wave_mat = map_mat;

map_mat(map_mat==1) = 100;

%% Path Calculation
path = zeros(100,2);
path(1,:) = [1 2];
% start point 부터 시작해 숫자가 작은 곳으로 연결
for i = 1:100
    m = path(i,1); n = path(i,2);
    if (m==9)&&(n==19)
        break
    end
    while(1)
        if (m-1 ~= 0) && (n-1 ~= 0)
            if (map_mat(m-1,n-1)<map_mat(m,n))
                path(i+1,:) = [m-1,n-1];
                break
            end
        end
        if (m-1 ~= 0)
            if map_mat(m-1,n)<map_mat(m,n)
                path(i+1,:) = [m-1,n];
                break
            end
        end
        if (m-1 ~= 0)
            if map_mat(m-1,n+1)<map_mat(m,n)
                path(i+1,:) = [m-1,n+1];
                break
            end
        end
        if (n-1 ~= 0)
            if map_mat(m,n-1)<map_mat(m,n)
                path(i+1,:) = [m,n-1];
                break
            end    
        end
        if (n+1 ~= 23)
            if map_mat(m,n+1)<map_mat(m,n)
                path(i+1,:) = [m,n+1];
                break
            end 
        end
        if (m+1 ~= 16) && (n-1 ~= 0)
            if map_mat(m+1,n-1)<map_mat(m,n) 
                path(i+1,:) = [m+1,n-1];
                break
            end    
        end
        if (m+1 ~= 16)
            if map_mat(m+1,n)<map_mat(m,n)
                path(i+1,:) = [m+1,n];
                break
            end
        end
        if (m+1 ~= 16)  && (n+1 ~= 23)
            if map_mat(m+1,n+1)<map_mat(m,n)
                path(i+1,:) = [m+1,n+1];
                break
            end
        end
    end
end
        
%% plot path
map = [0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 1 1
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
map = binaryOccupancyMap(map);
show(map)
grid on 
grid minor
hold on
plot(path(1:43,2)-0.5,15.5-path(1:43,1),'r');
hold off
figure(2)
show(map)
grid on
grid minor