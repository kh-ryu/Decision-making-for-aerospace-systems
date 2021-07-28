dubConnObj = dubinsConnection;

% theta=pi일때 (x,y)위치에 따라 shortest path 조합 달라짐
startPose = [3 0.5 pi];
% Goal = (0,0,0)
goalPose = [0 0 0];

%pathSegObj.Motiontype에서 motion 조합 확인 가능
[pathSegObj, pathCosts] = connect(dubConnObj, startPose, goalPose);

show(pathSegObj{1})