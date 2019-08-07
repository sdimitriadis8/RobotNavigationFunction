% Navigation Functions
% Sokratis & Piyabhum
% University of Ioannina, 2017
% Version 6.4, Final Edition
% Cleaned code, green with no warning.
% Use drawRobot to draw.
% Rather shot for something that takes such a long time to do.
% Faster calculation O(2N) -> O(N), even less variable.
% Added one more example, classic 4 obstacles.
% **Backup (6.2) is in cloud.

close all
clc
warning off

r = 0.325; %6.4cm wheel diameter
d = 1.10; %11cm wheel base, 13.5cm diameter
mass = 0.6; % 600g with 3S 1000mAH Lipo battery
Vmax = 0.2; %20cm/s

kV = 1;
kTH = 1;

e = 10^(-2);
gridX = 30;
gridY = 30;
q0 = [gridX/2 gridY/2];

[X,Y] = meshgrid(1:gridX);

%%% Our example, 2 Obstacles 
goal = [13.0 21.0];
obsX = [9.0 21.0]; % qi position x
obsY = [9.0 21.0]; % qi position y
obs_size = [4.0 4.0]; % pi size of qi
r_pos = [12.0 4.0]; % q current
r_th = 0;

% %%% Our example, 4 Obstacles 
% goal = [13.0 21.0];
% obsX = [12.5 20.0 25.0 8.0]; % qi position x
% obsY = [12.5 20.0 14.0 20.0]; % qi position y
% obs_size = [2.5 1.8 1.0 1.0]; % pi size of qi 
% r_pos = [5.0 8.0]; % q current
% r_th = 0;

% %%% Classic Example
% goal = [15.0 15.0];
% obsX = [10.0 20.0 10.0 20.0]; % qi position x
% obsY = [10.0 10.0 20.0 20.0]; % qi position y
% obs_size = [3.2 3.2 3.2 3.2]; % pi size of qi 
% r_pos = [10.0 5.0]; % q current
% r_th = 20;

%%% Good K is about 0.5-1 + number of obstacles
%%% Reference: Lecture from Arizona State Univ., 
%%% Mechanical and Aerospace Engineering
k = 0.7+numel(obsX);

NF = zeros(size(X,1),size(X,2));
for x = 1:size(X,1)
    for y = 1:size(X,2)
        %Calculating Z
        g = norm([x y]-goal);
        
        %Calculating B
        b = ((gridX/2)^2)-(norm([x y]-q0)^2);
        for n = 1:numel(obsX)
            betaQ = (norm([x y]-[obsX(n) obsY(n)])^2)-(obs_size(n)^2);
            b = b*betaQ;
        end   
        NF(y,x) = (g.^2)/real(((g.^(2*k))+b)^(1/k));
    end
end
[DX,DY] = gradient(NF);
DX = (-1)*DX;
DY = (-1)*DY;

% Simulating Robot
flag = 0;
hFig = figure(1);
set(hFig, 'Position', [100 10 720 720])

while flag ~= 1
tX = round(r_pos(1));
tY = round(r_pos(2));

targetTh = atan2d(DY(tY,tX),DX(tY,tX));  
thDot = (targetTh-r_th)/d;
vDot = sqrt((DX(tY,tX).^2)+(DY(tY,tX).^2));
if vDot < e
    targetTh = atan2d(goal(2)-r_pos(2),goal(1)-r_pos(1));  
    thDot = (targetTh-r_th)/d;
    vDot = Vmax/4;
end

xDot = vDot*cosd(r_th);
yDot = vDot*sind(r_th);

if thDot*d > 4*Vmax
    kTH = (4*Vmax)/(thDot*d);
    thDot = thDot * kTH;
else
    if thDot*d < (-1)*4*Vmax
    kTH = (-4*Vmax)/(thDot*d);
    thDot = thDot * kTH;
    end
end

if xDot > Vmax && xDot > yDot
    kV = Vmax/xDot;
    xDot = kV*xDot;
    yDot = kV*yDot;
else
    if yDot > Vmax
        kV = Vmax/yDot;
        xDot = kV*xDot;
        yDot = kV*yDot;
    end
end
r_pos(1) = r_pos(1)+xDot;
r_pos(2) = r_pos(2)+yDot;
r_th = r_th+thDot;

x_top(1) = r_pos(1) ;
y_top(1) = r_pos(2) ;
x_top(2) = r_pos(1) +d * cosd(r_th);
y_top(2) = r_pos(2) +d * sind(r_th);

if norm(r_pos-goal) < 0.5
    flag = 1;
end
    %Call draw function here
    quiver(X,Y,DX,DY);
    hold on;
    contour(NF);
    hold on;
    drawRobot(r_pos(1),r_pos(2),r_th,obsX,obsY,obs_size,goal,q0);
    grid on;

    title('Navigation Functions Simulation');
    axis([0 gridX 0 gridY]);
    pbaspect([1 1 1])
    pause(0.05);
end