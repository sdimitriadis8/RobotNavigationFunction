function drawRobot(x, y, angle,obsX,obsY,obs_size,goal,q0)  
th = 0:pi/50:2*pi;
gridX = 30;
gridY = 30;
d = 1.10;

% Robot
roboX = d/2 * cos(th) + x;
roboY = d/2 * sin(th) + y;
x_ornt(1) = x;
y_ornt(1) = y;
x_ornt(2) = x + d * cosd(angle);
y_ornt(2) = y + d * sind(angle);

% Goal
gx = 0.5 * cos(th) + goal(1);
gy = 0.5 * sin(th) + goal(2);

% Workspace
workspaceX = gridX/2 * cos(th) + q0(1);
workspaceY = gridY/2 * sin(th) + q0(2);

% Drawing Workspace (circle)
plot(workspaceX, workspaceY, 'black', 'LineWidth', 1);
hold on

% Drawing Goal
scatter(goal(1),goal(2),'black')
plot(gx, gy, 'black', 'LineWidth', 3);
hold on

% Drawing obstacles
scatter(obsX,obsY,'red')
for i = 1:numel(obsX)
    x_start = obs_size(i) * cos(th) + obsX(i);
    y_start = obs_size(i) * sin(th) + obsY(i);
    plot(x_start, y_start, 'red', 'LineWidth', 3);
    hold on
end

% Drawing Robot
plot(x_ornt, y_ornt, 'red', 'LineWidth', 3);
plot(roboX, roboY, 'blue', 'LineWidth', 3);
hold off
end