clear

%% Paths
addpath('./Dependencies');
addpath('../Dependencies');
addpath('../Data/Lab/Localization')
addpath('../Data/Lab/Mapping')
addpath('../Data/Lab/motionPlanning')
addpath('../Data/openHours')
addpath('../Localization')
addpath('../Mapping')
addpath('../Maps')

%% Initialization
% init = [22, 65]; goal = [90, 10];
init = [1, -2]; goal = [-2, -1.5];

%%%%%%%%%% RRT %%%%%%%%%%
% step = 10;
% robotRad = 0.2;

%%%%%%%%%% RRT Simulation %%%%%%%%%%
% global dataStore
% step = 0.5;
% robotRad = 0.2;

%%%%%%%%%% Probabilistic Roadmaps %%%%%%%%%%
n = 50;
robotRad = 0.16;


%% Map Choice
% hw6b = readmatrix("hw6b.txt")'; map = hw6b;
% lim = [0 0 100 100];

load practiceMap_4credits.mat
[minX, maxX] = bounds([map(:, 1); map(:, 3)]); [minY, maxY] = bounds([map(:, 2); map(:, 4)]);
t = 0.1;
tempmap = map;
[tempthickmap, lim] = modLine(map, t, robotRad, 1);

%% Cellular Decomposition
%%%%%%%%%% Cell decomposition %%%%%%%%%%
% [cellDecomp, map] = cellDecomposition(map, lim);

% -------------Plot-------------
% hold on
% for i = 1:size(cellDecomp, 1)
%     p1 = plot([cellDecomp(i, 1) cellDecomp(i, 3)], [cellDecomp(i, 2) cellDecomp(i, 4)], LineWidth=0.5, Color='red');
% end
% for i = 1:size(map, 1)
%     p2 = plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([p1 p2], 'Decomposed Cell boundary', 'Map boundary')
% hold off
% title("Map Cellular Decomposition", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500])
% -------------end Plot-------------

%%%%%%%%%% FindPath %%%%%%%%%%
% [nodes, G] = createRoadmap(map, lim);
% [path, G, nodes] = findPath(init, goal, G, nodes, map);

% -------------Plot-------------
% hold on
% for i = 1:size(map, 1)
%     p2 = plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% for i = 1:size(cellDecomp, 1)
%     p1 = plot([cellDecomp(i, 1) cellDecomp(i, 3)], [cellDecomp(i, 2) cellDecomp(i, 4)], LineWidth=0.5, Color='red');
% end
% h = plot(G, 'XData', nodes(:, 1), 'YData', nodes(:, 2), 'LineWidth', 1, 'EdgeColor','blue');
% p3 = plot(nodes(path, 1), nodes(path, 2), 'LineWidth', 1, 'Color', 'green');
% hold off
% legend([p1 p2 h p3], 'Decomposed Cell boundary', 'Map boundary', 'Graph', 'Shortest Path')
% hold off
% title("Map Cellular Decomposition", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500])
% -------------end Plot-------------

%% Rapidly exploding Random Tree
% wayPoints = buildRTT(init, goal, step, robotRad, map, lim);
% [~, map] = cellDecomposition(map, lim); 

% -------------Plot-------------
% hold on
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% plot(wayPoints(:, 1), wayPoints(:, 2), 'LineWidth', 1, 'Color', 'red');
% scatter(wayPoints(1, 1), wayPoints(1, 2), 'Marker', 'square', 'MarkerFaceColor', 'green')
% scatter(wayPoints(2:end-1, 1), wayPoints(2:end-1, 2), 'filled', 'MarkerFaceColor','blue')
% scatter(wayPoints(end, 1), wayPoints(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan')
% hold off
% -------------end Plot-------------

%%%%%%%%%% RRT Simulation %%%%%%%%%%
% -------------Plot-------------
% map = map(1:end-4, :);
% polymap = [map (map(:, 3:4) + [0.1 0.1]) (map(:, 1:2) + [0.1 0.1])];
% wayPoints = buildRTT(init, goal, step, robotRad, polymap', lim); 
% hold on
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% p1 = plot(wayPoints(:, 1), wayPoints(:, 2), 'LineWidth', 1, 'Color', 'red');
% p2 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3)', 'Linewidth', 1, 'Color', 'blue', 'DisplayName', 'O/H localization');
% p3 = scatter(wayPoints(1, 1), wayPoints(1, 2), 'Marker', 'square', 'MarkerFaceColor', 'green');
% p4 = scatter(wayPoints(2:end-1, 1), wayPoints(2:end-1, 2), 'filled', 'MarkerFaceColor','blue');
% p5 = scatter(wayPoints(end, 1), wayPoints(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan');
% hold off
% legend([p1 p2 p3 p4 p2], 'WayPoint Trajectory', 'Starting Point', 'Intermediate Nodes', 'Goal', 'o/H Localization');
% title("Map Cellular Decomposition", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500])
% -------------end Plot-------------

%% Probabilistic RoadMaps
f = @(n, map, lim) lowDiscrepancy(n, map, lim);
[G, nodes] = buildPRM(map, n, f, robotRad, t, init, goal);
wayPoints = nodes(shortestpath(G, size(nodes, 1) - 1, size(nodes, 1)), :);
tempthickmap = point2Line(tempthickmap, lim);

% -------------Plot-------------
hold on
for i = 1:size(tempthickmap, 1)
    plot([tempthickmap(i, 1) tempthickmap(i, 3)], [tempthickmap(i, 2) tempthickmap(i, 4)], LineWidth=1, Color='black');
end
for i = 1:size(tempmap, 1)
    plot([tempmap(i, 1) tempmap(i, 3)], [tempmap(i, 2) tempmap(i, 4)], 'LineWidth', 0.5, 'Color', [0.4940 0.1840 0.5560], 'LineStyle','--');
end
p1 = plot(G, 'XData', nodes(:, 1), 'YData', nodes(:, 2), 'LineWidth', 1, 'EdgeColor','blue');
if ~isempty(wayPoints)
    p2 = plot(wayPoints(:, 1), wayPoints(:, 2), LineWidth=1, Color='green');
    p3 = scatter(wayPoints(1, 1), wayPoints(1, 2), 'Marker', 'square', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
    p4 = scatter(wayPoints(2:end-1, 1), wayPoints(2:end-1, 2), 'filled', 'MarkerFaceColor','magenta');
    p5 = scatter(wayPoints(end, 1), wayPoints(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan');
    legend([p1 p2 p4 p3 p5], 'Graph', 'Path', 'Intermediate Nodes', 'Intial Point', 'Goal');
else
    p3 = scatter(nodes(end - 1, 1), nodes(end - 1, 2), 'Marker', 'square', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
    p5 = scatter(nodes(end, 1), nodes(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan');
    legend([p1 p3 p5], 'Graph', 'Intial Point', 'Goal');
end
hold off
title("Probabilistic Roadmap", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [400, 150, 600, 500]);
% -------------end Plot-------------

%% Visibility PRM
% [G, nodes] = visibilityPRM(hw6b, n, robotRad, lim, init, goal);
% wayPoints = nodes(shortestpath(G, size(nodes, 1) - 1, size(nodes, 1)), :);
% [~, map] = cellDecomposition(hw6b, lim);

% -------------Plot-------------
% hold on
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% p1 = plot(G, 'XData', nodes(:, 1), 'YData', nodes(:, 2), 'LineWidth', 1, 'EdgeColor','blue');
% if ~isempty(wayPoints)
%     p2 = plot(wayPoints(:, 1), wayPoints(:, 2), LineWidth=1, Color='green');
%     p3 = scatter(wayPoints(1, 1), wayPoints(1, 2), 'Marker', 'square', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
%     p4 = scatter(wayPoints(2:end-1, 1), wayPoints(2:end-1, 2), 'filled', 'MarkerFaceColor','magenta');
%     p5 = scatter(wayPoints(end, 1), wayPoints(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan');
%     legend([p1 p2 p4 p3 p5], 'Graph', 'Path', 'Intermediate Nodes', 'Intial Point', 'Goal');
% else
%     p3 = scatter(nodes(end - 1, 1), nodes(end - 1, 2), 'Marker', 'square', 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
%     p5 = scatter(nodes(end, 1), nodes(end, 2), 'Marker', 'diamond', 'MarkerFaceColor', 'cyan');
%     legend([p1 p3 p5], 'Graph', 'Intial Point', 'Goal');
% end
% hold off
% title("Probabilistic Roadmap", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);
% -------------end Plot-------------