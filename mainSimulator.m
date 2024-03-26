clear

addpath(genpath('./'))
%% Lab Open Hours PF

% load w_0p2_iter4_truthPose.mat
% load practiceMap_4credits.mat
% 
% X = dataStore.particles_x;
% Y = dataStore.particles_y;
% theta = dataStore.particles_theta;
% w = dataStore.particles_w;
% 
% hold on
% p1 = scatter(X(1, :), Y(1, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
% p3 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'DisplayName', 'O/H Localization', 'Marker','diamond', 'MarkerFaceColor','green');
% for i = 1:1:length(X(:, 1))
%     [~, maxind] = max(w(i, :));
%     p4 = scatter(X(i, maxind), Y(i, maxind), 'd', 'filled', 'red', 'DisplayName', 'Best Particle');
% end
% p5 = scatter(X(end, :), Y(end, :), 's', 'filled', 'blue', 'DisplayName', 'Final Particles');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([p1 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
% title("Robot Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);

%% Competiton

load compMap.mat
load Group4Run1.mat
load PoseTeam4Run1.mat

wheel2Center = 0.16;

X = dataStore.particles_x;
Y = dataStore.particles_y;
theta = dataStore.particles_theta;
w = dataStore.particles_w;
pose = truthLog(:, 2:4);

hold on
p1 = scatter(X(1, :), Y(1, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
p3 = plot(pose(:, 1), pose(:, 2), 'DisplayName', 'O/H Localization', 'Marker','o', 'MarkerFaceColor','green');
quiver(pose(:, 1), pose(:, 2), wheel2Center*cos(pose(:, 3)), wheel2Center*sin(pose(:, 3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'green');
for i = 1:1:length(X(:, 1))
    [~, maxind] = max(w(i, :));
    p4 = scatter(X(i, maxind), Y(i, maxind), 'o', 'filled', 'red', 'DisplayName', 'Best Particle');
    quiver(X(i, maxind), Y(i, maxind), wheel2Center*cos(theta(i, maxind)), wheel2Center*sin(theta(i, maxind)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'r');
end
p5 = scatter(X(end, :), Y(end, :), 's', 'filled', 'blue', 'DisplayName', 'Final Particles');
quiver(X(end, maxind), Y(end, maxind), wheel2Center*cos(theta(end, maxind)), wheel2Center*sin(theta(end, maxind)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'blue');
for i = 1:size(map, 1)
    plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
end
legend([p1 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
title("Robot Trajectory - Run 1", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [400, 150, 600, 500]);