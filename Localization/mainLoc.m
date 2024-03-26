clear;
clf;

%% Paths
addpath('../Dependencies');
addpath('../Data/Lab/Localization')
addpath('../Data/Lab/Mapping')
addpath('../Data/Lab/motionPlanning')
addpath('../Data/openHours')
addpath('../Localization')
addpath('../Mapping')
addpath('../Maps')
addpath('../Motion Planning')
addpath('../Motion Planning/Dependencies')

%% Data Choice
load RJLab2Run1.mat

%% Map Choice
load lab2WallMap2023.mat
load lab2Beacon_2023.mat
map = lab2WallMap2023;
mapBeacon = beacon;
[minX, maxX] = bounds([map(:, 1); map(:, 3)]); [minY, maxY] = bounds([map(:, 2); map(:, 4)]);
lim = [minX minY maxX maxY];

%% Initialization
robotRad = 0.16;
nParticles = 500;
Q = diag([0.005*ones(1, 9) 0.0001*ones(1, 2)]);
R = diag([0.005 0.005 0.001]);

g = @(x, u) integrateOdom(x, u(1), u(2));
G = @(x, u) GjacDiffDrive(x, u);
h = @(x, id, map, mapBeacon) hDepthBeacon(x, id, map, mapBeacon);

particles = [(lim(3) - lim(1))*rand(1, nParticles) + lim(1);
            (lim(4) - lim(2))*rand(1, nParticles) + lim(2);
            2*pi*rand(1, nParticles);
            ones(1, nParticles)/nParticles];

%% Offline PF
j = 2;
for i = 2:size(dataStore.rsdepth, 1)
    u = dataStore.odometry(i, 2:3)';
    z = [dataStore.rsdepth(i, 3:end)'; -ones(3, 1)];
    if (dataStore.rsdepth(i, 1) - dataStore.beacon(j, 1) < 0 && dataStore.rsdepth(i, 1) - dataStore.beacon(j, 1) > -0.7)
        z = [dataStore.rsdepth(i, 3:end)'; dataStore.beacon(j, 4:5)'; dataStore.beacon(j, 3)];
        j = j + 1;
    end
    Xnew = PF(particles(end-3:end, :), u, z, R, Q, g, h, map, mapBeacon);
    particles = [particles; Xnew];
end
%% Plot
hold on
p1 = scatter(particles(1, :), particles(2, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
p3 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'DisplayName', 'O/H Localization');
p2 = scatter(dataStore.truthPose(1, 2), dataStore.truthPose(1, 3), 'square', 'filled', 'cyan', 'DisplayName', 'Start');
p5 = scatter(dataStore.truthPose(end, 2), dataStore.truthPose(end, 3), 'square', 'filled', 'green', 'DisplayName', 'End');
[~, maxind] = max(particles(4:4:size(particles, 1), :), [], 2); r = 0.16;
for i = 1:length(maxind)
    p4 = scatter(particles(i*4 - 3, maxind(i)), particles(i*4 - 2, maxind(i)), 'filled', 'red', 'DisplayName', 'Best Particle');
    quiver(particles(i*4 - 3, maxind(i)), particles(i*4 - 2, maxind(i)), r*cos(particles(i*4 - 1, maxind(i))), r*sin(particles(i*4 - 1, maxind(i))), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'r');
end
p6 = scatter(particles(end-3, maxind(end)), particles(end-2, maxind(end)), 'filled', 'blue', 'DisplayName', 'End Particle');
quiver(particles(end - 3, maxind(end)), particles(end - 2, maxind(end)), r*cos(particles(end - 1, maxind(end))), r*sin(particles(end - 1, maxind(end))), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'blue');
for i = 1:size(map, 1)
    plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
end
legend([p1 p2 p3 p4 p5 p6],"Interpreter","tex", 'Location', 'best');
title("Robot Trajectory", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [400, 150, 600, 500]);

%% motionControl PF
% 
% global dataStore
% 
% figure(1)
% hold on
% p1 = scatter(dataStore.particles(1, :), dataStore.particles(2, :), 'o', 'filled', 'black', 'DisplayName', 'Initial Particles');
% p2 = plot(dataStore.deadReck(:, 1), dataStore.deadReck(:, 2), 'Linewidth', 1, 'DisplayName', 'Dead Reckoning');
% p3 = plot(dataStore.truthPose(:, 2), dataStore.truthPose(:, 3), 'DisplayName', 'O/H Localization');
% for i = 1:1:length(dataStore.particles(:, 1))/4
%     [~, maxind] = max(dataStore.particles(4*(i - 1) + 4, :));
%     p4 = scatter(dataStore.particles(4*(i - 1) + 1, maxind), dataStore.particles(4*(i - 1) + 2, maxind), 'd', 'filled', 'red', 'DisplayName', 'Best Particle');
% end
% p5 = scatter(dataStore.particles(end - 3, :), dataStore.particles(end - 2, :), 's', 'filled', 'blue', 'DisplayName', 'Final Particles');
% for i = 1:size(map, 1)
%     plot([map(i, 1) map(i, 3)], [map(i, 2) map(i, 4)], LineWidth=1, Color='black');
% end
% legend([p1 p2 p3 p4 p5],"Interpreter","tex", 'Location', 'best');
% title("Robot Trajectory", "Interpreter","tex");
% xlabel("X (m)");
% ylabel("Y (m)");
% fontsize(gca,14,"points");
% set(gcf, 'Position',  [400, 150, 600, 500]);