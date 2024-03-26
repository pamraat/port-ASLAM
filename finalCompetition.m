function[dataStore] = finalCompetition(Robot,maxTime, offset_x, offset_y)
%% Function Description
%   motionControl: final competition script to run
%
%   INPUTS
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
%       offset_x    camera x-offset (robot frame) in meters
%       offset_y    camera y-offset (robot frame) in meters
%
%   OUTPUTS
%       dataStore   struct containing logged data
%% Initialize Program Simulator/Robot Data
% Check robot connection
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

% Check if robot real or simulated
try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

% Initialize dataStore
global dataStore;
dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', [], 'particles_x', [], 'particles_y', [], 'particles_theta', [], 'particles_w', []);

% Initialize noRobotCount
noRobotCount = 0;

% Add subfolders to path
addpath(genpath('./'))

%% Initialize Map
load compMap.mat
beaconLoc = [beaconLoc(:, 2:3) beaconLoc(:,1)];
dataStore.walls = zeros(size(optWalls, 1), 1);
dataStore.waypoints = [];
%% Initialize Robot Parameters
wheel2Center = 0.16; % meters
maxV = 0.2; % m/s
maxW = 0.3;
num_particles = 300;
if nargin < 4; sensor_pos = [0, 0.08]; else; sensor_pos = [offset_x, offset_y]; end
state = 0;  % state variable
%% State 0 Variables (PF)
state_0_turn_time = 8; % seconds
state_0_start_time = 0; % seconds
num_waypoints = size(waypoints,1);
waypoints = [waypoints; ECwaypoints];
angle_increment = 360 / (num_particles / num_waypoints) * (pi / 180);
X = zeros(4, num_particles);
for i = 1:num_particles
    X(1, i) = waypoints(mod(i,num_waypoints)+1, 1);
    X(2, i) = waypoints(mod(i,num_waypoints)+1, 2);
    X(3, i) = floor(i/ num_waypoints) * angle_increment;
    X(4, i) = 1/num_particles;
end
state_0_R = diag([0.005 0.005 0.001]);
state_0_Q = diag([0.005*ones(1, 9) 0.0001*ones(1, 2)]);
h = @(x, id, map, beaconLoc) hDepthBeacon(x, id, map, beaconLoc);
g = @(x, u) integrateOdom(x, u(1), u(2));
%% State 1 Variables (PRM)
t = 0.1;
state_1_map = map;
n = 50;
%% State 2 Variables (visitWaypoints)
epsilon = 0.2;
close_enough = 0.1;
go_to_pt = 1;
m = 50;
cells_optWalls = associate_cells_optwalls(map, optWalls, m, m);
Q_lod = 0.01*eye(2);
if nargin < 4; sensorPos_lod = [0 0.08]; else; sensorPos_lod = [offset_x, offset_y]; end
angles_lod = linspace(27, -27, 9)'*pi/180;
lt = zeros(50);
max_a = max([map(:,1);map(:,2);map(:,3);map(:,4)]);
min_a = min([map(:,1);map(:,2);map(:,3);map(:,4)]);
ax = linspace(min_a, max_a, n + 1);
ay = linspace(min_a, max_a, m + 1);
axC = linspace(ax(1), ax(end - 1), n) + (ax(end) - ax(1))/(2*n);
ayC = linspace(ay(1), ay(end - 1), m) + (ay(end) - ay(1))/(2*m);
[aX, aY] = meshgrid(ax, ay);
optwall_cell_matrix = zeros(size(optWalls,1),size(aX,1),size(aX,2));
[aXC, aYC] = meshgrid(axC, ayC);
%% State 3 Variables (Backup)
timrev = -1;
timturn = -1;

%% Plot Initialization
figure(2)
hold on
for i = 1:size(state_1_map, 1)
    plot([state_1_map(i, 1) state_1_map(i, 3)], [state_1_map(i, 2) state_1_map(i, 4)], LineWidth=1, Color='black');
end
title("Robot Trajectory", "Interpreter","tex");
xlabel("X (m)");
ylabel("Y (m)");
fontsize(gca,14,"points");
set(gcf, 'Position',  [400, 150, 600, 500]);

tic
try
    while toc < maxTime
    
        %% READ & STORE SENSOR DATA
        [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
        fwdVelnew = 0;
        angVelnew = 0;
        if(dataStore.bump(end,2) == 1 || dataStore.bump(end,3) == 1 || dataStore.bump(end,7) == 1)
            if dataStore.bump(end,3) == 1, lef = 1; else, lef = 0; end
            timrev = dataStore.bump(end,1);
            state = 3;
        end
    
        %% State 0: Turn in place and gather initial position
        if state == 0
            if state_0_start_time == 0; state_0_start_time = toc; end % initialize the correct time at which state_0 started
            if toc < state_0_start_time + state_0_turn_time % spin as long as toc < state_0_start_time + time you want to turn
                angVelnew = maxW;
                fwdVelnew = 0;
                if isempty(dataStore.beacon)
                    X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)'; -ones(3, 1)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
                else
                    X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)';dataStore.beacon(end, 4:5)';dataStore.beacon(end,3)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
                end
            else
                index_max_w = find(max(dataStore.particles_w(end,:)));
                pt = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
                p1 = scatter(pt(1), pt(2), 'filled', 'cyan', 'DisplayName', 'Initial position');
                quiver(pt(1), pt(2), wheel2Center*cos(pt(3)), wheel2Center*sin(pt(3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'cyan');
                state = 1;
            end
            dataStore.particles_x = [dataStore.particles_x; X(1,:)];
            dataStore.particles_y = [dataStore.particles_y; X(2,:)];
            dataStore.particles_theta = [dataStore.particles_theta; X(3,:)];
            dataStore.particles_w = [dataStore.particles_w; X(4,:)];
            index_max_w = find(max(dataStore.particles_w(end,:)));
            pt = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
            p2 = scatter(pt(1), pt(2), 'filled', 'red', 'DisplayName', 'Best Particle');
            quiver(pt(1), pt(2), wheel2Center*cos(pt(3)), wheel2Center*sin(pt(3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'r');
        
        %% State 1: Generate Waypoints for Navigation
        elseif state == 1
            % pause movement for one while loop iteration and generate waypoint
            % path robot_path with PRM
            disp("State 1")
            % update PF
            angVelnew = 0;
            fwdVelnew = 0;
            if isempty(dataStore.beacon)
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)'; -ones(3, 1)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            else
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)';dataStore.beacon(end, 4:5)';dataStore.beacon(end,3)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            end
            dataStore.particles_x = [dataStore.particles_x; X(1,:)];
            dataStore.particles_y = [dataStore.particles_y; X(2,:)];
            dataStore.particles_theta = [dataStore.particles_theta; X(3,:)];
            dataStore.particles_w = [dataStore.particles_w; X(4,:)];
            
            % generate PRM
            index_max_w = find(max(dataStore.particles_w(end,:)));
            init = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w)];
            goal = [waypoints(1, :)];
            f = @(n, map, lim) lowDiscrepancy(n, map, lim);
            [G, nodes] = buildPRM(state_1_map, n, f, wheel2Center, t, init, goal);
            robot_path = nodes(shortestpath(G, size(nodes, 1) - 1, size(nodes, 1)), :);
            state = 2;
            scatter(robot_path(1:end-1, 1), robot_path(1:end-1, 2), 'filled', 'MarkerFaceColor','magenta');
            p3 = plot(robot_path(:, 1), robot_path(:, 2), 'DisplayName' ,'PRM Path' ,LineWidth=1, Color='green');
            scatter(robot_path(:,1), robot_path(:,2), 'red')
        %% State 2: Navigate between Waypoints
        elseif state == 2
            % essentially, implement a visitWaypoints function here with
            % robot_path as the waypoints to visit (robot_path from state 1).
            % the goal waypoint will be the last point on the robot_path
            
            % update PF
            if isempty(dataStore.beacon)
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)'; -ones(3, 1)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            else
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)';dataStore.beacon(end, 4:5)';dataStore.beacon(end,3)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            end
            dataStore.particles_x = [dataStore.particles_x; X(1,:)];
            dataStore.particles_y = [dataStore.particles_y; X(2,:)];
            dataStore.particles_theta = [dataStore.particles_theta; X(3,:)];
            dataStore.particles_w = [dataStore.particles_w; X(4,:)];
    
            % visitWaypoints
            index_max_w = find(max(dataStore.particles_w(end,:)));
            current_pos = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
    
            if (norm(robot_path(go_to_pt,:) - current_pos(1:2)) > close_enough)
                [fwdVelnew, angVelnew] = feedbackLin(robot_path(go_to_pt,1)-current_pos(1), robot_path(go_to_pt,2)-current_pos(2), current_pos(3), epsilon);
            else
                go_to_pt = go_to_pt + 1;
                fwdVelnew = 0;
                angVelnew = 0;
            end
            fwdVel = fwdVelnew;
            angVel = angVelnew;
            index_max_w = find(max(dataStore.particles_w(end,:)));
            pt = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
            p2 = scatter(pt(1), pt(2), 'filled', 'red', 'DisplayName', 'Best Particle');
            quiver(pt(1), pt(2), wheel2Center*cos(pt(3)), wheel2Center*sin(pt(3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'r');
            % if robot reaches the last point, generate a new waypoint path
            % with PRM
            if go_to_pt > size(robot_path,1)
                BeepCreate(Robot);
                dataStore.waypoints = [dataStore.waypoints; waypoints(1,:)];
                waypoints(1,:) = [];
                state = 1;
                go_to_pt = 1;
            end
            lt = logOddsDepth(pt, lt, dataStore.rsdepth(end, 3:end)', Q_lod, axC, ayC, sensorPos_lod, angles_lod);
            lt(lt>300) = 300;
            pdf = exp(lt)./(1 + exp(lt));
            % if the size of the waypoint list is one, the last waypoint has
            % just been visited. move on to state 99
            if size(waypoints,1) == 1
                state = 99;
            end
        %% State 3: Backup in case of collision
        elseif state == 3
            if isempty(dataStore.beacon)
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)'; -ones(3, 1)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            else
                X = PF(X, [dataStore.odometry(end,2); dataStore.odometry(end,3)], [dataStore.rsdepth(end, 3:11)';dataStore.beacon(end, 4:5)';dataStore.beacon(end,3)], state_0_R, state_0_Q, g, h, state_1_map, beaconLoc);
            end
            dataStore.particles_x = [dataStore.particles_x; X(1,:)];
            dataStore.particles_y = [dataStore.particles_y; X(2,:)];
            dataStore.particles_theta = [dataStore.particles_theta; X(3,:)];
            dataStore.particles_w = [dataStore.particles_w; X(4,:)];
    
            index_max_w = find(max(dataStore.particles_w(end,:)));
            pt = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
            p2 = scatter(pt(1), pt(2), 'filled', 'red', 'DisplayName', 'Best Particle');
            quiver(pt(1), pt(2), wheel2Center*cos(pt(3)), wheel2Center*sin(pt(3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'r');
            
            % Backup command
            if(timrev ~= -1)
                if(dataStore.bump(end,1) < timrev + 0.25/abs(fwdVel))
                    fwdVelnew = -fwdVel;
                    angVelnew = 0;
                else
                    timrev = -1;
                    fwdVelnew = 0;
                    angVelnew = -1;
                    timturn = dataStore.bump(end,1);
                end
            elseif(timturn ~= -1)
                if(dataStore.bump(end,1) < timturn + pi/3)
                    fwdVelnew = 0;
                    angVelnew = (1 - 2*lef)*(-1);
                else
                    state = 2;
                    fwdVelnew = fwdVel;
                    angVelnew = angVel;
                    timturn = -1;
                end
            else
                state = 2;
                fwdVelnew = fwdVel;
                angVelnew = angVel;
            end
        %% State 99: Temporary Break
        elseif state == 99
            try    
                for i = 1:size(assc,1)
                    num_active = 0;
                    for j = 1:size(assc,2)
                        for k = 1:size(assc,3)
                            if assc(i, j, k) == 1 && pdf(j, k) > 0.7
                                num_active = num_active + 1;
                            end
                        end
                    end
                    if num_active > 4
                        dataStore.walls(i) = 1;
                    end
                end
            catch
            end
            index_max_w = find(max(dataStore.particles_w(end,:)));
            pt = [dataStore.particles_x(end,index_max_w), dataStore.particles_y(end,index_max_w), dataStore.particles_theta(end,index_max_w)];
            p4 = scatter(pt(1), pt(2), 'filled', 'blue', 'DisplayName', 'Final Particle');
            quiver(pt(1), pt(2), wheel2Center*cos(pt(3)), wheel2Center*sin(pt(3)), 'LineWidth', 2, 'MaxHeadSize', 5, 'Color', 'blue');
            break
        end
        %% assign robot velocities, but if overhead localization loses the robot for too long, stop it
        % CONTROL FUNCTION (send robot commands)
        [cmdV,cmdW] = limitCmds(fwdVelnew,angVelnew,maxV,wheel2Center);
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0,0);
        else
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW); 
        end
    end
catch
    for i = 1:size(dataStore.walls, 1)
        if dataStore.walls(i, 1) == 0
            plot([optWalls(i, 1) optWalls(i, 3)], [optWalls(i, 2) optWalls(i, 4)], LineWidth=1, Color='red');
        else
            plot([optWalls(i, 1) optWalls(i, 3)], [optWalls(i, 2) optWalls(i, 4)], LineWidth=1, Color='black');
        end
    end
    legend([p1 p2 p3])
end
for i = 1:size(dataStore.walls, 1)
    if dataStore.walls(i, 1) == 0
        plot([optWalls(i, 1) optWalls(i, 3)], [optWalls(i, 2) optWalls(i, 4)], LineWidth=1, Color='red');
    else
        plot([optWalls(i, 1) optWalls(i, 3)], [optWalls(i, 2) optWalls(i, 4)], LineWidth=1, Color='black');
    end
end
legend([p1 p2 p3 p4])


save('dataStore.mat', 'dataStore')
% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0);