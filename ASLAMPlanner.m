function[dataStore] = ASLAMPlanner(Robot,maxTime, offset_x, offset_y)
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
    sensor_pos = [0, 0.08];
elseif nargin < 4
    sensor_pos = [0, 0.08];
else
    sensor_pos = [offset_x, offset_y];
end

% Check if robot real or simulated
try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end

% Initialize dataStore
global dataStore;
dataStore = struct('truthPose', [], ...
                    'odometry', [], ...
                    'rsdepth', [],...
                    'bump', [], ...
                    'beacon', [], ...
                    'particles_x', [], ...
                    'particles_y', [], ...
                    'particles_theta', [], ...
                    'particles_w', []);

% Initialize noRobotCount
noRobotCount = 0;

% Add subfolders to path
addpath(genpath('./'))

%% Initialize Robot Parameters
wheel2Center = 0.16; % meters
maxV = 0.2; % m/s
num_particles = 300;
fwdVel = 0.3;
angVel = 0;
timrev = -1;
timturn = -1;

tic 

while toc < maxTime

    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % BACKUP IN CASE OF BUMP
    [fwdVelnew, angVelnew, timrev, timturn] = backupBump(dataStore, fwdVel, angVel, timrev, timturn);

    [cmdV,cmdW] = limitCmds(fwdVelnew,angVelnew,maxV,wheel2Center);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

SetFwdVelAngVelCreate(Robot, 0,0);