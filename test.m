% Car Controller Simulation with Normalized FLC Inputs
% This script sets up a simulation environment with obstacles,
% a start and target position, loads a fuzzy logic controller,
% and simulates car movement using the FLC outputs.
%
% The FLC expects:
%   - dv (vertical distance to an obstacle) normalized to [0, 1]
%   - dh (horizontal distance to an obstacle) normalized to [0, 1]
%   - angle_error in degrees in the range [-180, 180]
%
% Adjust max_dv and max_dh as needed for your environment.

%% Clear workspace and set up the figure
clear; close all; clc;

figure('Name', 'Car Controller Test Space');
hold on;
grid on;
axis([0 10 0 4]);

%% Plot obstacles (rectangles)
% First obstacle
x1 = [5 6 6 5];
y1 = [0 0 1 1];
fill(x1, y1, 'k');

% Second obstacle
x1 = [6 7 7 6];
y1 = [0 0 2 2];
fill(x1, y1, 'k');

% Third obstacle
x1 = [7 10 10 7];
y1 = [0 0 3 3];
fill(x1, y1, 'k');

%% Plot start and target positions
% Start position
start_pos = [4.1, 0.3];
plot(start_pos(1), start_pos(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); 
text(start_pos(1)+0.1, start_pos(2), 'Start', 'Color', 'blue');

% Target position
target_pos = [10, 3.2];
plot(target_pos(1), target_pos(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(target_pos(1)-0.5, target_pos(2), 'Target', 'Color', 'green');

xlabel('X (m)');
ylabel('Y (m)');
title('Car Controller Test Space');

%% Load the Fuzzy Logic Controller
% Ensure that 'car_fuzzy.fis' is in the current folder or MATLAB path.
fis = readfis('car_fuzzy.fis');

%% Simulation Parameters
dt = 1.1;             % Time step (seconds)
max_steps = 400;    % Maximum number of simulation steps
car_pos = start_pos;  % Initial position
theta = 0;          % Initial heading (degrees)
velocity = 0.2;       % Increased constant velocity (m/s) for more noticeable movement
path_x = car_pos(1);
path_y = car_pos(2);

%% Simulation Loop (Simplified Logging)
for step = 1:max_steps
    % Compute the distance to the target (if needed for FLC inputs)
    dist_to_target = norm(target_pos - car_pos);
    
    % Compute the desired heading (in degrees) using atan2d
    desired_theta = atan2d(target_pos(2) - car_pos(2), target_pos(1) - car_pos(1));
    
    % Compute angle error (in degrees), wrapping it to [-180, 180]
    angle_error = mod(desired_theta - theta + 180, 360) - 180;
    
    % Compute obstacle distances: dv (vertical) and dh (horizontal)
    [dv, dh] = computeObstacleDistances(car_pos(1), car_pos(2));
    
    % Normalize dv and dh to the range [0, 1]
    max_dv = 3;  % Maximum expected vertical distance
    max_dh = 5;  % Maximum expected horizontal distance
    dv_norm = min(dv, max_dv) / max_dv;
    dh_norm = min(dh, max_dh) / max_dh;
    
    % Get FLC output (Heading Change) using the normalized inputs
    % The FLC is expected to have inputs: [dv_norm, dh_norm, angle_error]
    heading_change = evalfis(fis, [dv_norm, dh_norm, angle_error]);
    
    % (Optional) Damping factor removed -- you can simply comment it out:
    % dampingFactor = 0.5;
    % heading_change = heading_change * dampingFactor;
    
    % Log out the following:
    %   - Current theta
    %   - dv_norm and dh_norm (normalized obstacle distances)
    %   - angle_error (the theta input to the FIS)
    %   - FLC output (heading_change)
    %   - Effective change in theta (heading_change * dt)
    fprintf('Step %d: theta = %.2f°, dv_norm = %.2f, dh_norm = %.2f, angle_error = %.2f°, FLC output = %.2f°, Change in theta = %.2f°\n', ...
        step, theta, dv_norm, dh_norm, angle_error, heading_change, heading_change * dt);
    
    % Update the car's heading (in degrees)
    theta = theta + heading_change * dt;
    
    % Update the car's position using cosd and sind (angles in degrees)
    car_pos(1) = car_pos(1) + velocity * cosd(theta) * dt;
    car_pos(2) = car_pos(2) + velocity * sind(theta) * dt;
    
    % Store the path for plotting
    path_x(end+1) = car_pos(1);
    path_y(end+1) = car_pos(2);
    
    % Check for collision with obstacles
    if isInCollision(car_pos(1), car_pos(2))
        disp('Collision detected! Stopping.');
        break;
    end
    
    % Check if target has been reached
    if dist_to_target < 0.1
        disp('Target reached!');
        break;
    end
end



%% Plot Car Path
plotCarPath(path_x, path_y);

%% Local Functions
function [dv, dh] = computeObstacleDistances(x, y)
    % computeObstacleDistances Computes the vertical (dv) and horizontal (dh)
    % distances from point (x, y) to the nearest obstacle reference point.
    %
    % Here, obstacles are defined by reference points.
    obstacles = [
        5.5, 0.5;
        6.5, 1.5;
        7.4,  2
    ]; % Each row: [obs_x, obs_y]
    
    dv = inf;
    dh = inf;
    
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        
        % Compute vertical and horizontal distances separately
        dv = min(dv, abs(y - obs_y));
        dh = min(dh, abs(x - obs_x));
    end
end

function collision = isInCollision(x, y)
    % isInCollision Checks whether the point (x, y) is within any obstacle.
    collision = false;
    
    % First obstacle: rectangle from (5,0) to (6,1)
    if (x >= 5 && x <= 6 && y >= 0 && y <= 1)
        collision = true;
    end
    
    % Second obstacle: rectangle from (6,1) to (7,2)
    if (x >= 6 && x <= 7 && y >= 1 && y <= 2)
        collision = true;
    end
    
    % Third obstacle: rectangle from (7,2) to (10,3)
    if (x >= 7 && x <= 10 && y >= 2 && y <= 3)
        collision = true;
    end
end

function plotCarPath(path_x, path_y)
    % plotCarPath Plots the path traveled by the car.
    plot(path_x, path_y, 'r-', 'LineWidth', 1.5);
end

