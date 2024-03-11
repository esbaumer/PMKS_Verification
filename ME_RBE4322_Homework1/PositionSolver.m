% Clearing the workspace and closing all figures
clc;        % Clear Command Window
close all;  % Close all open figures
clear all;  % Clear workspace variables

% Example Kinematics Problem

% Define Coordinates of Joints in 3D Space (x, y, z)
A = [1.4 0.485 0];
B = [1.67 0.99 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.54 0];
F = [-0.98 2.57 0];
G = [0.05 0.2 0];

% Calculate Lengths of Links between Joints
AB = norm(A-B); % Distance between A and B
BC = norm(B-C); % Distance between B and C
CD = norm(C-D); % Distance between C and D
DE = norm(D-E); % Distance between D and E
CE = norm(C-E); % Distance between C and E
EF = norm(E-F); % Distance between E and F
FG = norm(F-G); % Distance between F and G

% Initial Angle Calculation for Link AB
initialAngle_AB = atan2(B(2)-A(2), B(1)-A(1));

% Position Analysis
theta = 0; % Initialize theta

% Initialize Arrays to Store New Positions of Joints
A_vec = zeros(10,3);
B_vec = zeros(10,3);
C_vec = zeros(10,3);
D_vec = zeros(10,3);
E_vec = zeros(10,3);
F_vec = zeros(10,3);
G_vec = zeros(10,3);

% Set Initial Positions
A_vec(1,:) = A;
B_vec(1,:) = B;
C_vec(1,:) = C;
D_vec(1,:) = D;
E_vec(1,:) = E;
F_vec(1,:) = F;
G_vec(1,:) = G;

% Iterate through angles to update positions
for theta = 2:1:10 % Iterate from 2 to 10 degrees, incrementing by 1 degree
    % First, initialize ground joints
    A_vec(theta,:) = A;
    D_vec(theta,:) = D;
    G_vec(theta,:) = G;

    % Next, calculate new position of B
    B_vec(theta, :) = vpa(A + [AB*cos(initialAngle_AB + deg2rad(theta)), AB*sin(initialAngle_AB + deg2rad(theta)), 0]);

    % Lastly, calculate new positions of C, E, F using custom function 'circCircSolver'
    C_vec(theta, :) = circCircSolver(B_vec(theta, :), BC, D, CD, C_vec(theta-1, :));
    E_vec(theta, :) = circCircSolver(D_vec(theta, :), DE, C_vec(theta, :), CE, E_vec(theta-1, :));
    F_vec(theta, :) = circCircSolver(E_vec(theta, :), EF, G, FG, F_vec(theta-1, :));
end

% Create Directory for Saving Results
folderName = 'Kin/Pos';  % Directory name
if ~exist(folderName, 'dir')
   mkdir(folderName);  % Create the directory if it doesn't exist
end

% Save Joint Positions in the Created Directory
save('Kin/Pos/A', 'A_vec');
save('Kin/Pos/B', 'B_vec');
save('Kin/Pos/C', 'C_vec');
save('Kin/Pos/D', 'D_vec');
save('Kin/Pos/E', 'E_vec');
save('Kin/Pos/F', 'F_vec');
save('Kin/Pos/G', 'G_vec');

% Custom Function to Solve Circle-Circle Intersection Problem
function [pos] = circCircSolver(p1, r1, p2, r2, prev_point)
    % Calculate intersection points of two circles
    [posVecX, posVecY] = circcirc(p1(1), p1(2), r1, p2(1), p2(2), r2);
    
    % Check for valid intersection points
    circIntersect_x = any(isnan(vpa(posVecX)));
    circIntersect_y = any(isnan(vpa(posVecY)));

    % If circles intersect, determine the closest intersection point
    if circIntersect_x == 0 && circIntersect_y == 0
        pos_1 = [posVecX(1), posVecY(1), 0]; % First intersection point
        pos_2 = [posVecX(2), posVecY(2), 0]; % Second intersection point
        
        % Calculate distances from previous point to each intersection point
        dist1 = norm(pos_1 - prev_point);
        dist2 = norm(pos_2 - prev_point);

        % Choose the closest intersection point
        if dist1 < dist2
            pos = vpa(pos_1);
        else
            pos = vpa(pos_2);
        end
    end
end