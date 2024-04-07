function Mechanism = PosSolver(Mechanism, input_speed)
max_iterations = 900;

% Initialize positions for maximum iterations
[Mechanism] = initializePositions(Mechanism, max_iterations);

% Calculate distances between points
Mechanism = calculateDistances(Mechanism);

% Initialize variables for iteration
[iteration, theta, thetaIncrement] = initializeVariables(Mechanism);

% Initialize angular speed
Mechanism = initializeInputSpeed(Mechanism, input_speed, max_iterations);

% Main loop for calculating joint positions
[Mechanism] = calculateJointPositions(Mechanism, theta, thetaIncrement, iteration, max_iterations);

baseDir = 'Kin/Pos';
% Save joint positions
saveJointPositions(Mechanism, baseDir)
end

% Function to initialize variables for the simulation
function [iteration, theta, thetaIncrement] = initializeVariables(Mechanism)
iteration = 2;
thetaIncrement = 1; % Angle increment (in degrees)
theta = atan2(Mechanism.Joint.B(1,2) - Mechanism.Joint.A(1,2), Mechanism.Joint.B(1,1) - Mechanism.Joint.A(1,1)); % Initial angle of link AB with adjustment if necessary
end

% Function to initialize positions for all joints for max iterations
function Mechanism = initializePositions(Mechanism, max_iterations)
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    initialJointPosition = Mechanism.Joint.(jointNames{i});
    Mechanism.Joint.(jointNames{i}) = zeros(max_iterations, 3); % Initialize with zeros
    Mechanism.Joint.(jointNames{i})(1, :) = initialJointPosition; % Set initial position
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    initialLinkPosition = Mechanism.LinkCoM.(linkNames{i});
    Mechanism.LinkCoM.(linkNames{i}) = zeros(max_iterations, 3); % Initialize with zeros
    Mechanism.LinkCoM.(linkNames{i})(1, :) = initialLinkPosition; % Set initial position
end
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(Mechanism.Joint.B - Mechanism.Joint.A);
% Link BC
Mechanism.LinkLength.BC = norm(Mechanism.Joint.C - Mechanism.Joint.B);
% Link CDE
Mechanism.LinkLength.CD = norm(Mechanism.Joint.C - Mechanism.Joint.D);
Mechanism.LinkLength.CE = norm(Mechanism.Joint.C - Mechanism.Joint.E);
Mechanism.LinkLength.DE = norm(Mechanism.Joint.D - Mechanism.Joint.E);
% Link EF
Mechanism.LinkLength.EF = norm(Mechanism.Joint.F - Mechanism.Joint.E);
% Link FG
Mechanism.LinkLength.FG = norm(Mechanism.Joint.F - Mechanism.Joint.G);
Mechanism.LinkLength.FH = norm(Mechanism.Joint.F - Mechanism.TracerPoint.H);
Mechanism.LinkLength.GH = norm(Mechanism.Joint.G - Mechanism.TracerPoint.H);
end

% Main function to calculate joint positions through iterations
function [Mechanism] = calculateJointPositions(Mechanism, theta, thetaIncrement, iteration, max_iterations)
forwardDir = true; % Flag to indicate the direction of rotation. Mechanism should be going forward on its last iteration

while ~(determineEqual(Mechanism.Joint.B(1, :), Mechanism.Joint.B(iteration - 1, :)) && ...
        ~isequal(iteration, 2) && forwardDir) && iteration < max_iterations
    [Mechanism, theta, thetaIncrement, forwardDir, iteration] = updateJointPositions(Mechanism, theta, thetaIncrement, iteration, forwardDir);
end

% Trim positions and speeds to the last filled iteration
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.Joint.(jointNames{i}) = Mechanism.Joint.(jointNames{i})(1:iteration-1, :);
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinkCoM.(linkNames{i}) = Mechanism.LinkCoM.(linkNames{i})(1:iteration-1,:);
end
Mechanism.inputSpeed= Mechanism.inputSpeed(1:iteration-1,:);
end

% Function to update positions based on current state
function [Mechanism, theta, thetaIncrement, forwardDir, iteration] = updateJointPositions(Mechanism, theta, thetaIncrement, iteration, forwardDir)
% Calculate current joint angles
theta = theta + deg2rad(thetaIncrement);

% Calculate new positions for joints
[Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir);
if ~valid
    % Revert theta if new positions are invalid and flip direction
    thetaIncrement = thetaIncrement * -1;
    forwardDir = ~forwardDir;
end
end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);
D = Mechanism.Joint.D(1, :);
G = Mechanism.Joint.G(1,:);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-circle intersections for C, E, F, H
C = circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BC, D(1), D(2), Mechanism.LinkLength.CD, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2));
if isempty(C), valid = false; return; end

E = circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CE, D(1), D(2), Mechanism.LinkLength.DE, Mechanism.Joint.E(iteration - 1, 1), Mechanism.Joint.E(iteration - 1, 2));
if isempty(D), valid = false; return; end

F = circleCircleIntersection(E(1), E(2), Mechanism.LinkLength.EF, G(1), G(2), Mechanism.LinkLength.FG, Mechanism.Joint.F(iteration - 1, 1), Mechanism.Joint.F(iteration - 1, 2));
if isempty(F), valid = false; return; end

H = circleCircleIntersection(F(1), F(2), Mechanism.LinkLength.FH, G(1), G(2), Mechanism.LinkLength.GH, Mechanism.TracerPoint.H(iteration - 1, 1), Mechanism.TracerPoint.H(iteration - 1, 2));
if isempty(E), valid = false; return; end

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;
Mechanism.Joint.D(iteration, :) = D;
Mechanism.Joint.E(iteration, :) = E;
Mechanism.Joint.F(iteration, :) = F;
Mechanism.Joint.G(iteration, :) = G;
Mechanism.TracerPoint.H(iteration, :) = H;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.AB(iteration, :) = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BC(iteration, :) = Utils.determineCoM([B; C]);
Mechanism.LinkCoM.CDE(iteration, :) = Utils.determineCoM([D; E]);
Mechanism.LinkCoM.EF(iteration, :) = Utils.determineCoM([E; F]);
Mechanism.LinkCoM.FG(iteration, :) = Utils.determineCoM([G; H]);

if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end

% Utility function for circle-circle intersection calculation
function result = circleCircleIntersection(x1, y1, r1, x2, y2, r2, pointX, pointY)
% Find intersection points
[xIntersect, yIntersect] = circcirc(x1, y1, r1, x2, y2, r2);

% Check if the intersection points are determined
if isempty(xIntersect) || isempty(yIntersect)
    result = [];
    return;
end

if isnan(xIntersect(1)) || isnan(yIntersect(1))
    result = [];
    return;
end

% Calculate distances from intersection points to the given point
dist1 = sqrt((xIntersect(1) - pointX)^2 + (yIntersect(1) - pointY)^2);
dist2 = sqrt((xIntersect(2) - pointX)^2 + (yIntersect(2) - pointY)^2);

% Determine the closest intersection point
if dist1 < dist2
    result = [xIntersect(1) yIntersect(1) 0];
else
    result = [xIntersect(2) yIntersect(2) 0];
end
end

% Utility function to check if two arrays are approximately equal
function result = determineEqual(arr1, arr2)
tolerance = 0.001;
result = all(abs(arr1 - arr2) < tolerance);
end

function saveJointPositions(Mechanism, baseDir)

% Create Directory for Saving Results
folderName = 'Kin';
if ~exist(folderName, 'dir')
    mkdir(folderName);  % Create the directory if it doesn't exist
end

% Save Joint Positions in the Created Directory
save('Mechanism.mat', 'Mechanism');


jointNames = fieldnames(Mechanism.Joint);
jointFolder = fullfile(baseDir, 'Joint');
if ~exist(jointFolder, 'dir')
    mkdir(jointFolder);
end

for i = 1:length(jointNames)
    jointName = jointNames{i};
    % Create a temporary struct with the field name as the joint name
    tempStruct = struct(jointName, Mechanism.Joint.(jointName));
    % Save this struct using the -struct option
    save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
end

% Save link CoM positions
linkNames = fieldnames(Mechanism.LinkCoM);
linkCoMFolder = fullfile(baseDir, 'LinkCoM');
if ~exist(linkCoMFolder, 'dir')
    mkdir(linkCoMFolder);
end

for i = 1:length(linkNames)
    linkName = linkNames{i};
    % Create a temporary struct with the field name as the link name
    tempStruct = struct(linkName, Mechanism.LinkCoM.(linkName));
    % Save this struct using the -struct option
    save(fullfile(linkCoMFolder, linkName), '-struct', 'tempStruct', linkName);
end
end

function Mechanism = initializeInputSpeed(Mechanism, input_speed, max_iterations)
Mechanism.inputSpeed = zeros(max_iterations, 1);
Mechanism.inputSpeed(1) = input_speed; % 10 rpm to 1.0472 rad/s
end

% % Clearing the workspace and closing all figures
% clc;        % Clear Command Window
% close all;  % Close all open figures
% clear all;  % Clear workspace variables
% 
% % Example Kinematics Problem
% 
% % Define Coordinates of Joints in 3D Space (x, y, z)
% A = [1.4 0.485 0];
% B = [1.67 0.99 0];
% C = [0.255 1.035 0];
% D = [0.285 0.055 0];
% E = [0.195 2.54 0];
% F = [-0.98 2.57 0];
% G = [0.05 0.2 0];
% 
% % Calculate Lengths of Links between Joints
% AB = norm(A-B); % Distance between A and B
% BC = norm(B-C); % Distance between B and C
% CD = norm(C-D); % Distance between C and D
% DE = norm(D-E); % Distance between D and E
% CE = norm(C-E); % Distance between C and E
% EF = norm(E-F); % Distance between E and F
% FG = norm(F-G); % Distance between F and G
% 
% % Initial Angle Calculation for Link AB
% initialAngle_AB = atan2(B(2)-A(2), B(1)-A(1));
% 
% % Position Analysis
% theta = 0; % Initialize theta
% 
% % Initialize Arrays to Store New Positions of Joints
% A_vec = zeros(10,3);
% B_vec = zeros(10,3);
% C_vec = zeros(10,3);
% D_vec = zeros(10,3);
% E_vec = zeros(10,3);
% F_vec = zeros(10,3);
% G_vec = zeros(10,3);
% 
% % Set Initial Positions
% A_vec(1,:) = A;
% B_vec(1,:) = B;
% C_vec(1,:) = C;
% D_vec(1,:) = D;
% E_vec(1,:) = E;
% F_vec(1,:) = F;
% G_vec(1,:) = G;
% 
% % Iterate through angles to update positions
% for theta = 2:1:10 % Iterate from 2 to 10 degrees, incrementing by 1 degree
%     % First, initialize ground joints
%     A_vec(theta,:) = A;
%     D_vec(theta,:) = D;
%     G_vec(theta,:) = G;
% 
%     % Next, calculate new position of B
%     B_vec(theta, :) = vpa(A + [AB*cos(initialAngle_AB + deg2rad(theta)), AB*sin(initialAngle_AB + deg2rad(theta)), 0]);
% 
%     % Lastly, calculate new positions of C, E, F using custom function 'circCircSolver'
%     C_vec(theta, :) = circCircSolver(B_vec(theta, :), BC, D, CD, C_vec(theta-1, :));
%     E_vec(theta, :) = circCircSolver(D_vec(theta, :), DE, C_vec(theta, :), CE, E_vec(theta-1, :));
%     F_vec(theta, :) = circCircSolver(E_vec(theta, :), EF, G, FG, F_vec(theta-1, :));
% end
% 
% % Create Directory for Saving Results
% folderName = 'Kin/Pos';  % Directory name
% if ~exist(folderName, 'dir')
%    mkdir(folderName);  % Create the directory if it doesn't exist
% end
% 
% % Save Joint Positions in the Created Directory
% save('Kin/Pos/A', 'A_vec');
% save('Kin/Pos/B', 'B_vec');
% save('Kin/Pos/C', 'C_vec');
% save('Kin/Pos/D', 'D_vec');
% save('Kin/Pos/E', 'E_vec');
% save('Kin/Pos/F', 'F_vec');
% save('Kin/Pos/G', 'G_vec');
% 
% % Custom Function to Solve Circle-Circle Intersection Problem
% function [pos] = circCircSolver(p1, r1, p2, r2, prev_point)
%     % Calculate intersection points of two circles
%     [posVecX, posVecY] = circcirc(p1(1), p1(2), r1, p2(1), p2(2), r2);
% 
%     % Check for valid intersection points
%     circIntersect_x = any(isnan(vpa(posVecX)));
%     circIntersect_y = any(isnan(vpa(posVecY)));
% 
%     % If circles intersect, determine the closest intersection point
%     if circIntersect_x == 0 && circIntersect_y == 0
%         pos_1 = [posVecX(1), posVecY(1), 0]; % First intersection point
%         pos_2 = [posVecX(2), posVecY(2), 0]; % Second intersection point
% 
%         % Calculate distances from previous point to each intersection point
%         dist1 = norm(pos_1 - prev_point);
%         dist2 = norm(pos_2 - prev_point);
% 
%         % Choose the closest intersection point
%         if dist1 < dist2
%             pos = vpa(pos_1);
%         else
%             pos = vpa(pos_2);
%         end
%     end
% end