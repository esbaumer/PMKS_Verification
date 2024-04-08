% Initialization
clear; close all; clc;

% Use the function to find the project root
% currentDir = pwd; % Current directory
% projectRoot = GeneralUtils.findProjectRoot(currentDir, 'PMKS_Simulator_Verification');
% projectRoot = fullfile(currentDir, '..', '..', '..', 'CommonUtils');

% Specify the path to CommonUtils relative to the project root
% utilsFolderPath = fullfile(projectRoot, 'CommonUtils');

% Get the current script's directory
currentDir = fileparts(mfilename('fullpath'));

% Construct the path to the 'CommonUtils' directory
utilsFolderPath = fullfile(currentDir, '..', '..', '..', 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

% Initialize Joint positions
A = [0 0 0];
B = [4 2 0];
C = [12 0 0];
D = [20 2 0];

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.TracerPoint.D = D;

% Define masses for each link or joint
Mechanism.LinkCoM.AB = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BCD = Utils.determineCoM([B; C; D]);

% Define masses for each link
Mechanism.Mass.AB = 5; 
Mechanism.Mass.BCD= 10;

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.1; 
Mechanism.MassMoI.BCD = 0.2; 

% Define angular velocity of the link where a motor is attached
input_speed = 1.0472; % 10 rpm to 1.0472 rad/s
save('Mechanism.mat', 'Mechanism');

% Call PosSolver to calculate and store positions
Mechanism = PosSolver(Mechanism, input_speed);
save('Mechanism.mat', 'Mechanism');

% Call VelAccSolver to calculate and store velocities and accelerations
Mechanism = VelAccSolver(Mechanism);
save('Mechanism.mat', 'Mechanism');

% Call ForceSolver to calculate and store forces and torques
%     % Scenarios: [newtonFlag, gravityFlag, frictionFlag]
%     % scenarios = [0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1];
% scenarios = [1 1 0];
% Mechanism = ForceSolver(Mechanism);
% save('Mechanism.mat', 'Mechanism');

baseDir = 'Kin';
csvDir = 'CSVOutput';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

% baseDir = 'Force';
% exportMatricesToCSV(baseDir, csvDir);
