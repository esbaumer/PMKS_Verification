% Initialization
clear; close all; clc;

% Use the function to find the project root
currentDir = pwd; % Current directory
projectRoot = GeneralUtils.findProjectRoot(currentDir, 'PMKS_Simulator_Verification');

% Specify the path to CommonUtils relative to the project root
utilsFolderPath = fullfile(projectRoot, 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

% Define Coordinates of Joints in 3D Space (x, y, z)
A = [-2.14 3.025 0];
B = [-0.719 3.025 0];
C = [1.85 5.83 0];
D = [2.14 3.025 0];
E = [0.228 5 0];
F = [-0.207 6.48 0];
G = [2.3 4.52 0];

massAB = 13.68;
massBCEF = 33.07;
massCDG = 57.7;

massMoIAB = 0.43;
massMoIBCEF = 5.75;
massMoICDG = 30.36;

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Joint.D = D;
Mechanism.TracerPoint.E = E;
Mechanism.TracerPoint.F = F;
Mechanism.TracerPoint.G = G;

% Define masses for each link or joint
Mechanism.LinkCoM.AB = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BCEF = Utils.determineCoM([B; C; E; F]);
Mechanism.LinkCoM.CDG = Utils.determineCoM([C; D; G]);

% Define masses for each link
Mechanism.Mass.AB = 5; 
Mechanism.Mass.BCEF = 10;
Mechanism.Mass.CDG = 5; 

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.1; 
Mechanism.MassMoI.BCEF = 0.2; 
Mechanism.MassMoI.CDG = 0.1; 

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
% Mechanism = ForceSolver(Mechanism, scenarios);
% save('Mechanism.mat', 'Mechanism');

baseDir = 'Kin';
csvDir = 'CSVOutput';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

% baseDir = 'Force';
% GeneralUtils.exportMatricesToCSV(baseDir, csvDir);
