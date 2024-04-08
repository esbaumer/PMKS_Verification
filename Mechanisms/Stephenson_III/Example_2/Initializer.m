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
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

massAB = 13.68;
massBC = 33.07;
massDCE = 57.7;
massEF = 27.54;
massFG = 59.94;

massMoIAB = 0.43;
massMoIBC = 5.75;
massMoIDCE = 30.36;
massMoIEF = 3.34;
massMoIFG = 34.03;

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Joint.D = D;
Mechanism.Joint.E = E;
Mechanism.Joint.F = F;
Mechanism.Joint.G = G;

% Define Tracer Points 
Mechanism.TracerPoint = struct();

% Define masses for each link or joint
Mechanism.LinkCoM.AB = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BCE = Utils.determineCoM([B; C; E]);
Mechanism.LinkCoM.CD = Utils.determineCoM([C; D]);
Mechanism.LinkCoM.EF = Utils.determineCoM([E; F]);
Mechanism.LinkCoM.FG = Utils.determineCoM([F; G]);

% Define masses for each link
Mechanism.Mass.AB = 5; 
Mechanism.Mass.BCE = 10;
Mechanism.Mass.CD = 5; 
Mechanism.Mass.EF = 10;
Mechanism.Mass.FG = 5; 

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.1; 
Mechanism.MassMoI.BCE = 0.2; 
Mechanism.MassMoI.CD = 0.1; 
Mechanism.MassMoI.EF = 0.2; 
Mechanism.MassMoI.FG = 0.1;  

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
scenarios = [1 1 0];
Mechanism = ForceSolver(Mechanism, scenarios);
save('Mechanism.mat', 'Mechanism');

baseDir = 'Kin';
csvDir = 'CSVOutput';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

baseDir = 'Force';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);
