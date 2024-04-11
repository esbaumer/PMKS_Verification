clear; close all; clc;

% Use the function to find the project root
% currentDir = pwd; % Current directory
% projectRoot = GeneralUtils.findProjectRoot(currentDir, 'PMKS_Simulator_Verification');
% 
% % Specify the path to CommonUtils relative to the project root
% utilsFolderPath = fullfile(projectRoot, 'CommonUtils');
% 
% % Add this path to MATLAB's search paths
% addpath(utilsFolderPath);
% Get the current script's directory
currentDir = fileparts(mfilename('fullpath'));

% Construct the path to the 'CommonUtils' directory
utilsFolderPath = fullfile(currentDir, '..', '..', '..', 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

% Define Coordinates of Joints in 3D Space (x, y, z)
A = [0 0 0];
B = [424.82 0 0];
C = [1232.05 -667.56 0];
D = [984 0 0];
% This represents the positions of the sensors 
E = [200 0 0]; % input link
F = [521.54 -701.15 0]; % Coupler link 1
G = [1101.53 -827.25 0]; % Coupler link 2
H = [1147.4 -280.00 0]; % Follower Link

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Joint.D = D;

Mechanism.TracerPoint.E = E;
Mechanism.TracerPoint.F = F;
Mechanism.TracerPoint.G = G;
Mechanism.TracerPoint.H = H;

% Define masses for each link or joint
Mechanism.LinkCoM.ABE = [274.02 -3.32 0];
Mechanism.LinkCoM.BCFG = [834.02 -531.72 0];
Mechanism.LinkCoM.CDH = [1174.20 -351.61 0];


% Mechanism.LinkCoM.ABE = [274.02 -3.32 21.3];
% Mechanism.LinkCoM.BCFG = [834.02 -531.72 40.35];
% Mechanism.LinkCoM.CDH = [1174.20 -351.61 21.30];


% Mass for each link
Mechanism.Mass.ABE = 470.19;
Mechanism.Mass.BCFG = 736.82;
Mechanism.Mass.CDH = 515.32;

% Mass moment of inertia for each link
Mechanism.MassMoI.ABE = 325979.23;
Mechanism.MassMoI.BCFG = 74929853.54;
Mechanism.MassMoI.CDH = 2177460.20;

% Desired for Stress Analysis. Maybe wanna include all the lengths to be
% utilized within PosSolver
Mechanism.ABELength = 10;
Mechanism.BCFGLength = 10;
Mechanism.CDHLength = 10;

% Desired for Stress Analysis. Another idea that is since we know the
% density, the mass, and the depth of the link, we could determine what the
% cross sectional area would be. But for now, I think hard coding these
% values are okay
Mechanism.crossSectionalAreaABE = 10;
Mechanism.crossSectionalAreaBCFG = 10;
Mechanism.crossSectionalAreaCDH = 10;

% Define the modulus of elasticity for each link
Mechanism.modulusElasticity = 10e6;

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

Mechanism = StressSolver(Mechanism, scenarios);
save('Mechanism.mat', 'Mechanism');

csvDir = 'CSVOutput';

baseDir = 'Kin';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

baseDir = 'Force';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

baseDir = 'Stress';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);
