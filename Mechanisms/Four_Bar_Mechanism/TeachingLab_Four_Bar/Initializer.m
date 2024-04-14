% Initialization
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

currentDir = fileparts(mfilename('fullpath'));

% Construct the path to the 'CommonUtils' directory
utilsFolderPath = fullfile(currentDir, '..', '..', '..', 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

% Define Coordinates of Joints in 3D Space (x, y, z)
A = [-2.14 3.025 0];
B = [-0.719 3.025 0];
C = [1.85 5.83 0];
D = [2.14 3.025 0];

% Tracer Points for where the sensors are located
E = [0.228 5 0]; % Coolterm, 1
F = [-0.207 6.48 0]; % Coolterm, 2
G = [2.3 4.52 0]; % Coolterm, 3
H = [-0.207 6.48 0]; % WitMotion, 1
I = [2.3 4.52 0]; % WitMotion, 2

massABEH = 13.68;
massBCFG = 33.07;
massCDI = 57.7;

massMoIABEH = 0.43;
massMoIBCFG = 5.75;
massMoICDI = 30.36;

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Joint.D = D;

Mechanism.TracerPoint.E = E; % Coolterm, 1
Mechanism.TracerPoint.F = F; % Coolterm, 2
Mechanism.TracerPoint.G = G; % Coolterm, 3
Mechanism.TracerPoint.H = H; % WitMotion, 1
Mechanism.TracerPoint.I = I; % WitMotion, 2

% Define masses for each link or joint
Mechanism.LinkCoM.ABEH = [0 0 0];
Mechanism.LinkCoM.BCFG = [0 0 0];
Mechanism.LinkCoM.CDI = [0 0 0];

% Define masses for each link
Mechanism.Mass.ABEH = 5; 
Mechanism.Mass.BCFG = 10;
Mechanism.Mass.CDI = 5; 

% Define mass moments of inertia for each link
Mechanism.MassMoI.ABEH = 0.1; 
Mechanism.MassMoI.BCFG = 0.2; 
Mechanism.MassMoI.CDI = 0.1; 

% Desired for Stress Analysis. Maybe wanna include all the lengths to be
% utilized within PosSolver
Mechanism.ABEHLength = 10;
Mechanism.BCFGLength = 10;
Mechanism.CDILength = 10;

% Desired for Stress Analysis. Another idea that is since we know the
% density, the mass, and the depth of the link, we could determine what the
% cross sectional area would be. But for now, I think hard coding these
% values are okay
Mechanism.crossSectionalAreaABEH = 10;
Mechanism.crossSectionalAreaBCFG = 10;
Mechanism.crossSectionalAreaCDI = 10;

% Define angular velocity of the link where a motor is attached
input_speed = zeros(1, 3);
input_speed(1) = GeneralUtils.rpmToRadPerSec(10);
input_speed(2) = GeneralUtils.rpmToRadPerSec(20);
input_speed(3) = GeneralUtils.rpmToRadPerSec(30);

input_speed_str = [10, 20, 30];

Mechanism.input_speed_str = input_speed_str;

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

% Mechanism = StressSolver(Mechanism, scenarios);
% save('Mechanism.mat', 'Mechanism');

csvDir = 'CSVOutput';

baseDir = 'Kin';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

% baseDir = 'Force';
% GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

% baseDir = 'Stress';
% GeneralUtils.exportMatricesToCSV(baseDir, csvDir);
