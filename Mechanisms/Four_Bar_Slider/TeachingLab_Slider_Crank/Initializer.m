% Initialization
clear; close all; clc;

% Use the function to find the project root
% currentDir = pwd; % Current directory
% projectRoot = GeneralUtils.findProjectRoot(currentDir, 'PMKS_Simulator_Verification');
% 
% % Specify the path to CommonUtils relative to the project root
% utilsFolderPath = fullfile(projectRoot, 'CommonUtils');

% Get the current script's directory
currentDir = fileparts(mfilename('fullpath'));

% Construct the path to the 'CommonUtils' directory
utilsFolderPath = fullfile(currentDir, '..', '..', '..', 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

A=[0 0 0]; %motor input
B=[0,0.381,0]; %connection between crankshaft and connecting rod
C=[0.14756,0,0]; %connecting rod and piston 

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Theta = 0;

% Define Tracer Points 
Mechanism.TracerPoint = struct();

% Define masses for each link or joint
Mechanism.LinkCoM.AB = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BC = Utils.determineCoM([B; C]);

% Define masses for each link
Mechanism.Mass.AB = 1.08532;
Mechanism.Mass.BC= 0.50144;
Mechanism.Mass.Piston = 1.31788;

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.0004647594;
Mechanism.MassMoI.BC = 0.0030344427; 

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
input_speed = 15.707963249999972; % 150 rpm to 15.707963249999972 rad/s
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
