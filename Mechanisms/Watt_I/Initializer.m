clear; close all; clc;

% % Use the function to find the project root
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
utilsFolderPath = fullfile(currentDir, '..', '..', 'CommonUtils');

% Add this path to MATLAB's search paths
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

% Initialize Joint positions
A = [-3.74, -2.41, 0];
B = [-2.72, 0.91, 0];
C = [1.58, 0.43, 0];
D = [-0.24, 4.01, 0]; 
E = [5.08, 5.31, 0]; 
F = [8.14, 3.35, 0];
G = [7.32, -3.51, 0];

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
Mechanism.LinkCoM.BCD = Utils.determineCoM([B; C; D]);
Mechanism.LinkCoM.DE = Utils.determineCoM([D; E]);
Mechanism.LinkCoM.EF = Utils.determineCoM([E; F]);
Mechanism.LinkCoM.CFG = Utils.determineCoM([C; F; G]);

% Define masses for each link
Mechanism.Mass.AB = 5; 
Mechanism.Mass.BCD = 10;
Mechanism.Mass.DE = 5; 
Mechanism.Mass.EF = 10;
Mechanism.Mass.CFG = 5; 

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.1; 
Mechanism.MassMoI.BCD = 0.2; 
Mechanism.MassMoI.DE = 0.1; 
Mechanism.MassMoI.EF = 0.2; 
Mechanism.MassMoI.CFG = 0.1;  

% Desired for Stress Analysis. Maybe wanna include all the lengths to be
% utilized within PosSolver
Mechanism.ABLength = 10;
Mechanism.BCDLength = 10;
Mechanism.DELength = 10;
Mechanism.EFLength = 10;
Mechanism.CFGLength = 10;

% Desired for Stress Analysis. Another idea that is since we know the
% density, the mass, and the depth of the link, we could determine what the
% cross sectional area would be. But for now, I think hard coding these
% values are okay
Mechanism.crossSectionalAreaAB = 10e-4; % square meters
Mechanism.crossSectionalAreaBCD = 10e-4; % square meters
Mechanism.crossSectionalAreaDE = 10e-4; % square meters
Mechanism.crossSectionalAreaEF = 10e-4; % square meters
Mechanism.crossSectionalAreaCFG = 10e-4; % square meters

% Define the modulus of elasticity for each link
Mechanism.modulusElasticity = 200e9;

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
