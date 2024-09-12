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

A=[13.79,10.01,0]; % motor input
B=[13.79,13.82,0]; % connection between crankshaft and connecting rod
C=[-0.97,10.01,0]; % connecting rod and piston 
E=[13.79,13.82,0]; % BNO sensor attached to the connecting rod

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Theta = 0;

% Define Tracer Points 
Mechanism.TracerPoint.E = E; % BNO Sensor, Python Graph

% Define masses for each link or joint
Mechanism.LinkCoM.AB = [0 0.1 0];
Mechanism.LinkCoM.BCE = [0.5 0.7 0];

% Define angles for each link
Mechanism.Angle.AB = [0 0 rad2deg(atan2((Mechanism.LinkCoM.AB(2) - A(2)), Mechanism.LinkCoM.AB(1) - A(1)))];
Mechanism.Angle.BCE = [0 0 rad2deg(atan2((Mechanism.LinkCoM.BCE(2) - B(2)), Mechanism.LinkCoM.BCE(1) - B(1)))];

% Define masses for each link
Mechanism.Mass.AB = 1.08532;
Mechanism.Mass.BCE= 0.50144;
Mechanism.Mass.Piston = 1.31788;

% Define mass moments of inertia for each link
Mechanism.MassMoI.AB = 0.0004647594;
Mechanism.MassMoI.BCE = 0.0030344427; 

% Desired for Stress Analysis. Maybe wanna include all the lengths to be
% utilized within PosSolver
Mechanism.ABLength = 10;
Mechanism.BCELength = 10;

% Desired for Stress Analysis. Another idea that is since we know the
% density, the mass, and the depth of the link, we could determine what the
% cross sectional area would be. But for now, I think hard coding these
% values are okay
Mechanism.crossSectionalAreaAB = 10;
Mechanism.crossSectionalAreaBCE = 10;
Mechanism.crossSectionalAreaCD = 10;

% Define the modulus of elasticity for each link
Mechanism.modulusElasticity = 10e6;

% Define angular velocity of the link where a motor is attached
% input_speed = zeros(1, 3);
input_speed = zeros(1,2);
input_speed(1) = GeneralUtils.rpmToRadPerSec(35.8);
input_speed(2) = GeneralUtils.rpmToRadPerSec(50.3);
% input_speed(3) = GeneralUtils.rpmToRadPerSec(70);

input_speed_str = [35.8, 50.3];
% input_speed_str = [15];

Mechanism.input_speed_str = input_speed_str;

% Define angular velocity of the link where a motor is attached
% input_speed = 15.707963249999972; % 150 rpm to 15.707963249999972 rad/s
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

% Mechanism = StressSolver(Mechanism, scenarios);
% save('Mechanism.mat', 'Mechanism');

csvDir = 'CSVOutput';

baseDir = 'Kin';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

baseDir = 'Force';
GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

% baseDir = 'Stress';
% GeneralUtils.exportMatricesToCSV(baseDir, csvDir);

load('Mechanism')

% Define a map from sensors to their respective data types
sensorDataTypes = containers.Map(...
    {'E'}, ...
    {...
    {'Angle', 'AngVel'}, ...  % Data types for sensor B
    }...
    );

sensorSourceMap = containers.Map({'E'}, {'PythonGraph'});

Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap);
