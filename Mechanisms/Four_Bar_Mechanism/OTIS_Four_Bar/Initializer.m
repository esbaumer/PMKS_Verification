clear; close all; clc;

% Initilize path to call functions within Utils
utilsFolderPath = fullfile(pwd);
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

Mechanism.Mass.ABE = 470.19;
Mechanism.Mass.BCFG = 736.82;
Mechanism.Mass.CDH = 515.32;

Mechanism.MassMoI.ABE = 325979.23;
Mechanism.MassMoI.BCFG = 74929853.54;
Mechanism.MassMoI.CDH = 2177460.20;

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

% Define angular velocity of the link where a motor is attached
input_speed = 1.0472; % 10 rpm to 1.0472 rad/s

% Call PosSolver to calculate and store positions
Mechanism = PosSolver(Mechanism, input_speed);

% Call VelAccSolver to calculate and store velocities and accelerations
Mechanism = VelAccSolver(Mechanism);

% Call ForceSolver to calculate and store forces and torques
%     % Scenarios: [newtonFlag, gravityFlag, frictionFlag]
%     % scenarios = [0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1];
scenarios = [1 1 0];
Mechanism = ForceSolver(Mechanism, scenarios);
%% 

% Optionally, save the fully initialized and solved Mechanism structure for later use
save('Mechanism.mat', 'Mechanism');

baseDir = 'Kin';
csvDir = 'CSVOutput';
exportMatricesToCSV(baseDir, csvDir);

baseDir = 'Force';
exportMatricesToCSV(baseDir, csvDir);


function exportMatricesToCSV(baseDir, csvDir)
    % Create CSV directory if it doesn't exist
    if ~exist(csvDir, 'dir')
        mkdir(csvDir);
    end
    
    % Process each .mat file
    processDirectory(baseDir, baseDir, csvDir);
end

function processDirectory(baseDir, currentDir, csvDir)
    items = dir(currentDir);
    for i = 1:length(items)
        if items(i).isdir && ~ismember(items(i).name, {'.', '..'})
            % If it's a subdirectory, recursively process it
            processDirectory(baseDir, fullfile(currentDir, items(i).name), csvDir);
        elseif ~items(i).isdir
            % Process .mat file
            matFilePath = fullfile(currentDir, items(i).name);
            data = load(matFilePath);
            fieldName = fieldnames(data);
            if ~isempty(fieldName)
                matrix = data.(fieldName{1});
                if isnumeric(matrix) && size(matrix, 2) == 3
                    % Construct CSV file path
                    relPath = strrep(currentDir, baseDir, ''); % Relative path
                    csvFileName = strrep(items(i).name, '.mat', '.csv');
                    csvFilePath = fullfile(csvDir, relPath, csvFileName);
                    
                    % Ensure subdirectory exists
                    [csvFileDir, ~, ~] = fileparts(csvFilePath);
                    if ~exist(csvFileDir, 'dir')
                        mkdir(csvFileDir);
                    end
                    
                    % Write matrix to CSV
                    writeMatrixToCSV(matrix, csvFilePath);
                end
            end
        end
    end
end

function writeMatrixToCSV(matrix, csvFilePath)
    % Open CSV file
    fileId = fopen(csvFilePath, 'w');
    % Check if the file is opened successfully
    if fileId == -1
        error('Failed to open file for writing: %s', csvFilePath);
    end
    % Write each row of the matrix to the CSV file
    for i = 1:size(matrix, 1)
        fprintf(fileId, '%f,%f,%f\n', matrix(i, 1), matrix(i, 2), matrix(i, 3));
    end
    % Close the file
    fclose(fileId);
end
