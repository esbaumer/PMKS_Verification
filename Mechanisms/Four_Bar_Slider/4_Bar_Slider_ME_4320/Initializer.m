% Initialization
clear; close all; clc;

% Initilize path to call functions within Utils
utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

% Initialize Mechanism structure with necessary fields
Mechanism = struct();

A=[0 0 0]; %motor input
B=[0,3.81,0]; %connection between crankshaft and connecting rod
C=[14.756,0,0]; %connecting rod and piston 

% Define initial joint positions (example values)
Mechanism.Joint.A = A;
Mechanism.Joint.B = B;
Mechanism.Joint.C = C;
Mechanism.Theta = 0;

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

% Define angular velocity of the link where a motor is attached
input_speed = 15.707963249999972; % 150 rpm to 15.707963249999972 rad/s

% Call PosSolver to calculate and store positions
Mechanism = PosSolver(Mechanism, input_speed);

% Call VelAccSolver to calculate and store velocities and accelerations
Mechanism = VelAccSolver(Mechanism);

% Call ForceSolver to calculate and store forces and torques 
Mechanism = ForceSolver(Mechanism);

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
