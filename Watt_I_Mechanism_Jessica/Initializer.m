Initialization
clear; close all; clc;

% Initilize path to call functions within Utils
utilsFolderPath = fullfile(pwd);
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

% Define angular velocity of the link where a motor is attached
input_speed = 1.0472; % 10 rpm to 1.0472 rad/s

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

% Generate forceResults.csv
% saveDataToCSV('Force', 'forceResults.csv');
% 
% % Generate kinematicResults.csv
% saveDataToCSV('Kin', 'kinematicResults.csv');
% 
% 
% function saveDataToCSV(baseDir, outputFile)
%     % Initialize an empty cell array to store the data
%     csvData = {};
% 
%     % Recursively process each file in the directory and collect data
%     csvData = processData(baseDir, '', csvData);
% 
%     % Convert collected data to a table and write to CSV
%     dataTable = cell2table(csvData, 'VariableNames', {'Path', 'Value'});
%     writetable(dataTable, outputFile);
% end
% 
% function csvData = processData(folderPath, pathPrefix, csvData)
%     List all items in the directory
%     items = dir(folderPath);
%     for i = 1:length(items)
%         if ~ismember(items(i).name, {'.', '..'})
%             currentPath = fullfile(pathPrefix, items(i).name);
%             if items(i).isdir
%                 If the item is a directory, recurse into it
%                 csvData = processData(fullfile(folderPath, items(i).name), currentPath, csvData);
%             else
%                 Process file
%                 filePath = fullfile(folderPath, items(i).name);
%                 data = load(filePath);
%                 fieldName = fieldnames(data);
%                 Assuming there's only one field per .mat file
%                 if ~isempty(fieldName)
%                     value = data.(fieldName{1});
%                     Assuming value can be converted to string with mat2str or num2str
%                     if isnumeric(value) || islogical(value)
%                         valueStr = num2str(value(:).');
%                     else
%                         valueStr = mat2str(value);
%                     end
%                     Add to the csvData cell array
%                     csvData{end+1, 1} = [currentPath, '/', fieldName{1}]; % Path
%                     csvData{end, 2} = valueStr; % Value
%                 end
%             end
%         end
%     end
% end