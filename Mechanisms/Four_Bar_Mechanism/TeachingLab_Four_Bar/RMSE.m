clear; close all; clc;

% Define the base paths for theoretical data
baseTheoreticalVelPath = 'CSVOutput/Vel/AngVel/';

TheoreticalPath = 'CSVOutput';

% Define the base paths for experimental data
ExperimentalPath = 'Experimental';
CoolTermExperimentalPath = 'Experimental/CoolTerm';
WitMotionExperimentalPath = 'Experimental/WitMotion';

% Define sensors and their corresponding links
sensors = {'E', 'F', 'G', 'H', 'I'};
sensorToLinkMap = containers.Map({'E', 'F', 'G', 'H', 'I'}, {'ABEH', 'BCFG', 'BCFG', 'ABEH', 'CDI'});

% Define speeds
speeds = {'f10RPM', 'f20RPM', 'f30RPM'};

expData = readExperimentalData(ExperimentalPath);
theoData = readTheoreticalData(TheoreticalPath);

% Define a map from sensors to their respective data types
sensorDataTypes = containers.Map(...
    {'E', 'F', 'G', 'H', 'I'}, ...
    {...
    {'Angle', 'LinVel'}, ...  % Data types for sensor E
    {'AngVel', 'LinAcc'}, ... % Data types for sensor F
    {'AngAcc', 'LinVel'}, ... % Data types for sensor G
    {'Angle', 'AngVel', 'LinAcc'}, ...  % Data types for sensor H
    {'AngVel', 'LinVel', 'AngAcc'}  ... % Data types for sensor I
    }...
    );

% Calculate RMSE for all sensors according to specified data types in the map
rmseResults = struct();  % Initialize an empty struct to hold all results

for sensor = keys(sensorDataTypes)
    currentSensor = sensor{1};
    dataTypes = sensorDataTypes(currentSensor);  % Retrieve data types for current sensor
    % Compute RMSE for the current sensor across its specified data types
    rmseResults.(currentSensor) = calculateRMSEForSensor(expData, theoData, currentSensor, dataTypes, speeds);
end

% Save results to CSV
resultsFilename = 'RMSE_Results.csv';
saveResultsToCSV(rmseResults, resultsFilename);

% Display results
% disp('RMSE Calculation Results:');
% disp(rmseResults);

% Functions needed to store experimental and theoretical data
% function dataStruct = readTheoreticalData(basePath)
% categoryMap = containers.Map({'Acc', 'Vel', 'Pos'}, ...
%     {{'AngAcc', 'LinAcc'}, {'AngVel', 'LinVel'}, {'', ''}});
% dataStruct = struct(); % Initialize the main data structure
%
% for k = categoryMap.keys
%     category = k{1};
%     subCategories = categoryMap(category);
%     categoryPath = fullfile(basePath, category);
%
%     for subCategory = subCategories
%         subCategoryPath = fullfile(categoryPath, subCategory{1});
%         if any(strcmp(subCategory{1}, {'LinAcc', 'LinVel'})) || strcmp(category, 'Pos')
%             dataStruct = processNestedDirectories(subCategoryPath, dataStruct, category, subCategory{1});
%         else
%             dataStruct = processSpeedDirectories(subCategoryPath, dataStruct, category, subCategory{1}, '');
%         end
%     end
% end
% end
% function dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable)
%     % Ensure the base category structure exists
%     if ~isfield(dataStruct, category)
%         dataStruct.(category) = struct();
%     end
%
%     % Handle potential empty subCategory appropriately
%     if isempty(subCategory)
%         subCategoryField = 'defaultSubCategory';  % Use a default subcategory name if empty
%     else
%         subCategoryField = subCategory;
%     end
%
%     % Ensure the sub-category structure exists
%     if ~isfield(dataStruct.(category), subCategoryField)
%         dataStruct.(category).(subCategoryField) = struct();
%     end
%
%     % Assign final target for structure updating
%     finalTarget = dataStruct.(category).(subCategoryField);
%
%     % Handle nested directory if provided
%     if ~isempty(nestedDir)
%         if ~isfield(finalTarget, nestedDir)
%             finalTarget.(nestedDir) = struct();
%         end
%         finalTarget = finalTarget.(nestedDir);
%     end
%
%     % Ensure the item structure exists
%     if ~isfield(finalTarget, itemName)
%         finalTarget.(itemName) = struct();
%     end
%
%     % Assign data table to the correct field based on speedTag
%     if ~isempty(speedTag)
%         finalTarget.(itemName).(speedTag) = dataTable;
%     else
%         finalTarget.(itemName).default = dataTable;  % Use default if no speed tag is present
%     end
%
%     % Update the main data structure with modified data
%     if ~isempty(nestedDir)
%         dataStruct.(category).(subCategoryField).(nestedDir) = finalTarget;
%     else
%         dataStruct.(category).(subCategoryField) = finalTarget;
%     end
% end
% function dataStruct = processNestedDirectories(path, dataStruct, category, subCategory)
% % This function processes directories for LinAcc, LinVel and Pos that have nested Joint/LinkCoM directories
% subDirs = {'Joint', 'LinkCoM'}; % Possible nested directories
% for subDir = subDirs
%     nestedPath = fullfile(path, subDir{1});
%     if isfolder(nestedPath)
%         dataStruct = processSpeedDirectories(nestedPath, dataStruct, category, subCategory, subDir{1});
%     end
% end
% end
% function dataStruct = processSpeedDirectories(path, dataStruct, category, subCategory, nestedDir)
% % This function processes directories that include different speeds, applicable to all categories now
% speeds = dir(path);
% for speed = speeds'
%     if speed.isdir && ~ismember(speed.name, {'.', '..'})
%         speedPath = fullfile(path, speed.name);
%         dataStruct = readDataFromDirectory(speedPath, dataStruct, category, subCategory, nestedDir);
%     else
%         speedPath = path;
%         dataStruct = readDataFromDirectory(speedPath, dataStruct, category, subCategory, nestedDir);
%     end
% end
% end
% function dataStruct = readDataFromDirectory(path, dataStruct, category, subCategory, nestedDir)
% csvFiles = dir(fullfile(path, '*.csv'));
% for file = csvFiles'
%     % Extract information from the filename; handle optional speed tag
%     % tokens = regexp(file.name, '^(.+?)(?:_speed(\d+))?\.csv$', 'tokens');
%     tokens = regexp(file.name, '^(.+?)\.csv$', 'tokens');
%     if isempty(tokens) || isempty(tokens{1})
%         continue; % Skip if filename doesn't match expected format
%     end
%
%     itemName = tokens{1}{1}; % Item name is always the first token, directly accessed
%     speedTag = getSpeedTagFromPath(path); % Extract speed tag from the directory path
%     % speedTag = ''; % Default speed tag is empty
%     if length(tokens{1}) > 1 && ~isempty(tokens{1}{2})
%         speedTag = ['speed', tokens{1}{2}]; % Ensure 'speed' prefix is used
%     end
%
%     % Read the CSV file into a table
%     dataTable = readtable(fullfile(file.folder, file.name));
%
%     % Update the data structure
%     dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable);
% end
% end
% function speedTag = getSpeedTagFromPath(path)
% % Helper function to extract the speed tag from the directory path
% [~, lastDir] = fileparts(path); % Extracts the last directory name which is expected to be the speed
% speedTag = lastDir; % Directly use the directory name as the speed tag
% end

function dataStruct = readTheoreticalData(basePath)
% Define mappings of categories to their relevant subcategories
categoryMap = containers.Map({'Acc', 'Vel', 'Pos'}, ...
    {{'AngAcc', 'LinAcc'}, {'AngVel', 'LinVel'}, {'Joint', 'LinkCoM'}});  % No subcategories for Pos

dataStruct = struct(); % Initialize the main data structure

% Iterate over each category like Acc, Vel, Pos
for k = categoryMap.keys
    category = k{1};
    subCategories = categoryMap(category);
    categoryPath = fullfile(basePath, category);

    % Process each subcategory appropriately

    for subCategory = subCategories
        subCategoryPath = fullfile(categoryPath, subCategory{1});
        if strcmp(subCategory{1}, 'LinAcc') || strcmp(subCategory{1}, 'LinVel')
            dataStruct = processNestedDirectories(subCategoryPath, dataStruct, category, subCategory{1});
        elseif strcmp(category, 'Pos')
            dataStruct = processNestedDirectories(categoryPath, dataStruct, category, '');
        else
            dataStruct = processSpeedDirectories(subCategoryPath, dataStruct, category, subCategory{1}, '');
        end
    end
end
end

function dataStruct = processNestedDirectories(path, dataStruct, category, subCategory)
% Process directories for LinAcc, LinVel that have nested Joint/LinkCoM directories
subDirs = {'Joint', 'LinkCoM'};
for subDir = subDirs
    nestedPath = fullfile(path, subDir{1});
    if isfolder(nestedPath)
        dataStruct = processSpeedDirectories(nestedPath, dataStruct, category, subCategory, subDir{1});
    end
end
end

function dataStruct = processSpeedDirectories(path, dataStruct, category, subCategory, nestedDir)
% Process directories that include different speeds or default data
speeds = dir(path);
for speed = speeds'
    if speed.isdir && ~ismember(speed.name, {'.', '..'})
        speedPath = fullfile(path, speed.name);
        dataStruct = readDataFromDirectory(speedPath, dataStruct, category, subCategory, nestedDir);
    else
        dataStruct = readDataFromDirectory(path, dataStruct, category, subCategory, nestedDir);
    end
end
end

function dataStruct = readDataFromDirectory(path, dataStruct, category, subCategory, nestedDir)
csvFiles = dir(fullfile(path, '*.csv'));
for file = csvFiles'
    % Handle optional speed tag in file name
    tokens = regexp(file.name, '^(.+?)\.csv$', 'tokens');
    if isempty(tokens) || isempty(tokens{1})
        continue;
    end

    itemName = tokens{1}{1};
    speedTag = getSpeedTagFromPath(path);

    dataTable = readtable(fullfile(file.folder, file.name));
    dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable);
end
end

function speedTag = getSpeedTagFromPath(path)
[~, lastDir] = fileparts(path); % Extracts the last directory name
if contains(lastDir, 'RPM')
    speedTag = lastDir; % Use the directory name as the speed tag if it contains 'RPM'
else
    speedTag = ''; % Set speed tag as empty if 'RPM' is not found
end
end

function dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable)
if ~isfield(dataStruct, category)
    dataStruct.(category) = struct();
end

if strcmp(category, 'Pos') % Pos, which is a special case
    finalTarget = dataStruct.(category);
    if ~isempty(nestedDir)
        if ~isfield(finalTarget, nestedDir)
            finalTarget.(nestedDir) = struct();
        end
        finalTarget = finalTarget.(nestedDir);
    end

    if ~isfield(finalTarget, itemName)
        finalTarget.(itemName) = struct();
    end

        finalTarget.(itemName) = dataTable;

        dataStruct.(category).(nestedDir) = finalTarget;
    return;
end

subCategoryField = subCategory;

if ~isfield(dataStruct.(category), subCategoryField)
    dataStruct.(category).(subCategoryField) = struct();
end

finalTarget = dataStruct.(category).(subCategoryField);
if ~isempty(nestedDir)
    if ~isfield(finalTarget, nestedDir)
        finalTarget.(nestedDir) = struct();
    end
    finalTarget = finalTarget.(nestedDir);
end

if ~isfield(finalTarget, itemName)
    finalTarget.(itemName) = struct();
end

if ~isempty(speedTag)
    finalTarget.(itemName).(speedTag) = dataTable;
else
    finalTarget.(itemName).default = dataTable;
end

if isempty(nestedDir)
    dataStruct.(category).(subCategoryField) = finalTarget;
else
    dataStruct.(category).(subCategoryField).(nestedDir) = finalTarget;
end
end


function expData = readExperimentalData(baseExperimentalPath)
expData = struct(); % Initialize
subFolders = {'CoolTerm', 'WitMotion'}; % Subdirectories to iterate through
filenames = {'10RPM', '20RPM', '30RPM'}; % RPM filenames

for i = 1:length(subFolders)
    % Initialize sub-structures for each subfolder
    expData.(subFolders{i}) = struct();
    currentPath = fullfile(baseExperimentalPath, subFolders{i}); % Path to current subdirectory

    for j = 1:length(filenames)
        safeFieldName = ['f' filenames{j}]; % Prepend 'f' to ensure the name starts with a letter

        % Construct file path
        if i == 1  % For 'CoolTerm', read XLSX files
            xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
            % Check and read XLSX file
            if isfile(xlsxPath)
                expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
                % expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath, 'Range', 'A1'); % Adjust 'Range' if necessary
            end
        else  % For 'WitMotion', read CSV files
            csvPath = fullfile(currentPath, filenames{j} + ".csv");
            % Check and read CSV file, including headers
            if isfile(csvPath)
                opts = detectImportOptions(csvPath);
                opts.Delimiter = ','; % Set the delimiter
                % Retain all header lines, modify if there's a different number of header rows
                opts.PreserveVariableNames = true;
                expData.(subFolders{i}).(safeFieldName) = readtable(csvPath, opts);
            end
        end
    end
end
end

% Function to calculate RMSE for a given sensor and its data types
function results = calculateRMSEForSensor(expData, theoData, sensor, dataTypes, speeds)
results = struct();
for dataType = dataTypes
    results.(dataType{1}) = struct();  % Initialize a struct for each data type
    for speed = speeds
        % Calculate RMSE using a hypothetical function, for a given dataType and speed
        rmseValue = calculateRMSE(expData, theoData, sensor, dataType{1}, speed{1});
        % Store RMSE value in the struct under its corresponding speed
        results.(dataType{1}).(speed{1}) = rmseValue;
    end
end
end

% Retriev the desired experimental data
function expData = retrieveExpData(dataSet, sensor, dataType, speed)
% Map sensors to their respective data sources (CoolTerm or WitMotion)
sensorSourceMap = containers.Map({'E', 'F', 'G', 'H', 'I'}, ...
    {'CoolTerm', 'CoolTerm', 'CoolTerm', 'WitMotion', 'WitMotion'});
source = sensorSourceMap(sensor);

% Check if the required data is available
if isfield(dataSet, source) && isfield(dataSet.(source), speed)
    rawData = dataSet.(source).(speed); % nxm table of data
    expData = processData(rawData, sensor, dataType);
else
    expData = []; % Return empty if not found
end
end
function processedData = processData(rawData, sensor, dataType)
% Initialize processed data structure
processedData = struct('Time', [], 'Values', []);

% Define sensor-specific columns and process data accordingly
sensorDataMap = containers.Map(...
    {'E', 'F', 'G', 'H', 'I'}, ...
    {8:13, 4:7, 14:19, 1:3, 4:6} ...  % Example column indices for each sensor
    );
columnIndices = sensorDataMap(sensor);

% Extract data based on dataType
switch dataType
    % TODO: don't determine the mean, just pull the values based
    % from respective time domain. Most likely do a comparison
    % between the theoretical values to the experimental values and
    % get the values that most closely match the thoeretical values
    case 'Angle'
        processedData.Values = mean(rawData(:, columnIndices(1:3)), 2); % Example: mean of first three columns
    case 'LinVel'
        processedData.Values = rawData(:, columnIndices(4)); % Example: fourth column
    case 'AngVel'
        % Further processing can be added here
end

% Example time column extraction
processedData.Time = rawData(:, 2); % Assuming second column is time

% Filter data to remove outliers or bad data
processedData = filterData(processedData);
end

function filteredData = filterData(data)
% Example filtering operation
filteredData = data;
% goodIndices = data.Values < threshold; % Define 'threshold' based on your criteria
% filteredData = struct('Time', data.Time(goodIndices), 'Values', data.Values(goodIndices));
end
% Retriev the desired theoretical data
function theoData = retrieveTheoData(dataSet, sensor, dataType, speed)
% Placeholder for actual retrieval logic
% Example:
% TODO: Determine logic for pulling the theoretical data
if isfield(dataSet, sensor) && isfield(dataSet.(sensor), dataType) && isfield(dataSet.(sensor).(dataType), speed)
    theoData = dataSet.(sensor).(dataType).(speed);
else
    theoData = []; % Return empty if not found
end
end

function rmseResults = calculateRMSE(expDataSet, theoDataSet, sensor, dataType, speed)
rmseResults = struct(); % Initialize results structure

% Retrieve experimental and theoretical data for the given sensor, dataType, and speed
theoData = retrieveTheoData(theoDataSet, sensor, dataType, speed);
% TODO: Make sure you add theoData as a input argument
expData = retrieveExpData(expDataSet, theoData, sensor, dataType, speed);

% Calculate RMSE if both experimental and theoretical data are available
if ~isempty(expData) && ~isempty(theoData)
    timestamps = expData.Time;
    interpolatedTheoData = interp1(theoData.Time, theoData.Values, timestamps, 'linear', 'extrap');
    rmse = sqrt(mean((expData.Values - interpolatedTheoData).^2));

    % Store RMSE in the results structure
    rmseResults = rmse;
else
    warning('Missing data for sensor %s, data type %s, speed %s', sensor, dataType, speed);
    rmseResults = NaN; % Assign NaN to indicate missing data calculation
end

return;
end
% Function to save RMSE results to CSV
function saveResultsToCSV(rmseResults, filename)
fid = fopen(filename, 'w');
fprintf(fid, 'Sensor,DataType,Speed,RMSE\n');
for sensor = fieldnames(rmseResults)'
    for dataType = fieldnames(rmseResults.(sensor{1}))'
        for speed = fieldnames(rmseResults.(sensor{1}).(dataType{1}))'
            rmseValue = rmseResults.(sensor{1}).(dataType{1}).(speed{1});
            fprintf(fid, '%s,%s,%s,%f\n', sensor{1}, dataType{1}, speed{1}, rmseValue);
        end
    end
end
fclose(fid);
end

% function dataStruct = readDataFromDir(path, dataStruct, category, subCategory)
%     % Read data directly from directories that do not have further subdirectories
%     csvFiles = dir(fullfile(path, '*.csv'));
%     for file = csvFiles'
%         [itemName, speedTag] = parseFileName(file.name);
%         if isempty(itemName)
%             continue;
%         end
%         dataTable = readtable(fullfile(file.folder, file.name));
%         dataStruct = updateDataStruct(dataStruct, category, subCategory, '', itemName, speedTag, dataTable);
%     end
% end
%
% function dataStruct = readNestedData(path, dataStruct, category, subCategory, nestedDir)
%     % Read data from nested directories like Joint and LinkCoM under LinAcc and LinVel
%     csvFiles = dir(fullfile(path, '*.csv'));
%     for file = csvFiles'
%         [itemName, speedTag] = parseFileName(file.name);
%         if isempty(itemName)
%             continue;
%         end
%         dataTable = readtable(fullfile(file.folder, file.name));
%         dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable);
%     end
% end
% function linkAngVelData = readTheoreticalLinkData(basePath)
% linkNames = {'ABEH', 'BCFG', 'CDI'}; % Define the links you're interested in
% speeds = {'speed1', 'speed2', 'speed3'}; % Speed variants
% linkAngVelData = struct(); % Initialize empty struct
%
% for linkIdx = 1:length(linkNames)
%     for speedIdx = 1:length(speeds)
%         fileName = sprintf('%s.csv_%s.csv', linkNames{linkIdx}, speeds{speedIdx});
%         fullPath = fullfile(basePath, fileName);
%
%         if isfile(fullPath)
%             data = readtable(fullPath);
%             if ~isfield(linkAngVelData, linkNames{linkIdx})
%                 linkAngVelData.(linkNames{linkIdx}) = struct();
%             end
%             linkAngVelData.(linkNames{linkIdx}).(speeds{speedIdx}) = data;
%         else
%             warning('File does not exist: %s', fullPath);
%         end
%     end
% end
% end
%
% function adjustedAngles = determineAngles(jointData)
% % Assuming jointData is a structure or table with fields 'jointID' and 'angle'
% % Initialize an array to store adjusted angles with the same size as the input
% if ~isempty(jointData) && isfield(jointData(1), 'data')
%     % Assuming all 'data' fields have the same number of rows (361 in your case)
%     numberOfRows = 5;
%     % Adjusted initialization of adjustedAngles based on dynamic size
%     adjustedAngles = zeros(numberOfRows, length(jointData)); % Flipped to match 361x6
% else
%     % Handle the case where jointData might be empty or not properly structured
%     adjustedAngles = []; % Or any other fallback initialization
% end
%
% % Create a map from 'jointData'
% jointIndexMap = containers.Map('KeyType', 'char', 'ValueType', 'int32');
%
% for i = 1:length(jointData)
%     jointIndexMap(jointData(i).jointName) = i;
% end
%
% for theta_iterator = 1:size(jointData(1))
%     for rowNum = 1:5
%         % Determine the offset based on jointID. Example adjustments:
%         if theta_iterator == 1 % For joint A with respect to another joint
%             % Pull the appropriate joint values
%             A = table2array(jointData(jointIndexMap('A')).data(theta_iterator,:));
%             B = table2array(jointData(jointIndexMap('B')).data(theta_iterator,:));
%             angle = atan2(B(2) - A(2), B(1) - A(1));
%             % TODO: Do this process for all desired joint positions
%             adjustedAngles(rowNum, theta_iterator) = 180 - angle;
%         elseif theta_iterator == 2 % For joint B
%             A = table2array(jointData(jointIndexMap('A')).data(theta_iterator,:));
%             B = table2array(jointData(jointIndexMap('B')).data(theta_iterator,:));
%             angle = atan2(B(2) - A(2), B(1) - A(1));
%             %CHECK ON THIS PART
%             your_offset_AB = 180-angle;
%
%             adjustedAngles(theta_iterator) = theta + your_offset_AB;
%         elseif theta_iterator == 3 % For joint C
%             % Adjust angle based on your criteria for joint B
%             adjustedAngles(theta_iterator) = theta + your_offset_AB;
%         elseif theta_iterator == 4 % For joint D
%             % Adjust angle based on your criteria for joint B
%             adjustedAngles(theta_iterator) = theta + your_offset_AB;
%         elseif theta_iterator == 5 % For joint E
%             % Adjust angle based on your criteria for joint B
%             adjustedAngles(theta_iterator) = theta + your_offset_AB;
%         elseif theta_iterator == 6 % For joint F
%             % Default case if no specific offset criteria are met
%             adjustedAngles(theta_iterator) = theta; % No adjustment
%         end
%         % Add more conditions as needed for other joints
%     end
% end
% end
% function data =  readExperimentalCoolTermData(CoolTermExperimentalPath)
% % Directory and filenames setup
% dirPath = fullfile(CoolTermExperimentalPath); % Adjust base path as necessary
% filenames = {'10RPM.xlsx', '20RPM.xlsx', '30RPM.xlsx'};
%
% % Initialize an empty table for concatenated results
% concatenatedData = [];
%
% % Loop to Process Each File
% for i = 1:length(filenames)
%     fullPath = fullfile(dirPath, filenames{i});
%
%     % Check if the file exists before attempting to read
%     if isfile(fullPath)
%         % Detect import options based on the current file and sheet
%         opts = detectImportOptions(fullPath, 'NumHeaderLines', 3);
%         % Read the data into a table
%         tempData = readtable(fullPath, opts);
%
%         % Concatenate the new data table to the existing data
%         if isempty(concatenatedData)
%             concatenatedData = tempData;
%         else
%             concatenatedData = [concatenatedData; tempData]; % Assuming the data structure is the same across files
%         end
%     else
%         warning('File does not exist: %s', fullPath);
%     end
% end
%
%
% % Process concatenated data to find specific sensor values
% % Note: The following processing assumes 'concatenatedData' structure is consistent across files
%
% % Adjust column name as per your actual data structure
% hallSensorColumn = concatenatedData.Var3;
% oneIndices = find(hallSensorColumn == 1);
%
% if length(oneIndices) < 2
%     disp('Not enough data points where Hall sensor equals 1.');
%     return; % Exit if not enough data points
% end
%
% % Find the correct indices as per your logic
% secondOneIndex = oneIndices(2);
% nextOneIndexArray = oneIndices(oneIndices > secondOneIndex + 1);
%
% x = length(oneIndices);
% i=1;
% while (i < x)
%     indexDifference = oneIndices(i+1) - oneIndices(i);
%     if (indexDifference ==1)
%         oneIndices = [oneIndices(1:i); oneIndices(i+2:end)];
%     end
%     x = length(oneIndices);
%     i = i + 1;
% end
%
% if isempty(nextOneIndexArray)
%     disp('No subsequent non-consecutive 1 found after the second occurrence.');
%     return; % Exit if no subsequent non-consecutive 1 found
% else
%     nextOneIndex = nextOneIndexArray(1);
% end
%
% % Assuming 'OrientationX', 'OrientationY', 'OrientationZ', and 'GyroY' are column names
% % Extract and store required data from concatenatedData based on identified indices
%
% data.timestep = concatenatedData.Var2(secondOneIndex:nextOneIndex);
% data.MPUOrientationX = concatenatedData.Var5(secondOneIndex:nextOneIndex);
% data.MPUOrientationY = concatenatedData.Var6(secondOneIndex:nextOneIndex);
% data.MPUOrientationZ = concatenatedData.Var7(secondOneIndex:nextOneIndex);
% data.MPUGyroY = concatenatedData.Var4(secondOneIndex:nextOneIndex);
%
% data.BNOCouplerOrientationX = concatenatedData.Var8(secondOneIndex:nextOneIndex);
% data.BNOCouplerOrientationY = concatenatedData.Var9(secondOneIndex:nextOneIndex);
% data.BNOCouplerOrientationZ = concatenatedData.Var10(secondOneIndex:nextOneIndex);
% data.BNOCouplerGyroX = concatenatedData.Var11(secondOneIndex:nextOneIndex);
% data.BNOCouplerGyroY = concatenatedData.Var12(secondOneIndex:nextOneIndex);
% data.BNOCouplerGyroZ = concatenatedData.Var13(secondOneIndex:nextOneIndex);
%
% data.BNORockerOrientationX = concatenatedData.Var14(secondOneIndex:nextOneIndex);
% data.BNORockerOrientationY = concatenatedData.Var15(secondOneIndex:nextOneIndex);
% data.BNORockerOrientationZ = concatenatedData.Var16(secondOneIndex:nextOneIndex);
% data.BNORockerGyroX = concatenatedData.Var17(secondOneIndex:nextOneIndex);
% data.BNORockerGyroY = concatenatedData.Var18(secondOneIndex:nextOneIndex);
% data.BNORockerGyroZ = concatenatedData.Var19(secondOneIndex:nextOneIndex);
%
% end
%
% function WMdata = readExperimentalWitMotionData(WitMotionExperimentalPath)
% dirPath = fullfile(WitMotionExperimentalPath); % Adjust base path as necessary
% filenames = {'10RPM.csv', '20RPM.csv', '30RPM.csv'};
%
% % Initialize an empty table for concatenated results
% concatenatedData = [];
%
% % Loop to Process Each File
% for i = 1:length(filenames)
%     fullPath = fullfile(dirPath, filenames{i});
%
%     % Check if the file exists before attempting to read
%     if isfile(fullPath)
%         % Detect import options based on the current file and sheet
%         opts = detectImportOptions(fullPath, 'NumHeaderLines', 1);
%         % Read the data into a table
%         tempData = readtable(fullPath, opts);
%
%         % Concatenate the new data table to the existing data
%         if isempty(concatenatedData)
%             concatenatedData = tempData;
%         else
%             concatenatedData = [concatenatedData; tempData]; % Assuming the data structure is the same across files
%         end
%     else
%         warning('File does not exist: %s', fullPath);
%     end
% end
% deviceNames = concatenatedData.Var2;
% % Find unique strings and their first occurrence index
% [uniqueStrings, ia, ~] = unique(deviceNames, 'stable');
%
% % Find indices of each unique string in the original list
% indicesForEachUniqueString = cell(length(uniqueStrings), 1);
% for i = 1:length(uniqueStrings)
%     indicesForEachUniqueString{i} = find(ismember(deviceNames, uniqueStrings(i)));
% end
%
% table2 = indicesForEachUniqueString{2,1};
% table1 = indicesForEachUniqueString{1,1};
%
% a9concatenatedData = concatenatedData(table2, :);
% twelveconcatenatedData = concatenatedData(table1, :);
%
% for i  = 1:size(table2)
%     a9Index = table2(i);
%     a9data(i,:) = concatenatedData.Var11(a9Index);
%
% end
% %
% % for i = 1:size(table1)
% %     twelveIndex = table1(i);
% %     twelvedata(i, :) = concatenatedData(twelveIndex);
% % end
%
% % Once this is pulled, then we want to determine the time range. Time
% % range will be defined from the first instance that the YAngle goes
% % from negative to positive to the next instance.
% oneIndex = -1;
% secondOneIndex = -1;
% for i=1:length(a9data)
%     a9current = a9data(i);
%     a9next = a9data(i+1);
%
%     if a9current <0 && a9next > 0
%         if oneIndex == -1
%             oneIndex = i+1;
%         elseif secondOneIndex ==-1
%             secondOneIndex = i+1;
%             break
%         end
%     end
%
% end
% cat = 390482;
% % Afterward, we will cut the index values that are not needed. We know
% % what is not needed if the row values precede or exceed the time
% % range.
%
% % Lastly, We store the desired angle and angular velocity of the sensor
% %
% % Assuming 'OrientationX', 'OrientationY', 'OrientationZ', and 'GyroY' are column names
% % Extract and store required data from concatenatedData based on identified indices
%
% WMdata.WMtimestep = a9concatenatedData.Var1(oneIndex:secondOneIndex);
%
% WMdata.InputXAngle = a9concatenatedData.Var10(oneIndex:secondOneIndex);
% WMdata.InputYAngle = a9concatenatedData.Var11(oneIndex:secondOneIndex);
% WMdata.InputZAngle = a9concatenatedData.Var12(oneIndex:secondOneIndex);
%
% WMdata.InputXVel = a9concatenatedData.Var7(oneIndex:secondOneIndex);
% WMdata.InputYVel = a9concatenatedData.Var8(oneIndex:secondOneIndex);
% WMdata.InputZVel = a9concatenatedData.Var9(oneIndex:secondOneIndex);
%
% WMdata.OutputXAngle = twelveconcatenatedData.Var10(oneIndex:secondOneIndex);
% WMdata.OutputYAngle = twelveconcatenatedData.Var11(oneIndex:secondOneIndex);
% WMdata.OutputZAngle = twelveconcatenatedData.Var12(oneIndex:secondOneIndex);
%
% WMdata.OutputXVel = twelveconcatenatedData.Var7(oneIndex:secondOneIndex);
% WMdata.OutputYVel = twelveconcatenatedData.Var8(oneIndex:secondOneIndex);
% WMdata.OutputZVel = twelveconcatenatedData.Var9(oneIndex:secondOneIndex);
%
%
%
% end
% Placeholder for processing; adapt based on your actual needs
% function compareAngVel(expData, theoData)
% % Example: Compare angular velocity for a specific joint and link
% % This is highly dependent on how your data is structured and needs to be adjusted
% % Assume expData and theoData are structured to facilitate direct comparison
%
% jointName = 'E'; % Example
% linkName = 'ABEH'; % The link this joint is part of
% speed = 'speed1'; % Example speed variant
%
% expAngVel = expData.(jointName).AngVel; % Placeholder: Adjust to your structure
% theoAngVel = theoData.(linkName).(speed).AngVel; % Placeholder: Adjust to your structure
%
% % Interpolation and RMSE calculation would go here
% % This is a conceptual outline; specifics depend on your data's organization and needs
% end
% function [itemName, speedTag] = parseFileName(fileName)
%     % Parse filenames that follow the pattern 'itemName_speedTag.csv'
%     tokens = regexp(fileName, '^(.+?)_(speed\d+)\.csv$', 'tokens');
%     if isempty(tokens)
%         itemName = '';
%         speedTag = '';
%     else
%         itemName = tokens{1}{1};
%         speedTag = tokens{1}{2};
%     end
% end
% function dataStruct = readDataFromDirectory(path, dataStruct, category, subCategory, nestedDir)
%     csvFiles = dir(fullfile(path, '*.csv'));
%     for file = csvFiles'
%         % Extract information from the filename; handle optional speed tag
%         % tokens = regexp(file.name, '^(.+?)(?:_(speed\d+))?\.csv$', 'tokens');
%         % tokens = regexp(file.name, '^(.+?)(?:_speed(\d+))?\.csv$', 'tokens');
%         tokens = regexp(file.name, '^(.+?)_speed(\d+)\.csv$', 'tokens');
%         if isempty(tokens) || isempty(tokens{1})
%             continue; % Skip if filename doesn't match expected format
%         end
%
%         itemName = tokens{1}{1}; % Item name is always the first token, directly accessed
%         speedTag = ''; % Default speed tag is empty
%         if size(tokens{1}, 2) > 1 % Check if there is a speed tag
%             speedTag = tokens{1}{2}; % Speed tag is the second token if it exists
%         end
%
%         % Read the CSV file into a table
%         dataTable = readtable(fullfile(file.folder, file.name));
%
%         % Update the data structure
%         dataStruct = updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable);
%     end
% end
% function jointData = readTheoreticalJointData(basePath)
% speeds = {'speed1', 'speed2', 'speed3'}; % Define the different speeds
% jointData = struct(); % Initialize an empty struct for storing data
%
% files = dir(fullfile(basePath, '*.csv')); % List all CSV files in the directory
% for i = 1:length(files)
%     fileName = files(i).name;
%     filePath = fullfile(files(i).folder, fileName);
%
%     Extract the joint name and speed from the file name
%     [jointName, speedTag] = strtok(fileName, '_');
%     speedTag = erase(speedTag, ['.csv', '_']); % Remove extra characters to isolate the speed tag
%
%     Check if the speed tag is one of the defined speeds
%     if any(strcmp(speeds, speedTag))
%         Read CSV file into table
%         data = readtable(filePath);
%
%         Store data in struct using dynamic field names for joint and speed
%         if isfield(jointData, jointName)
%             jointData.(jointName).(speedTag) = data;
%         else
%             jointData.(jointName) = struct(speedTag, data);
%         end
%     end
% end
% end
