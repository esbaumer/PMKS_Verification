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
    {'AngVel'}, ...  % Data types for sensor E
    {'AngVel'}, ... % Data types for sensor F
    {'AngVel'}, ... % Data types for sensor G
    {'AngVel', 'LinAcc'}, ...  % Data types for sensor H
    {'AngVel', 'LinVel', 'AngAcc'}  ... % Data types for sensor I
    % {'Angle', 'LinVel'}, ...  % Data types for sensor E
    % {'AngVel', 'LinAcc'}, ... % Data types for sensor F
    % {'AngAcc', 'LinVel'}, ... % Data types for sensor G
    % {'Angle', 'AngVel', 'LinAcc'}, ...  % Data types for sensor H
    % {'AngVel', 'LinVel', 'AngAcc'}  ... % Data types for sensor I
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

function dataStruct = readTheoreticalData(basePath)
% Define mappings of categories to their relevant subcategories
categoryMap = containers.Map({'Acc', 'Vel', 'Pos'}, ...
    {{'AngAcc', 'LinAcc'}, {'AngVel', 'LinVel'}, {'Angle', 'Point'}});  % No subcategories for Pos

dataStruct = struct(); % Initialize the main data structure

% Iterate over each category like Acc, Vel, Pos
for k = categoryMap.keys
    category = k{1};
    subCategories = categoryMap(category);
    categoryPath = fullfile(basePath, category);

    % Process each subcategory appropriately

    for subCategory = subCategories
        subCategoryPath = fullfile(categoryPath, subCategory{1});
        if strcmp(subCategory{1}, 'LinAcc') || strcmp(subCategory{1}, 'LinVel') || strcmp(subCategory{1}, 'Point')
            dataStruct = processNestedDirectories(subCategoryPath, dataStruct, category, subCategory{1});
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

% if strcmp(category, 'Pos') % Pos, which is a special case
%     finalTarget = dataStruct.(category);
%     if ~isempty(nestedDir)
%         if ~isfield(finalTarget, nestedDir)
%             finalTarget.(nestedDir) = struct();
%         end
%         finalTarget = finalTarget.(nestedDir);
%     end
% 
%     if ~isfield(finalTarget, itemName)
%         finalTarget.(itemName) = struct();
%     end
% 
%     finalTarget.(itemName) = dataTable;
% 
%     dataStruct.(category).(nestedDir) = finalTarget;
%     return;
% end

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
    finalTarget.(itemName) = dataTable;
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
function expData = retrieveExpData(dataSet, theoDataSet, sensor, dataType, speed)
% Map sensors to their respective data sources (CoolTerm or WitMotion)
sensorSourceMap = containers.Map({'E', 'F', 'G', 'H', 'I'}, ...
    {'CoolTerm', 'CoolTerm', 'CoolTerm', 'WitMotion', 'WitMotion'});
source = sensorSourceMap(sensor);

% Check if the required data is available
if isfield(dataSet, source) && isfield(dataSet.(source), speed)
    rawData = dataSet.(source).(speed); % nxm table of data
    % expData = processData(rawData, sensor, dataType);
    if (strcmp(source, 'CoolTerm'))
        expData = processCoolTermData(rawData, theoDataSet, sensor, dataType);
    else
        expData = processWitMotionData(rawData, theoDataSet, sensor, dataType);
    end
else
    expData = []; % Return empty if not found
end
end
% function processedData = processData(rawData, sensor, dataType)
% % Initialize processed data structure
% processedData = struct('Time', [], 'Values', []);
%
% % Define sensor-specific columns and process data accordingly
% sensorDataMap = containers.Map(...
%     {'E', 'F', 'G', 'H', 'I'}, ...
%     {8:13, 4:7, 14:19, 1:3, 4:6} ...  % Example column indices for each sensor
%     );
% columnIndices = sensorDataMap(sensor);
%
% % Extract data based on dataType
% switch dataType
%     % TODO: don't determine the mean, just pull the values based
%     % from respective time domain. Most likely do a comparison
%     % between the theoretical values to the experimental values and
%     % get the values that most closely match the thoeretical values
%     case 'Angle'
%         processedData.Values = mean(rawData(:, columnIndices(1:3)), 2); % Example: mean of first three columns
%     case 'LinVel'
%         processedData.Values = rawData(:, columnIndices(4)); % Example: fourth column
%     case 'AngVel'
%         % Further processing can be added here
% end
%
% % Example time column extraction
% processedData.Time = rawData(:, 2); % Assuming second column is time
%
% % Filter data to remove outliers or bad data
% processedData = filterData(processedData);
% end


% function filteredData = filterData(data)
% % Example filtering operation
% filteredData = data;
% % goodIndices = data.Values < threshold; % Define 'threshold' based on your criteria
% % filteredData = struct('Time', data.Time(goodIndices), 'Values', data.Values(goodIndices));
% end

% function processedData = processData(rawData, sensor, dataType)
%     % Initialize processed data structure
%     processedData = struct('Time', [], 'Values', []);
%
%     % Define sensor-specific columns and process data accordingly
%     sensorDataMap = containers.Map(...
%         {'E', 'F', 'G', 'H', 'I'}, ...
%         {8:13, 4:7, 14:19, 1:3, 4:6} ...  % Example column indices for each sensor
%         );
%     columnIndices = sensorDataMap(sensor);
%
%     % Extract data based on dataType
%     switch dataType
%         case 'Angle'
%             processedData.Values = rawData(:, columnIndices(1:3)); % Use all angle columns
%         case 'LinVel'
%             processedData.Values = rawData(:, columnIndices(4)); % Use specific LinVel column
%         case 'AngVel'
%             processedData.Values = rawData(:, columnIndices(5:end)); % Use all AngVel columns
%     end
%
%     % Extract time column - example uses the 1st column for time
%     processedData.Time = rawData(:, 1);
%
%     % Filter data to remove outliers or bad data
%     processedData = filterData(processedData);
%
%     return;
% end

% function filteredData = filterData(data)
%     % Placeholder for data filtering logic
%     % Example: Remove outliers using median and interquartile range
%     validIdx = (data.Values > (median(data.Values) - 1.5*iqr(data.Values))) & ...
%                (data.Values < (median(data.Values) + 1.5*iqr(data.Values)));
%     filteredData.Values = data.Values(validIdx);
%     filteredData.Time = data.Time(validIdx);
% end

% For CoolTerm Data Processing
function coolTermData = processCoolTermData(rawData, theoData, sensorID, dataType)
% Define sensor columns for angles and angular velocities
sensorColumnsMap = containers.Map(...
    {'F', 'E', 'G'}, ...
    {struct('Angle', [5:7], 'AngVel', 4), ...
    struct('Angle', [8:10], 'AngVel', [11:13]), ...
    struct('Angle', [14:16], 'AngVel', [17:19])});
columns = sensorColumnsMap(sensorID).(dataType);

binarySignal = rawData.Var3;  % Adjust 'Var3' to the correct variable name if different

% Identify valid data segments based on binary signals
oneIndices = find(binarySignal == 1);
validSegments = find(diff(oneIndices) > 1);  % Find non-consecutive ones

if isempty(validSegments) || length(validSegments) < 2
    error('Valid data segments not found.');
end

if isempty(validSegments) || length(validSegments) < 2
    error('Valid data segments not found.');
end

% Define the range for valid data based on identified segments
validStartIndex = oneIndices(validSegments(1));
validEndIndex = oneIndices(validSegments(2));

% Extract the valid data range
validData = rawData(validStartIndex:validEndIndex, :);

% Compare extracted data with theoretical data for further refinement
% comparisonResults = compareData(validData(:, columns), theoData);
% refinedDataIndices = find(comparisonResults);  % Rows closely matching theoretical data
% TODO: Get the column that closely match the theoretical data
% refinedDataIndices = validData(:,columns(1));

YData = validData(:,columns(1));
XData = validData(:, 2);
% Refine data by ensuring continuity and removing spikes
% refinedData = validData(refinedDataIndices, :);
% continuousData = removeSpikes(refinedData, columns);

% Store processed data for output
coolTermData.Time = XData;  % Time column
coolTermData.Values = YData;  % Extracted values based on dataType and sensor
% coolTermData.Time = continuousData(:, 2);  % Time column
% coolTermData.Values = continuousData(:, columns);  % Extracted values based on dataType and sensor
end

function isClose = compareData(experimental, theoretical)
% Define a simple threshold-based comparison for data matching
isClose = sum(abs(experimental - theoretical), 2) < someThreshold;  % Adjust threshold as needed
end

function cleanData = removeSpikes(data, columns)
% Remove spikes using median and median absolute deviation (MAD)
for col = columns
    medianVal = median(data(:, col));
    madVal = mad(data(:, col), 1);
    spikeIndices = abs(data(:, col) - medianVal) > 3 * madVal;
    data(spikeIndices, col) = NaN;  % Replace spikes with NaNs
end
cleanData = data(all(~isnan(data), 2), :);  % Discard any rows with NaNs
end


% For WitMotion Data Processing
function witMotionData = processWitMotionData(rawData, theoDataSet, sensorID, dataType)
% Determine the index for zero crossing in Angle Y
zeroCrossings = find(diff(sign(rawData(:, 11))) > 0) + 1;  % Assumes Angle Y is in the 11th column
if length(zeroCrossings) < 2
    error('Not enough zero crossings found.');
end

validStartIndex = zeroCrossings(1);
validEndIndex = zeroCrossings(2);

% Extract the valid data slice
witMotionData = rawData(validStartIndex:validEndIndex, :);
return;
end


% Retriev the desired theoretical data
function theoData = retrieveTheoData(dataSet, sensor, dataType, speed)
% Determine the main category based on dataType
switch dataType
    case {'LinVel', 'AngVel'}
        mainCategory = 'Vel';
    case {'LinAcc', 'AngAcc'}
        mainCategory = 'Acc';
    otherwise
        mainCategory = 'Pos';
end

% Determine the sub-category (Joint or LinkCoM or directly under the category)
if any(strcmp(dataType, {'LinVel', 'LinAcc'}))  % These involve Joint or LinkCoM
    if length(sensor) == 1  % Assuming sensor names for Joints are single characters
        subCategory = 'Joint';
    else
        subCategory = 'LinkCoM';
    end
else  % For angular data types or position, the sensor directly maps to data
    subCategory = '';
end

% Access the appropriate dataset
try
    if isempty(subCategory)
        % Directly under main category for angular data types
        dataField = dataSet.(mainCategory).(dataType);
    else
        % Nested under Joint or LinkCoM
        dataField = dataSet.(mainCategory).(dataType).(subCategory);
    end

    % Dynamically find the appropriate sensor field that contains the sensor ID
    theoData = [];
    if ~isempty(dataField)
        sensorFields = fieldnames(dataField);
        for i = 1:length(sensorFields)
            if contains(sensorFields{i}, sensor)
                if isfield(dataField.(sensorFields{i}), speed)
                    theoData = dataField.(sensorFields{i}).(speed);
                    break;
                end
            end
        end
    end
    if isempty(theoData)  % If no matching sensor field is found
        theoData = [];  % Return empty if not found
    end
catch
    theoData = [];  % Return empty if any field is not found or any error occurs
end
end


function rmseResults = calculateRMSE(expDataSet, theoDataSet, sensor, dataType, speed)
rmseResults = struct(); % Initialize results structure

% Retrieve experimental and theoretical data for the given sensor, dataType, and speed
theoData = retrieveTheoData(theoDataSet, sensor, dataType, speed);
expData = retrieveExpData(expDataSet, theoData, sensor, dataType, speed);

% Calculate RMSE if both experimental and theoretical data are available
if ~isempty(expData) && ~isempty(theoData)
    rpmValue = str2double(regexp(speed, '\d+', 'match'));  % Extract numerical part from speed string like 'f10RPM'
    timePerRevolution = 60 / rpmValue;  % Calculate the time for one full revolution (in seconds)
    numDataPoints = size(theoData, 1);  % Number of data points in the theoretical data
    theoreticalTime = linspace(0, timePerRevolution, numDataPoints);  % Create a linearly spaced time array

    % Calculate RMSE if both experimental and theoretical data are available
    timestampsRaw = expData.Time;
    timestamps = timestampsRaw - timestampsRaw(1,1);
    timestamps = table2array(timestamps) / 1000;

    interpolatedTheoData = interp1(theoreticalTime, table2array(theoData(:,3)), timestamps, 'linear', 'extrap');
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
