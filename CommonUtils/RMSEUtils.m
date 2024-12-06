classdef RMSEUtils
    methods(Static)
        function Mechanism = RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, processCoolTermData, processPythonGraphData, processWitMotionData, determineAdjustment, determineOffset)
            TheoreticalPath = 'CSVOutput';

            % Define the base paths for experimental data
            ExperimentalPath = 'Experimental';

            % Define speeds (e.g. {'f10RPM', 'f20RPM', 'f30RPM'})
            speeds = formatSpeeds(Mechanism.input_speed_str);

            % Raw data
            expData = RMSEUtils.readExperimentalData(ExperimentalPath, sensorSourceMap, speeds);
            theoData = RMSEUtils.readTheoreticalData(TheoreticalPath);

            % Calculate RMSE for all sensors according to specified data types in the map
            rmseResults = struct();  % Initialize an empty struct to hold all results

            for sensor = keys(sensorDataTypes)
                currentSensor = sensor{1};
                dataTypes = sensorDataTypes(currentSensor);  % Retrieve data types for current sensor
                % Compute RMSE for the current sensor across its specified data types
                rmseResults.(currentSensor) = RMSEUtils.calculateRMSEForSensor(expData, theoData, currentSensor, sensorSourceMap, dataTypes, speeds, processCoolTermData, processPythonGraphData, processWitMotionData, determineAdjustment, determineOffset);
            end

            % Save results to CSV
            resultsFilename = 'RMSE_Results';
            % Verify this logic later and make sure this function works
            RMSEUtils.saveResultsToCSV(rmseResults, resultsFilename);
        end

        % Function to calculate RMSE for a given sensor and its data types
        function results = calculateRMSEForSensor(expData, theoData, sensor, sensorSourceMap, dataTypes, speeds, processCoolTermData, processPythonGraphData, processWitMotionData, determineAdjustment, determineOffset)
            results = struct();
            for dataType = dataTypes
                for speed = speeds
                    % Calculate RMSE using a hypothetical function, for a given dataType and speed
                    rmseValue = RMSEUtils.calculateRMSE(expData, theoData, sensor, sensorSourceMap, dataType{1}, speed{1}, processCoolTermData, processPythonGraphData, processWitMotionData, determineAdjustment, determineOffset);
                    % Store RMSE value in the struct under its corresponding speed
                    results.(dataType{1}).(speed{1}) = rmseValue;
                end
            end
        end

        function rmseResults = calculateRMSE(expDataSet, theoDataSet, sensor, sensorSourceMap, dataType, speed, processCoolTermData, processPythonGraphData, processWitMotionData, determineAdjustment, determineOffset)
            % rmseResults = struct(); % Initialize results structure

            % Retrieve experimental and theoretical data for the given sensor, dataType, and speed
            expData = RMSEUtils.retrieveExpData(expDataSet, sensor, sensorSourceMap, dataType, speed, processCoolTermData, processPythonGraphData, processWitMotionData);
            theoData = RMSEUtils.retrieveTheoData(theoDataSet, expData, sensor, dataType, speed, determineAdjustment, determineOffset);

            % Calculate RMSE if both experimental and theoretical data are available
            if ~isempty(expData) && ~isempty(theoData)

                % Calculate RMSE if both experimental and theoretical data are available
                timestamps = expData.Time;

                interpolatedTheoData = interp1(theoData.Time, theoData.Values, timestamps, 'linear', 'extrap');
                rmse = sqrt(mean((expData.Values - interpolatedTheoData).^2));

                % Store RMSE in the results structure
                rmseResults = rmse;

                % Generate plot for verification
                % Plot the data with thicker lines
                fig = figure('Visible', 'off');
                
                % Plot data with thicker lines
                plot(timestamps, expData.Values, 'b', 'LineWidth', 2, 'DisplayName', 'Experimental Data');
                hold on;
                plot(theoData.Time, theoData.Values, 'g', 'LineWidth', 2, 'DisplayName', 'Theoretical Data');
                plot(timestamps, interpolatedTheoData, 'r--', 'LineWidth', 2, 'DisplayName', 'Interpolated Theoretical Data');
                
                % Add legend and labels
                legend('FontSize', 8, 'Location', 'northeast'); % Top-right legend placement
                xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); % Bold x-axis label
                if strcmp(dataType, 'Angle')
                    ylabel('Degrees', 'FontSize', 16, 'FontWeight', 'bold'); % Bold y-axis label
                elseif strcmp(dataType, 'AngVel')
                    ylabel('Rad/s', 'FontSize', 16, 'FontWeight', 'bold'); % Bold y-axis label
                else
                    ylabel('??', 'FontSize', 16, 'FontWeight', 'bold'); % Bold y-axis label
                end
                
                % Add title with larger font size
                titleSpeed = parseSpeed(speed);
                title(['RMSE Analysis for ' sensor ' - ' dataType ' - ' titleSpeed], 'FontSize', 16, 'FontWeight', 'bold'); % Bold title
                
                % Adjust axis tick labels for bold font
                set(gca, 'FontSize', 12, 'FontWeight', 'bold'); % Bold axis and tick labels
                
                hold off;


%                 fig = figure('Visible', 'off');
%                 hold on;
%                 plot(timestamps, expData.Values, 'b', 'DisplayName', 'Experimental Data');
%                 plot(theoData.Time, theoData.Values, 'g', 'DisplayName', 'Theoretical Data');
%                 plot(timestamps, interpolatedTheoData, 'r--', 'DisplayName', 'Interpolated Theoretical Data');
%                 legend show;
%                 xlabel('Time (s)');
%                 if strcmp(dataType, 'Angle')
%                     ylabel('Degrees');
%                 elseif strcmp(dataType, 'AngVel')
%                     ylabel('Rad/s');
%                 else
%                     ylabel('??');
%                 end
% 
%                 titleSpeed = parseSpeed(speed);
%                 title(['RMSE Analysis for ' sensor ' - ' dataType ' - ' titleSpeed]);
%                 hold off;

                % Define directory path for saving
                resultDir = fullfile('RMSE_Results', sensor, dataType, speed);

                % Ensure the directory exists
                if ~exist(resultDir, 'dir')
                    mkdir(resultDir);
                end

                % Save plot
                savefig(fig, fullfile(resultDir, 'graph.fig'));
                saveas(fig, fullfile(resultDir, 'graph.png'));

                % Close the figure
                close(fig);
            else
                warning('Missing data for sensor %s, data type %s, speed %s', sensor, dataType, speed);
                rmseResults = NaN; % Assign NaN to indicate missing data calculation
            end

            return;
        end


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
                    if strcmp(subCategory{1}, 'LinAcc') || strcmp(subCategory{1}, 'LinVel') || strcmp(subCategory{1}, 'Point')  || strcmp(subCategory{1}, 'Angle')
                        dataStruct = RMSEUtils.processNestedDirectories(subCategoryPath, dataStruct, category, subCategory{1});
                    else
                        dataStruct = RMSEUtils.processSpeedDirectories(subCategoryPath, dataStruct, category, subCategory{1}, '');
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
                    dataStruct = RMSEUtils.processSpeedDirectories(nestedPath, dataStruct, category, subCategory, subDir{1});
                end
            end
        end

        function dataStruct = processSpeedDirectories(path, dataStruct, category, subCategory, nestedDir)
            % Process directories that include different speeds or default data
            speeds = dir(path);
            for speed = speeds'
                if speed.isdir && ~ismember(speed.name, {'.', '..'})
                    speedPath = fullfile(path, speed.name);
                    dataStruct = RMSEUtils.readDataFromDirectory(speedPath, dataStruct, category, subCategory, nestedDir);
                else
                    dataStruct = RMSEUtils.readDataFromDirectory(path, dataStruct, category, subCategory, nestedDir);
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
                speedTag = RMSEUtils.getSpeedTagFromPath(path);

                dataTable = readtable(fullfile(file.folder, file.name));
                dataStruct = RMSEUtils.updateDataStruct(dataStruct, category, subCategory, nestedDir, itemName, speedTag, dataTable);
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

        function expData = readExperimentalData(baseExperimentalPath, sensorSourceMap, speeds)
            expData = struct(); % Initialize
            subFolders = mapValuesToUniqueArray(sensorSourceMap);  % Subdirectories to iterate through
            filenames = speeds; % RPM filenames

            for i = 1:length(subFolders)
                % Initialize sub-structures for each subfolder
                expData.(subFolders{i}) = struct();
                currentPath = fullfile(baseExperimentalPath, subFolders{i}); % Path to current subdirectory

                for j = 1:length(filenames)
                    safeFieldName = filenames{j};

                    % Construct file path
                    if strcmp(subFolders{i}, 'CoolTerm') % For 'CoolTerm', read XLSX files
                        xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
                        % Check and read XLSX file
                        if isfile(xlsxPath)
                            expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
                            % expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath, 'Range', 'A1'); % Adjust 'Range' if necessary
                        end
                    elseif strcmp(subFolders{i}, 'WitMotion') % For 'WitMotion', read CSV files
                        csvPath = fullfile(currentPath, filenames{j} + ".csv");
                        % Check and read CSV file, including headers
                        if isfile(csvPath)
                            opts = detectImportOptions(csvPath);
                            opts.Delimiter = ',';  % Set the delimiter

                            % Ensure the variable names (headers) are preserved as they are in the file
                            opts.PreserveVariableNames = true;

                            % Specify that the first row contains the headers
                            opts.VariableNamesLine = 1;  % This tells MATLAB that the first line contains variable names (headers)

                            % Ensure data starts reading from the line after the headers
                            opts.DataLine = 2;  % Start reading data from the second line, assuming the first line is the header

                            % Read the table using the specified options
                            expData.(subFolders{i}).(safeFieldName) = readtable(csvPath, opts);
                        end
                    elseif strcmp(subFolders{i}, 'PythonGraph') % For 'PythonGraph', read XLSX files
                        xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
                        % Check and read XLSX file
                        if isfile(xlsxPath)
                            expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
                            % If needed, you can specify 'Range' and other options in 'readtable'
                        end
                    else
                        % Handle other cases or give a warning/error
                        warning('Unknown subfolder type: %s', subFolders{i});
                    end
                end
            end
        end

        % Retriev the desired experimental data
        function expData = retrieveExpData(dataSet, sensor, sensorSourceMap, dataType, speed, processCoolTermData, processPythonGraphData, processWitMotionData)
            % Map sensors to their respective data sources (CoolTerm or WitMotion)
            source = sensorSourceMap(sensor);
            % Check if the required data is available
            if isfield(dataSet, source) && isfield(dataSet.(source), speed)
                rawData = dataSet.(source).(speed); % nxm table of data
                % expData = processData(rawData, sensor, dataType);
                if (strcmp(source, 'CoolTerm'))
                    expData = feval(processCoolTermData, rawData, sensor, dataType);
                elseif (strcmp(source, 'WitMotion'))
                    expData = feval(processWitMotionData, rawData, sensor, dataType);
                elseif (strcmp(source, 'PythonGraph'))
                    expData = feval(processPythonGraphData, rawData, sensor, dataType);
                else
                    warming('Application utilized to analyze data is unknown');
                    expData = [];
                end
            else
                expData = []; % Return empty if not found
            end
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

        function cols = getDataColumns(dataType)
            % Define data columns for different data types
            switch dataType
                case 'LinAcc'
                    cols = 4:6; % Columns for acceleration data
                case 'AngVel'
                    cols = 7:9; % Columns for angular velocity data
                case 'Angle'
                    cols = 10:12; % Columns for angle data
                otherwise
                    cols = [];
            end
        end

        function theoData = retrieveTheoData(dataSet, expData, sensor, dataType, speed, determineAdjustment, determineOffset)
            % Determine the main category based on dataType
            switch dataType
                case {'LinVel', 'AngVel'}
                    mainCategory = 'Vel';
                case {'LinAcc', 'AngAcc'}
                    mainCategory = 'Acc';
                case {'Angle', 'Point'}
                    mainCategory = 'Pos';
                otherwise
                    mainCategory = '?';
            end

            % Determine the sub-category (Joint or LinkCoM or directly under the category)
            if any(strcmp(dataType, {'LinVel', 'LinAcc', 'Point', 'Angle'}))
                if length(sensor) == 1  % Assuming sensor names for Joints are single characters
                    subCategory = 'Joint';
                else
                    subCategory = 'LinkCoM';
                end
            else
                subCategory = '';  % For angular data types or position, the sensor directly maps to data
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

                % Initialize theoDataArray
                theoDataArray = [];

                % Dynamically find the appropriate sensor field that contains the sensor ID
                if ~isempty(dataField)
                    sensorFields = fieldnames(dataField);
                    for i = 1:length(sensorFields)
                        if contains(sensorFields{i}, sensor)
                            % if strcmp(sensorFields{i}, sensor)
                            % Handle cases with and without speed specification
                            if ~isempty(speed) && isfield(dataField.(sensorFields{i}), speed)
                                theoDataArray = table2array(dataField.(sensorFields{i}).(speed)(:,3));
                                % Calculate Time Before Adjustment
                                rpmValue = str2double(strrep(regexp(speed, '\d+_\d+|\d+', 'match'), '_', '.'));
                                timePerRevolution = 60 / rpmValue;  % Time for one full revolution (in seconds)
                                numDataPoints = size(theoDataArray, 1);  % Number of data points in the theoretical data
                                theoreticalTime = linspace(0, timePerRevolution, numDataPoints).';  % Linearly spaced time array
                                theoData.Time = theoreticalTime;  % Set the time for theoData
                            else
                                theoDataArray = double(dataField.(sensorFields{i}){:, 3});

                                % Calculate Time Before Adjustment
                                rpmValue = str2double(strrep(regexp(speed, '\d+_\d+|\d+', 'match'), '_', '.'));
                                timePerRevolution = 60 / rpmValue;  % Time for one full revolution (in seconds)
                                numDataPoints = size(theoDataArray, 1);  % Number of data points in the theoretical data
                                theoreticalTime = linspace(0, timePerRevolution, numDataPoints).';  % Linearly spaced time array
                                theoData.Time = theoreticalTime;  % Set the time for theoData

                                % Perform Interpolation and Adjustments After Time Calculation
                                expTimeStart = expData.Time(1);  % Get the first timestep from expData
                                interpolatedTheoData = interp1(theoData.Time, theoDataArray, expTimeStart, 'linear');  % Interpolate to match first expData timestep

                                % Utilize the passed in adjustment function and make the adjustment accordingly
                                adjustment = feval(determineAdjustment, sensor, interpolatedTheoData, expData.Values(1));

                                % Pass the adjusted value into offset function
                                theoDataArray = feval(determineOffset, sensor, theoDataArray, adjustment);

                                % adjustment = expData.Values(1) - interpolatedTheoData;  % Calculate adjustment
                                % theoDataArray = theoDataArray + adjustment;  % Apply adjustment

                                % Additional Adjustments for Specific Data Types or Sensors
                                if strcmp(dataType, 'Angle')
                                    if strcmp(sensor, 'H') || strcmp(sensor, 'I')
                                        theoDataArray = adjustAngleRange(theoDataArray);
                                    end
                                end
                            end
                        end
                    end
                end

                if isempty(theoDataArray)  % If no matching sensor field is found
                    theoDataArray = [];  % Return empty if not found
                end
            catch
                theoDataArray = [];  % Return empty if any field is not found or any error occurs
            end

            % Set the adjusted values back to theoData
            theoData.Values = theoDataArray;
        end


        % Function to save RMSE results to CSV
        function saveResultsToCSV(rmseResults, baseFolder)
            % Ensure the base RMSE folder exists
            if ~exist(baseFolder, 'dir')
                mkdir(baseFolder);
            end

            % Iterate over all sensors
            for sensor = fieldnames(rmseResults)'
                sensorPath = fullfile(baseFolder, sensor{1});
                if ~exist(sensorPath, 'dir')
                    mkdir(sensorPath);
                end

                % Iterate over all data types
                for dataType = fieldnames(rmseResults.(sensor{1}))'
                    dataTypePath = fullfile(sensorPath, dataType{1});
                    if ~exist(dataTypePath, 'dir')
                        mkdir(dataTypePath);
                    end

                    % Iterate over all speeds
                    for speed = fieldnames(rmseResults.(sensor{1}).(dataType{1}))'
                        speedPath = fullfile(dataTypePath, speed{1});
                        if ~exist(speedPath, 'dir')
                            mkdir(speedPath);
                        end

                        % Retrieve the RMSE value
                        rmseValue = rmseResults.(sensor{1}).(dataType{1}).(speed{1});

                        % Define the filename for the Excel file
                        filename = fullfile(speedPath, 'RMSE.xlsx');

                        % Write RMSE value to an Excel file
                        xlswrite(filename, rmseValue, 'Sheet1', 'A1');
                    end
                end
            end
        end
    end
end

function speeds = formatSpeeds(input_speed_str)
% Initialize the cell array to store formatted speeds
speeds = cell(1, length(input_speed_str));

% Loop through each speed and format it
for i = 1:length(input_speed_str)
    % Format the string with 'f' at the start and 'RPM' at the end
    % speeds{i} = ['f' num2str(input_speed_str(i)) 'RPM'];
    speedStrTemp = strrep(num2str(input_speed_str(i)), '.', '_');  % Replace '.' with '_'
    speeds{i} = ['f' speedStrTemp 'RPM'];  % Construct the new name

end
end

function subFolders = mapValuesToUniqueArray(sensorSourceMap)
% Get all values from the map as a cell array
allValues = values(sensorSourceMap);

% Ensure all values are in a cell array of strings
if iscell(allValues{1})
    % Flatten the cell array in case it's nested
    allValues = [allValues{:}];
else
    % If the single value is not in a cell, wrap it
    allValues = allValues;
end

% Find unique values to avoid duplicates
uniqueValues = unique(allValues, 'stable');  % 'stable' keeps the original order

% Return the unique values as a cell array
subFolders = uniqueValues;
end

% Helper function to adjust angle data
function adjustedData = adjustAngleRange(data)
% Convert negative values to their positive complements
data = mod(data, 360);  % Ensure all values are in the range [0, 360)
adjustedData = zeros(size(data));  % Initialize the adjusted data array

for i = 1:length(data)
    if data(i) <= 90
        adjustedData(i) = data(i);
    elseif data(i) > 90 && data(i) <= 180
        adjustedData(i) = 180 - data(i);
    elseif data(i) > 180 && data(i) <= 270
        adjustedData(i) = -(data(i) - 180);
    else
        adjustedData(i) = -(360 - data(i));
    end
end
end

function speed = parseSpeed(formatted_speed_str)
% Remove the 'f' at the start and 'RPM' at the end
tempStr = erase(formatted_speed_str, {'f', 'RPM'});

% Replace underscores with dots
tempStr = strrep(tempStr, '_', '.');

% Add " RPM" to the numeric part
speed = [tempStr ' RPM'];
end




