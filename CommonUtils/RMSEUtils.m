classdef RMSEUtils
    methods(Static)
        function Mechanism = RMSESolver(Mechanism, fileToSpeedMap, sensorDataTypes, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, calculateRMSE, determineAdjustment, determineOffset, determineMap)
            % Main solver function to compute RMSE for all sensors and data types
            % Args:
            % - Mechanism: Struct to store the results
            % - sensorDataTypes: Map of sensors and their corresponding data types
            % - determineAdjustment, determineOffset, determineMap: Function handles

            % Define the base paths for data
            TheoreticalPath = 'CSVOutput';
            ExperimentalPath = 'Experimental';
            
            % Define speeds
%             speeds = formatSpeeds(Mechanism.input_speed_str);
           % Get the list of speeds from Mechanism
            speeds = Mechanism.input_speed_str;
            
            % Initialize an array to store file names
%             fileNames = cell(size(speeds));
%             
%             % Extract file names corresponding to each speed
%             for i = 1:length(speeds)
%                 speedKey = speeds(i); % Current speed as a key
%                 if isKey(fileToSpeedMap, speedKey)
%                     fileNames{i} = fileToSpeedMap(speedKey); % Retrieve the corresponding file name
%                 else
%                     error('Speed %s not found in fileToSpeedMap.', speedKey);
%                 end
%             end

            % Read experimental and theoretical data
            expData = RMSEUtils.readExperimentalData(ExperimentalPath, sensorSourceMap);
            theoData = RMSEUtils.readTheoreticalData(TheoreticalPath);

            % Initialize results
            rmseResults = struct();

            % Define the path to the WitMotion directory
            witMotionPath = fullfile(ExperimentalPath, 'WitMotion');
            
            % Get all files in the WitMotion directory
            fileInfo = dir(fullfile(witMotionPath, '*')); % Get all files and folders
            
            % Filter out directories and store file names
            files = {fileInfo(~[fileInfo.isdir]).name}; % Only include files, not subdirectories

            % Replace .csv with _csv in each file name
            files = cellfun(@(x) strrep(x, '.csv', '_csv'), files, 'UniformOutput', false);

            % Iterate over sensors and calculate RMSE
            for sensorKey = keys(sensorDataTypes)
                currentSensor = sensorKey{1};
                dataTypes = sensorDataTypes(currentSensor);

                % Compute RMSE for each sensor and its data types
                for dataType = dataTypes
%                     for speed = speeds
                    for file = files
                        try
                            rmseValue = feval(calculateRMSE, expData, theoData, currentSensor, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, determineMap, fileToSpeedMap, dataType{1}, file{1}, determineAdjustment, determineOffset);
                            rmseResults.(currentSensor).(dataType{1}).(file{1}) = rmseValue;
                        catch ME
                            warning('Failed to compute RMSE for %s - %s - %s: %s', currentSensor, dataType{1}, file{1}, ME.message);
                            rmseResults.(currentSensor).(dataType{1}).(file{1}) = NaN;
                        end
                    end
                end
            end

            % Save results to CSV
            RMSEUtils.saveResultsToCSV(rmseResults, 'RMSE_Results');
            Mechanism.results = rmseResults;
        end

        function rmse = calculateRMSE(expDataSet, theoDataSet, sensor, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType, speed, determineAdjustment, determineOffset)
            % Calculate RMSE for a specific sensor, data type, and speed
            % Args:
            % - expDataSet, theoDataSet: Experimental and theoretical data sets
            % - sensor, dataType, speed: Sensor name, data type, and speed

            % Retrieve data
            expData = RMSEUtils.retrieveExpData(expDataSet, sensor, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType, speed);
            theoData = RMSEUtils.retrieveTheoData(theoDataSet, expData, sensor, dataType, speed, determineAdjustment, determineOffset);

            % Validate data
            if isempty(expData) || isempty(theoData)
                error('Missing experimental or theoretical data');
            end

            % Interpolate theoretical data to experimental timestamps
            timestamps = expData.Time;
            interpolatedTheoData = interp1(theoData.Time, theoData.Values, timestamps, 'linear', 'extrap');

            % Remove outliers
            [filteredExpData, filteredTheoData] = RMSEUtils.removeOutliers(expData.Values, interpolatedTheoData);

            % Compute RMSE
            if isempty(filteredExpData)
                error('All data points were considered outliers');
            end
            rmse = sqrt(mean((filteredExpData - filteredTheoData).^2));

            % Generate and save the figure
            RMSEUtils.generateAndSaveFigure(timestamps, expData.Values, theoData.Time, theoData.Values, interpolatedTheoData, sensor, dataType, speed);
        end

        function [filteredExpData, filteredTheoData] = removeOutliers(expValues, theoValues)
            % Remove outliers based on the interquartile range (IQR) method
            differences = expValues - theoValues;
            Q1 = quantile(differences, 0.25);
            Q3 = quantile(differences, 0.75);
            IQR = Q3 - Q1;

            lowerBound = Q1 - 1.5 * IQR;
            upperBound = Q3 + 1.5 * IQR;

            % Mask for non-outliers
            nonOutlierMask = (differences >= lowerBound) & (differences <= upperBound);
            filteredExpData = expValues(nonOutlierMask);
            filteredTheoData = theoValues(nonOutlierMask);
        end

        function generateAndSaveFigure(timestamps, expValues, theoTime, theoValues, interpolatedTheoData, sensor, dataType, speed, fileToSpeedMap)
%             speedStr = fileToSpeedMap(strrep(speed, '_csv', '.csv'));
            speedStrDouble = fileToSpeedMap(strrep(speed, '_csv', '.csv'));

            % Convert double to a string with at most 2 decimal places
            speedStr = sprintf('%.2f', speedStrDouble);
            
            % Remove unnecessary trailing zeros
            speedStr = regexprep(speedStr, '\.?0+$', ''); % Removes trailing zeros and a decimal point if it's no longer needed
            speedStr = [speedStr, 'RPM'];

            % Generate and save the figure for RMSE analysis
            fig = figure('Visible', 'off');
            
            % Plot data with thicker lines
            plot(timestamps, expValues, 'b', 'LineWidth', 2, 'DisplayName', 'Experimental Data');
            hold on;
            plot(theoTime, theoValues, 'g', 'LineWidth', 2, 'DisplayName', 'Theoretical Data');
            plot(timestamps, interpolatedTheoData, 'r--', 'LineWidth', 2, 'DisplayName', 'Interpolated Theoretical Data');
            
            % Add legend and labels
            legend('FontSize', 8, 'Location', 'northeast');
            xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold');
            ylabel(dataType, 'FontSize', 16, 'FontWeight', 'bold');
            title(sprintf('RMSE Analysis for %s - %s - %s', sensor, dataType, speedStr), 'FontSize', 16, 'FontWeight', 'bold');
            set(gca, 'FontSize', 12, 'FontWeight', 'bold');
            
            % Save figure
            resultDir = fullfile('RMSE_Results', sensor, dataType, speed);
            if ~exist(resultDir, 'dir'), mkdir(resultDir); end
            savefig(fig, fullfile(resultDir, 'graph.fig'));
            saveas(fig, fullfile(resultDir, 'graph.png'));
            close(fig);
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

        function expData = readExperimentalData(baseExperimentalPath, sensorSourceMap)
            expData = struct(); % Initialize
            subFolders = mapValuesToUniqueArray(sensorSourceMap);  % Subdirectories to iterate through
        
            for i = 1:length(subFolders)
                % Initialize sub-structures for each subfolder
                expData.(subFolders{i}) = struct();
                currentPath = fullfile(baseExperimentalPath, subFolders{i}); % Path to current subdirectory
        
                if strcmp(subFolders{i}, 'WitMotion') % Handling 'WitMotion' directory
                    % Get all CSV files in the directory
                    csvFiles = dir(fullfile(currentPath, '*.csv')); % Adjust pattern if needed
        
                    % Iterate through each CSV file in the directory
                    for j = 1:length(csvFiles)
                        % Get file name and construct full path
                        csvFileName = csvFiles(j).name;
                        csvPath = fullfile(currentPath, csvFileName);
        
                        % Check and read CSV file, including headers
                        if isfile(csvPath)
                            opts = detectImportOptions(csvPath);
                            opts.Delimiter = ',';  % Set the delimiter
                            opts.PreserveVariableNames = true; % Preserve variable names
                            opts.VariableNamesLine = 1; % First row contains variable names
                            opts.DataLine = 2; % Data starts on the second line
        
                            % Safe field name
                            safeFieldName = matlab.lang.makeValidName(csvFileName);
        
                            % Read the table using the specified options
                            expData.(subFolders{i}).(safeFieldName) = readtable(csvPath, opts);
                        end
                    end
        
                elseif strcmp(subFolders{i}, 'CoolTerm') % Handling 'CoolTerm' directory
                    % Get all XLSX files in the directory
                    xlsxFiles = dir(fullfile(currentPath, '*.xlsx')); % Adjust pattern if needed
        
                    % Iterate through each XLSX file in the directory
                    for j = 1:length(xlsxFiles)
                        % Get file name and construct full path
                        xlsxFileName = xlsxFiles(j).name;
                        xlsxPath = fullfile(currentPath, xlsxFileName);
        
                        % Check and read XLSX file
                        if isfile(xlsxPath)
                            % Safe field name
                            safeFieldName = matlab.lang.makeValidName(xlsxFileName);
        
                            % Read the table
                            expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
                        end
                    end
        
                elseif strcmp(subFolders{i}, 'PythonGraph') % Handling 'PythonGraph' directory
                    % Get all XLSX files in the directory
                    xlsxFiles = dir(fullfile(currentPath, '*.xlsx')); % Adjust pattern if needed
        
                    % Iterate through each XLSX file in the directory
                    for j = 1:length(xlsxFiles)
                        % Get file name and construct full path
                        xlsxFileName = xlsxFiles(j).name;
                        xlsxPath = fullfile(currentPath, xlsxFileName);
        
                        % Check and read XLSX file
                        if isfile(xlsxPath)
                            % Safe field name
                            safeFieldName = matlab.lang.makeValidName(xlsxFileName);
        
                            % Read the table
                            expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
                        end
                    end
        
                else
                    % Skip unrecognized directories
                    warning('Skipping unrecognized directory: %s', subFolders{i});
                end
            end
        end


%         function expData = readExperimentalData(baseExperimentalPath, sensorSourceMap)
%             expData = struct(); % Initialize
%             subFolders = mapValuesToUniqueArray(sensorSourceMap);  % Subdirectories to iterate through
%         
%             for i = 1:length(subFolders)
%                 % Initialize sub-structures for each subfolder
%                 expData.(subFolders{i}) = struct();
%                 currentPath = fullfile(baseExperimentalPath, subFolders{i}); % Path to current subdirectory
%         
%                 if strcmp(subFolders{i}, 'WitMotion') % Special handling for 'WitMotion' subdirectory
%                     % Get all CSV files in the directory
%                     csvFiles = dir(fullfile(currentPath, '*.csv')); % Adjust pattern if needed
%         
%                     % Iterate through each CSV file in the directory
%                     for j = 1:length(csvFiles)
%                         % Get file name and construct full path
%                         csvFileName = csvFiles(j).name;
%                         csvPath = fullfile(currentPath, csvFileName);
%         
%                         % Check and read CSV file, including headers
%                         if isfile(csvPath)
%                             opts = detectImportOptions(csvPath);
%                             opts.Delimiter = ',';  % Set the delimiter
%         
%                             % Ensure the variable names (headers) are preserved as they are in the file
%                             opts.PreserveVariableNames = true;
%         
%                             % Specify that the first row contains the headers
%                             opts.VariableNamesLine = 1;  % This tells MATLAB that the first line contains variable names (headers)
%         
%                             % Ensure data starts reading from the line after the headers
%                             opts.DataLine = 2;  % Start reading data from the second line, assuming the first line is the header
%         
%                             % Safe field name (replace non-alphanumeric characters if needed)
%                             safeFieldName = matlab.lang.makeValidName(csvFileName);
%         
%                             % Read the table using the specified options
%                             expData.(subFolders{i}).(safeFieldName) = readtable(csvPath, opts);
%                         end
%                     end
%                 elseif strcmp(subFolders{i}, 'CoolTerm') % For 'CoolTerm', read XLSX files
%                     % Iterate through expected filenames (predetermined)
%                     for j = 1:length(filenames)
%                         safeFieldName = filenames{j};
%                         xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
%         
%                         % Check and read XLSX file
%                         if isfile(xlsxPath)
%                             expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
%                         end
%                     end
%                 elseif strcmp(subFolders{i}, 'PythonGraph') % For 'PythonGraph', read XLSX files
%                     % Iterate through expected filenames (predetermined)
%                     for j = 1:length(filenames)
%                         safeFieldName = filenames{j};
%                         xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
%         
%                         % Check and read XLSX file
%                         if isfile(xlsxPath)
%                             expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
%                         end
%                     end
%                 else
%                     % Handle other cases or give a warning/error
%                     warning('Unknown subfolder type: %s', subFolders{i});
%                 end
%             end
%         end

%         function expData = readExperimentalData(baseExperimentalPath, sensorSourceMap)
%             expData = struct(); % Initialize
%             subFolders = mapValuesToUniqueArray(sensorSourceMap);  % Subdirectories to iterate through
% %             filenames = speeds; % RPM filenames
% 
%             for i = 1:length(subFolders)
%                 % Initialize sub-structures for each subfolder
%                 expData.(subFolders{i}) = struct();
%                 currentPath = fullfile(baseExperimentalPath, subFolders{i}); % Path to current subdirectory
% 
%                 for j = 1:length(filenames)
%                     safeFieldName = filenames{j};
% 
%                     % Construct file path
%                     if strcmp(subFolders{i}, 'CoolTerm') % For 'CoolTerm', read XLSX files
%                         xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
%                         % Check and read XLSX file
%                         if isfile(xlsxPath)
%                             expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
%                             % expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath, 'Range', 'A1'); % Adjust 'Range' if necessary
%                         end
%                     elseif strcmp(subFolders{i}, 'WitMotion') % For 'WitMotion', read CSV files
%                         csvPath = fullfile(currentPath, filenames{j} + ".csv");
%                         % Check and read CSV file, including headers
%                         if isfile(csvPath)
%                             opts = detectImportOptions(csvPath);
%                             opts.Delimiter = ',';  % Set the delimiter
% 
%                             % Ensure the variable names (headers) are preserved as they are in the file
%                             opts.PreserveVariableNames = true;
% 
%                             % Specify that the first row contains the headers
%                             opts.VariableNamesLine = 1;  % This tells MATLAB that the first line contains variable names (headers)
% 
%                             % Ensure data starts reading from the line after the headers
%                             opts.DataLine = 2;  % Start reading data from the second line, assuming the first line is the header
% 
%                             % Read the table using the specified options
%                             expData.(subFolders{i}).(safeFieldName) = readtable(csvPath, opts);
%                         end
%                     elseif strcmp(subFolders{i}, 'PythonGraph') % For 'PythonGraph', read XLSX files
%                         xlsxPath = fullfile(currentPath, filenames{j} + ".xlsx");
%                         % Check and read XLSX file
%                         if isfile(xlsxPath)
%                             expData.(subFolders{i}).(safeFieldName) = readtable(xlsxPath);
%                             % If needed, you can specify 'Range' and other options in 'readtable'
%                         end
%                     else
%                         % Handle other cases or give a warning/error
%                         warning('Unknown subfolder type: %s', subFolders{i});
%                     end
%                 end
%             end
%         end

        % Retriev the desired experimental data
        function expData = retrieveExpData(dataSet, sensor, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType, speed)
            % Map sensors to their respective data sources (CoolTerm or WitMotion)
            source = sensorSourceMap(sensor);
            % Check if the required data is available
            if isfield(dataSet, source) && isfield(dataSet.(source), speed)
                rawData = dataSet.(source).(speed); % nxm table of data
                % expData = processData(rawData, sensor, dataType);
                if (strcmp(source, 'CoolTerm'))
                    expData = processCoolTermData(rawData, sensor, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType);
                elseif (strcmp(source, 'WitMotion'))
                    expData = processWitMotionData(rawData, sensor, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType);
                elseif (strcmp(source, 'PythonGraph'))
                    expData = processPythonGraphData(rawData, sensor, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType);
                else
                    warming('Application utilized to analyze data is unknown');
                    expData = [];
                end
            else
                expData = []; % Return empty if not found
            end
        end

        function theoData = retrieveTheoData(dataSet, expData, sensor, dataType, file, determineAdjustment, determineOffset, fileToSpeedMap)
            % Determine the main category based on dataType
            speedDouble = fileToSpeedMap(strrep(file, '_csv', '.csv'));

            % Extract integer and fractional parts
            integerPart = floor(speedDouble);
            fractionalPart = round(mod(speedDouble, 1) * 100); % Extract fractional part and scale to 2 digits
            
            % Construct the formatted string
            if fractionalPart == 0
                % No fractional part, just use the integer part
                speed = sprintf('f%dRPM', integerPart);
            elseif mod(fractionalPart, 10) == 0
                % Fractional part has a trailing zero, use only the first digit
                speed = sprintf('f%d_%dRPM', integerPart, fractionalPart / 10);
            else
                % Use both digits of the fractional part
                speed = sprintf('f%d_%02dRPM', integerPart, fractionalPart);
            end
            
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
                                 rpmValue = speedDouble;
%                                 rpmValue = str2double(strrep(regexp(speed, '\d+_\d+|\d+', 'match'), '_', '.'));
                                timePerRevolution = 60 / rpmValue;  % Time for one full revolution (in seconds)
                                numDataPoints = size(theoDataArray, 1);  % Number of data points in the theoretical data
                                theoreticalTime = linspace(0, timePerRevolution, numDataPoints).';  % Linearly spaced time array
                                theoData.Time = theoreticalTime;  % Set the time for theoData
                            else
                                theoDataArray = double(dataField.(sensorFields{i}){:, 3});

                                % Calculate Time Before Adjustment
%                                 rpmValue = str2double(strrep(regexp(file, '\d+_\d+|\d+', 'match'), '_', '.'));
                                rpmValue = speedDouble;
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

function coolTermData = processCoolTermData(rawData, sensorType, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType)

end

function pythonGraphData = processPythonGraphData(rawData, sensorType, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType)

end

function witMotionData = processWitMotionData(rawData, sensorType, sensorDataFlipMap, pullColumnDataMap, determineMap, dataType)
columnHeaders = rawData.Properties.VariableNames;
% Constants for column indices based on data type
TIME_COL = 1; % Time column index
SENSOR_ID_COL = 2; % Sensor ID column index
Angle_Z_COL = find(contains(columnHeaders, 'Angle Z')); % Column index for Angle Y

% Mapping sensor types to their corresponding sensor ID
[sensorMap, letterMap] = feval(determineMap, rawData, SENSOR_ID_COL);

inputLinkID = sensorMap('E');  % Always use sensor 'H' for zero crossing reference

% Filter data for the input link to find zero crossings
inputLinkData = rawData(contains(rawData{:, SENSOR_ID_COL}, inputLinkID), :);
zeroCrossings = find(diff(sign(table2array(inputLinkData(:, Angle_Z_COL)))) > 0) + 1;
if length(zeroCrossings) < 2
    error('Not enough zero crossings found for input link.');
end

% Determine start and end times for valid data using input link zero crossings
validStartTime = duration(table2array(inputLinkData(zeroCrossings(1), TIME_COL)));
validEndTime = duration(table2array(inputLinkData(zeroCrossings(2), TIME_COL)));

% Filter data for the current sensor type
sensorID = sensorMap(sensorType);
sensorData = rawData(contains(rawData{:, SENSOR_ID_COL}, sensorID), :);

% Find indices in sensorData that are within the valid time range determined by the input link
validIndices = sensorData{:, TIME_COL} >= validStartTime & sensorData{:, TIME_COL} <= validEndTime;
if sum(validIndices) == 0
    error('No data found for the current sensor within the valid time range.');
end

% Extract data slice based on the valid time indices
validData = sensorData(validIndices, :);

% Further refinement based on dataType to extract only relevant data
if (strcmp(dataType, 'Angle'))
    startColumn = find(contains(validData.Properties.VariableNames, "Angle X"));
elseif (strcmp(dataType, 'AngVel'))
    startColumn = find(contains(validData.Properties.VariableNames, "Angular velocity X"));
else
    disp("ERROR: DATATYPE NOT 'Angle' NOR 'AngVel'");
end

% Define the range from the start column to the next two columns
if ~isempty(startColumn)  % Ensure that a match was found
    dataColumns = startColumn:(startColumn + 2);  % Create the range of three columns
else
    dataColumns = [];  % Handle the case where no match is found
    disp("ERROR: DATACOLUMN NOT SET UP CORRECTLY");
end

% dataColumns = find(contains(validData(:, "Angle X")));
refinedData = validData(:, dataColumns);

% Prepare output structure
witMotionData = struct();
witMotionData.Time = table2array(validData(:, TIME_COL));
% Determine the actual start time by utilizing interpolation to find
% the starting theoretical where the input link is 0
x = [inputLinkData{zeroCrossings(1,1), Angle_Z_COL}, inputLinkData{zeroCrossings(1,1)-1, Angle_Z_COL}];  % Example angles in degrees

% Define the corresponding y values (times) as duration type
y = [inputLinkData{zeroCrossings(1,1), TIME_COL}, inputLinkData{zeroCrossings(1,1)-1, TIME_COL}];  % Example times

% Define the x value at which you want to interpolate
xq = 0;  % Instance where input link starts at 0 degree angle

% Perform interpolation using interp1 function
witMotionStartingTime = interp1(x, y, xq, 'linear');

witMotionData.Time = seconds(witMotionData.Time - witMotionStartingTime);
% Extract the values once to avoid repetition
columnExtractionIndex = pullColumnDataMap(strcat(sensorType, dataType));
values = table2array(refinedData(:, columnExtractionIndex));
% values = values * sensorDataFlipMap(strcat(sensorType, dataType));
if (sensorDataFlipMap(strcat(sensorType, dataType)) == 2)
    values = flip(values);
elseif (sensorDataFlipMap(strcat(sensorType, dataType)) == 3)
    values = values * -1;
    values = flip(values);
end

% Define the mapping for conversion based on conditions
if contains([letterMap(sensorID) dataType], 'EAngVel') || contains([letterMap(sensorID) dataType], 'FAngVel') || contains([letterMap(sensorID) dataType], 'GAngVel') || contains([letterMap(sensorID) dataType], 'HAngVel')
    witMotionData.Values = values * pi / 180; % Convert from deg/s to rad/s

elseif contains([letterMap(sensorID) dataType], 'EAngle') || contains([letterMap(sensorID) dataType], 'FAngle') || contains([letterMap(sensorID) dataType], 'GAngle') || contains([letterMap(sensorID) dataType], 'HAngle')
    witMotionData.Values = values; % No additional conversion required

else
    witMotionData.Values = table2array(refinedData(:, 1)); % Default case
end


% witMotionData.Values = refinedData;
witMotionData.SensorID = sensorID;  % Include sensor ID in the output for reference
end



