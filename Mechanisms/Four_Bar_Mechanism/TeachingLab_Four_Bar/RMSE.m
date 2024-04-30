function Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap)
 % TODO: Make sure to insert the processFunctions in as an argument and
 % utilize this within code
    Mechanism = RMSEUtils.RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, @processCoolTermData, @processPythonGraphData, @processWitMotionData);
end

% TODO: Put the logic here for pulling the respective column for
% WitMotion/CoolTerm/PythonGraphs

function coolTermData = processCoolTermData(rawData, sensorType, dataType)
% Define sensor columns for angles and angular velocities
            sensorColumnsMap = containers.Map(...
                {'F', 'E', 'G'}, ...
                {struct('AngVel', 4, 'Angle', 5:7), ...
                struct('Angle', 8:10, 'AngVel', 11:13), ...
                struct('Angle', 14:16, 'AngVel', 17:19)});
            columns = sensorColumnsMap(sensorType).(dataType);

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

            % Define map for selecting the correct Y column index based on sensor and dataType
            yColumnMap = containers.Map(...
                {'FAngVel', 'FAngle', 'EAngle', 'EAngVel', 'GAngle', 'GAngVel'}, ...
                {1, 2, 3, 1, 3, 1});
            % {1, 2, 1, 3, 1, 3});

            % Get the correct Y column index using the map
            yColumnIndex = columns(yColumnMap([sensorType dataType]));

            YData = validData(:, yColumnIndex);
            XData = validData(:, 2);
            % Refine data by ensuring continuity and removing spikes (maybe do later)
            % refinedData = validData(refinedDataIndices, :);
            % continuousData = removeSpikes(refinedData, columns);

            % Store processed data for output
            coolTermData.Time = table2array(XData);  % Time column
            timestamps = coolTermData.Time - coolTermData.Time(1,1);
            timestamps = timestamps / 1000;
            coolTermData.Time = timestamps;
            coolTermData.Values = table2array(YData);  % Extracted values based on dataType and sensor
            if (strcmp([sensorType dataType], 'EAngVel') || strcmp([sensorType dataType], 'GAngVel'))
            coolTermData.Values = coolTermData.Values * -1;
            end
end

function pythonGraphData = processPythonGraphData(rawData, sensorType, dataType)

end

function witMotionData = processWitMotionData(rawData, sensorType, dataType)
            % Constants for column indices based on data type
            TIME_COL = 1; % Time column index
            SENSOR_ID_COL = 2; % Sensor ID column index
            ANGLE_Y_COL = 11; % Column index for Angle Y

            % Mapping sensor types to their corresponding sensor ID
            sensorMap = containers.Map(...
                {'H', 'I'}, ...
                {'WT901BLE68(ef:e4:2c:bf:73:a9)', 'WT901BLE68(d2:5a:50:4a:21:12)'});
            letterMap = containers.Map(...
                {'WT901BLE68(ef:e4:2c:bf:73:a9)', 'WT901BLE68(d2:5a:50:4a:21:12)'}, ...
                {'H', 'I'});
            inputLinkID = sensorMap('H');  % Always use sensor 'H' for zero crossing reference

            % Filter data for the input link to find zero crossings
            inputLinkData = rawData(strcmp(rawData{:, SENSOR_ID_COL}, inputLinkID), :);
            zeroCrossings = find(diff(sign(table2array(inputLinkData(:, ANGLE_Y_COL)))) > 0) + 1;
            if length(zeroCrossings) < 2
                error('Not enough zero crossings found for input link.');
            end

            % Determine start and end times for valid data using input link zero crossings
            validStartTime = duration(table2array(inputLinkData(zeroCrossings(1), TIME_COL)));
            validEndTime = duration(table2array(inputLinkData(zeroCrossings(2), TIME_COL)));

            % Filter data for the current sensor type
            sensorID = sensorMap(sensorType);
            sensorData = rawData(strcmp(rawData{:, SENSOR_ID_COL}, sensorID), :);

            % Find indices in sensorData that are within the valid time range determined by the input link
            validIndices = sensorData{:, TIME_COL} >= validStartTime & sensorData{:, TIME_COL} <= validEndTime;
            if sum(validIndices) == 0
                error('No data found for the current sensor within the valid time range.');
            end

            % Extract data slice based on the valid time indices
            validData = sensorData(validIndices, :);

            % Further refinement based on dataType to extract only relevant data
            dataColumns = RMSEUtils.getDataColumns(dataType);
            refinedData = validData(:, dataColumns);

            % Prepare output structure
            witMotionData = struct();
            witMotionData.Time = table2array(validData(:, TIME_COL));
            witMotionData.Time = seconds(witMotionData.Time - witMotionData.Time(1));
            % TODO: Update this accordingly
            if (strcmp([letterMap(sensorID) dataType], 'HAngVel'))
               witMotionData.Values = table2array(refinedData(:,1));
               witMotionData.Values = witMotionData.Values * -1;
            elseif (strcmp([letterMap(sensorID) dataType], 'IAngVel'))
               witMotionData.Values = table2array(refinedData(:,3));
               witMotionData.Values = witMotionData.Values * -1;
            else
               witMotionData.Values = table2array(refinedData(:,1));
            end
            % witMotionData.Values = refinedData;
            witMotionData.SensorID = sensorID;  % Include sensor ID in the output for reference
        end