function Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap)
 % TODO: Make sure to insert the processFunctions in as an argument and
 % utilize this within code
    Mechanism = RMSEUtils.RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, @processCoolTermData, @processPythonGraphData, @processWitMotionData);
end

% TODO: Put the logic here for pulling the respective column for
% WitMotion/CoolTerm/PythonGraphs

function coolTermData = processCoolTermData(rawData, sensorType, dataType)

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
                {'E', 'F', 'G', 'H'}, ...
                {'WTCrank(c3:a3:c5:ed:f8:8e)', ...
                'WTCoupler_In(d2:d6:ae:79:8c:70)' ...
                'WTCoupler_Out(f9:7a:4a:df:60:c4)' ...
                'WTRocker(e2:c7:f5:e1:23:63)'});
            inputLinkID = sensorMap('E');  % use sensor 'E' for zero crossing reference

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
            witMotionData.Time = validData(:, TIME_COL);
            % TODO: Update this accordingly
            witMotionData.Values = refinedData(:,1);
            % witMotionData.Values = refinedData;
            witMotionData.SensorID = sensorID;  % Include sensor ID in the output for reference
        end