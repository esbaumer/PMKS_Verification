function Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap)
% TODO: Make sure to insert the processFunctions in as an argument and
% utilize this within code
Mechanism = RMSEUtils.RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, @processCoolTermData, @processPythonGraphData, @processWitMotionData, @determineAdjustment, @determineOffset);
end

% TODO: Put the logic here for pulling the respective column for
% WitMotion/CoolTerm/PythonGraphs

function coolTermData = processCoolTermData(rawData, sensorType, dataType)
end

function pythonGraphData = processPythonGraphData(rawData, sensorType, dataType)

end

function witMotionData = processWitMotionData(rawData, sensorType, dataType)
columnHeaders = rawData.Properties.VariableNames;
% Constants for column indices based on data type
TIME_COL = 1; % Time column index
SENSOR_ID_COL = 2; % Sensor ID column index
Angle_Z_COL = find(contains(columnHeaders, 'Angle Z')); % Column index for Angle Y

% Mapping sensor types to their corresponding sensor ID
% sensorMap = containers.Map(...
%     {'E', 'F', 'G', 'H'}, ...
%     {'ef:e4:2c:bf:73:a9', 'd2:5a:50:4a:21:12', 'c3:a3:c5:ed:f8:8e', 'd2:d6:ae:79:8c:70'});
% letterMap = containers.Map(...
%     {'ef:e4:2c:bf:73:a9', 'd2:5a:50:4a:21:12', 'c3:a3:c5:ed:f8:8e', 'd2:d6:ae:79:8c:70'}, ...
%     {'E', 'F', 'G', 'H'});
[sensorMap, letterMap] = determineMap(rawData, SENSOR_ID_COL);

% sensorMap = containers.Map(...
%     {'E', 'F', 'G'}, ...
%     {'COM10', 'COM9', 'COM7'});
% letterMap = containers.Map(...
%     {'COM10', 'COM9', 'COM7'}, ...
%     {'E', 'F', 'G'});
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
% dataColumns = RMSEUtils.getDataColumns(dataType);
if (strcmp(dataType, 'Angle'))
    startColumn = find(contains(validData.Properties.VariableNames, "Angle X"));
elseif (strcmp(dataType, 'AngVel'))
    startColumn = find(contains(validData.Properties.VariableNames, "Angular velocity X"));
else
    disp("ERROR: DATATYPE NOT 'Angle' NOR 'AngVel'");
end
% startColumn = find(contains(validData.Properties.VariableNames, "Angle X"));

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
values = table2array(refinedData(:, 3));

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

function adjustment = determineAdjustment(sensor, theoData, actualData)
     switch sensor
        case {'E', 'F', 'G', 'H'}
            adjustment = theoData - 90 - actualData;
        otherwise
            error('Invalid sensor type.');
    end
end

function offset = determineOffset(sensor, theoDataArray, adjustmentVal)
   % Extract the Time field from the theoData struct
    % timeData = theoData.Time; % 361x1 double array

    % Initialize offset to be the same size as timeData
    offset = zeros(size(theoDataArray));
    
    % Determine the offset based on the sensor type
    switch sensor
        case {'E', 'F', 'G', 'H'}
            offset = theoDataArray - 90 - adjustmentVal; % Apply the formula to each element
        otherwise
            error('Invalid sensor type.');
    end
end

function [selectedMap, letterMap] = determineMap(rawData, SENSOR_ID_COL)
    % Define potential maps
    sensorMap1 = containers.Map(...
        {'E', 'F', 'G', 'H'}, ...
        {'ef:e4:2c:bf:73:a9', 'd2:5a:50:4a:21:12', 'c3:a3:c5:ed:f8:8e', 'd2:d6:ae:79:8c:70'});
    sensorMap2 = containers.Map(...
        {'E', 'F', 'G'}, ...
        {'COM16', 'COM14', 'COM13'});
%         {'COM10', 'COM9', 'COM7'});
    
    % Get unique device names from rawData
    uniqueDeviceNames = unique(rawData{:, SENSOR_ID_COL});
    
    % Determine which map to use based on presence of known keys
    if all(ismember(values(sensorMap1), uniqueDeviceNames))
        selectedMap = sensorMap1;
    elseif all(ismember(values(sensorMap2), uniqueDeviceNames))
        selectedMap = sensorMap2;
    else        
        % Raise an error if no suitable map can be determined
        error('Please add the corresponding device name within determineMap function within RMSE function');
    end
    % Create the reverse map (letterMap) dynamically
    letterMap = containers.Map(values(selectedMap), keys(selectedMap));
end


