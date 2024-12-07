function Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap)
% TODO: Make sure to insert the processFunctions in as an argument and
% utilize this within code
Mechanism = RMSEUtils.RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, sensorDataFlipMap, pullColumnDataMap, @determineAdjustment, @determineOffset, @determineMap);
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


