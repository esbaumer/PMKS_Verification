function Mechanism = RMSE(Mechanism, sensorDataTypes, sensorSourceMap)
% TODO: Make sure to insert the processFunctions in as an argument and
% utilize this within code
Mechanism = RMSEUtils.RMSESolver(Mechanism, sensorDataTypes, sensorSourceMap, @processCoolTermData, @processPythonGraphData, @processWitMotionData);
end

function coolTermData = processCoolTermData(rawData, sensorType, dataType)

end

function witMotionData = processWitMotionData(rawData, sensorType, dataType)

end

function pythonGraphData = processPythonGraphData(rawData, sensorType, dataType)
% Constants for column indices
TIMESTEP_COL = 1;
HALL_SENSOR_COL = 2;
EST_RPM_COL = 3;
BNO_VL_PISTON = 4;
BNO_ANG_VEL_COL = 7;
% BNO_ORIENTATION_START_COL = 5; % X, Y, Z orientation start from this column
% BNO_ORIENTATION_END_COL = 7; % X, Y, Z orientation end at this column
% BNO_ANG_VEL_COL = 8;

binarySignal = rawData.HallEffectSensor;  % Adjust 'Var3' to the correct variable name if different

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

% Extract data within the valid range
validData = rawData(validStartIndex:validEndIndex, :);

% Determine columns based on dataType
% switch dataType
%     case 'Angle'
%         columns = BNO_ORIENTATION_START_COL:BNO_ORIENTATION_END_COL;
%     case 'AngVel'
%         columns = BNO_ANG_VEL_COL;
%     case 'LinAcc'
%         columns = ADXL_PISTON_LIN_ACC_COL;
%     otherwise
%         error('Unknown dataType specified');
% end

% Insert a time step column based on estimated RPM (convert to radians per second first)
% estRpm = rawData{validStartIndex, EST_RPM_COL};
% omega = estRpm * (2 * pi / 60); % Convert RPM to radians per second
% timesteps = (0 : height(validData) - 1)' / omega; % Create a timestep array
% validData.Timestep = seconds(timesteps); % Insert as duration in seconds
% validData.Timestep = seconds(validData)
% Assuming 'validData' is your table with a 'Time' column that are durations
firstTimestamp = validData.Timestamp(1); % Get the first timestamp

% Subtract the first timestamp from all timestamps to get the relative times
relativeTimes = validData.Timestamp - firstTimestamp;

% Convert the relative times from durations to seconds


% Now 'validData.Timestep' contains the time in seconds relative to the first timestamp


% Select and store the desired data based on dataType and sensor
pythonGraphData = struct();
% pythonGraphData.Time = validData.Timestep; % Use the new Timestep column
% pythonGraphData.Values = validData(:, columns); % Data of interest
% pythonGraphData.SensorID = sensor; % Include sensor ID for reference

% yColumnMap = containers.Map(...
%     {'EAngle', 'EAngVel', 'FLinAcc'}, ...
%     {2, 1, 1});

% Get the correct Y column index using the map
% sensorID = sensorMap(sensorType);
% sensorID =

% yColumnIndex = columns(yColumnMap([sensorType dataType]));
% TODO: Adjust accordingly once other sensors are added
yColumnIndex = BNO_ANG_VEL_COL;

YData = table2array(validData(:, yColumnIndex));
XData = seconds(relativeTimes / 1000);
% Refine data by ensuring continuity and removing spikes (maybe do later)
% refinedData = validData(refinedDataIndices, :);
% continuousData = removeSpikes(refinedData, columns);

% Store processed data for output
% pythonGraphData.Time = XData;  % Time column
pythonGraphData.Time = XData;
pythonGraphData.Values = YData;  % Extracted values based on dataType and sensor

% % Define the range for valid data based on identified segments
% validStartIndex = oneIndices(validSegments(1));
% validEndIndex = oneIndices(validSegments(2));
% 
% % Extract data within the valid range
% validData = rawData(validStartIndex:validEndIndex, :);
% 
% % Determine columns based on dataType
% switch dataType
%     case 'Angle'
%         columns = BNO_ORIENTATION_START_COL:BNO_ORIENTATION_END_COL;
%     case 'AngVel'
%         columns = BNO_ANG_VEL_COL;
%     case 'LinAcc'
%         columns = ADXL_PISTON_LIN_ACC_COL;
%     otherwise
%         error('Unknown dataType specified');
% end
% 
% % Insert a time step column based on estimated RPM (convert to radians per second first)
% estRpm = rawData{validStartIndex, EST_RPM_COL};
% omega = estRpm * (2 * pi / 60); % Convert RPM to radians per second
% timesteps = (0 : height(validData) - 1)' / omega; % Create a timestep array
% validData.Timestep = seconds(timesteps); % Insert as duration in seconds
% 
% % Select and store the desired data based on dataType and sensor
% pythonGraphData = struct();
% % pythonGraphData.Time = validData.Timestep; % Use the new Timestep column
% % pythonGraphData.Values = validData(:, columns); % Data of interest
% % pythonGraphData.SensorID = sensor; % Include sensor ID for reference
% 
% yColumnMap = containers.Map(...
%     {'EAngle', 'EAngVel', 'FLinAcc'}, ...
%     {2, 1, 1});
% 
% % Get the correct Y column index using the map
% % sensorID = sensorMap(sensorType);
% % sensorID =
% 
% yColumnIndex = columns(yColumnMap([sensorType dataType]));
% 
% YData = table2array(validData(:, yColumnIndex));
% XData = timesteps;
% % Refine data by ensuring continuity and removing spikes (maybe do later)
% % refinedData = validData(refinedDataIndices, :);
% % continuousData = removeSpikes(refinedData, columns);
% 
% % Store processed data for output
% pythonGraphData.Time = XData;  % Time column
% pythonGraphData.Values = YData;  % Extracted values based on dataType and sensor
end

