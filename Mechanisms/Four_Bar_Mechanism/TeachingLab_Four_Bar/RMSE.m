clear; close all; clc;

% Define the base path for each type of data
baseTheoreticalPosPath = 'CSVOutput/Pos/Joint/';
baseTheoreticalVelPath = 'CSVOutput/Vel/AngVel/';
baseTheoreticalAccPath = 'CSVOutput/Acc/AngAcc/';

CoolTermExperimentalPath = 'Experimental/CoolTerm';
WitMotionExperimentalPath = 'Experimental/WitMotion';

% Read the desired theoretical files to analyze
theoreticalJointData = readTheoreticalJointData(baseTheoreticalPosPath);
theoreticalAngleData = determineAngles(theoreticalJointData); % Determine the angles based on the passed in joint data
theoreticalAngVelData = readTheoreticalLinkData(baseTheoreticalVelPath);
theoreticalAngAccData = readTheoreticalLinkData(baseTheoreticalAccPath);

% Read the desired experimental files to analyze
experimentalCoolTermData = readExperimentalCoolTermData(CoolTermExperimentalPath);
experimentalWitMotionData = readExperimentalWitMotionData(WitMotionExperimentalPath);
%experimentalCTMPUAngleData = experimentalCoolTermData.Angle;
%experimentalCTAngVelData = experimentalCoolTermData.AngVel;
%experimentalCTAngAccData = experimentalCoolTermData.AngAcc;

experimentalCTMPUYAngleData = experimentalCoolTermData.MPUOrientationX;
experimentalCTMPUXAngleData =  experimentalCoolTermData.MPUOrientationY;
experimentalCTtimestep = experimentalCoolTermData.timestep;

experimentalWMtimestep = experimentalWitMotionData.WMtimestep;

firsttime = experimentalCTtimestep(1);
for i=1:length(experimentalCTtimestep)
    experimentalCTtimestep(i) = experimentalCTtimestep(i)-firsttime;
end

%experimentalWMtimestep(1) = extractAfter(experimentalWMtimestep(1), ':');
startingWM = experimentalWMtimestep(1);
for i=2:length(experimentalWMtimestep)
%    experimentalWMtimestep(i) = extractAfter(experimentalWMtimestep(i), ':');
    experimentalWMtimestep(i) = experimentalWMtimestep(i)-startingWM;
end

%interpolatedMPUAngle = interp(time, experimentalCTMPUYAngleData);
experimentalCTBNO1ZAngleData = experimentalCoolTermData.BNOCouplerOrientationZ;
experimentalCTBNO2ZAngleData =  experimentalCoolTermData.BNORockerOrientationZ;


experimentalWMInputAngleData = experimentalWitMotionData.InputZAngle;
experimentalWMRockerAngleData = experimentalWitMotionData.OutputZAngle;
%experimentalWMAngAccData = experimentalWitMotionData.AngAcc;

% Interpolated Theroetical
interpolatedAngle = interp1(theoreticalAngleData.x, theoreticalAngleData.y, experimentalData.x);
interpolatedAngVel = interp1(theoreticalAngVelData.x, theoreticalAngVelData.y, experimentalData.x);
interpolatedAngAcc = interp1(theoreticalAngAccData.x, theoreticalAngAccData.y, experimentalData.x);

% Determine the rmse for link components
rmse_angle = sqrt(mean((experimentalAngleData.y - interpolatedAngle).^2));
rmse_angVel = sqrt(mean((experimentalAngVelData.y - interpolatedAngVel).^2));
rmse_angAcc = sqrt(mean((experimentalAngAccData.y - interpolatedAngAcc).^2));

function jointData = readTheoreticalJointData(basePath)
files = dir(fullfile(basePath, '*.csv')); % List all CSV files in the directory
jointData = struct('jointName', {}, 'data', {}); % Initialize empty struct array
    for i = 1:length(files)
        jointName = files(i).name(1:end-4); % Remove '.csv' from filename to get joint name
        filePath = fullfile(files(i).folder, files(i).name);
        jointData(i).jointName = jointName;
        jointData(i).data = readtable(filePath); % Read CSV file into table
    end
end
% function jointData = readExperimentalJointData(basePath)
%     files = dir(fullfile(basePath, '*.csv')); % List all CSV files in the directory
%     jointData = struct('jointName', {}, 'data', {}); % Initialize empty struct array
%
%     for i = 1:length(files)
%         jointName = files(i).name(1:end-4); % Remove '.csv' from filename to get joint name
%         filePath = fullfile(files(i).folder, files(i).name);
%         jointData(i).jointName = jointName;
%         jointData(i).data = readtable(filePath); % Read CSV file into table
%     end
% end

function linkData = readTheoreticalLinkData(basePath)
files = dir(fullfile(basePath, '*.csv')); % List all CSV files in the directory
linkData = struct('linkName', {}, 'data', {}); % Initialize empty struct array

for i = 1:length(files)
    linkName = files(i).name(1:end-4); % Remove '.csv' from filename to get link name
    filePath = fullfile(files(i).folder, files(i).name);
    linkData(i).linkName = linkName;
    linkData(i).data = readtable(filePath); % Read CSV file into table
end
end
% function linkData = readExperimentalLinkData(basePath)
%     files = dir(fullfile(basePath, '*.csv')); % List all CSV files in the directory
%     linkData = struct('linkName', {}, 'data', {}); % Initialize empty struct array
%
%     for i = 1:length(files)
%         linkName = files(i).name(1:end-4); % Remove '.csv' from filename to get link name
%         filePath = fullfile(files(i).folder, files(i).name);
%         linkData(i).linkName = linkName;
%         linkData(i).data = readtable(filePath); % Read CSV file into table
%     end
% end

function adjustedAngles = determineAngles(jointData)
% Assuming jointData is a structure or table with fields 'jointID' and 'angle'
% Initialize an array to store adjusted angles with the same size as the input
if ~isempty(jointData) && isfield(jointData(1), 'data')
    % Assuming all 'data' fields have the same number of rows (361 in your case)
    numberOfRows = 5;
    % Adjusted initialization of adjustedAngles based on dynamic size
    adjustedAngles = zeros(numberOfRows, length(jointData)); % Flipped to match 361x6
else
    % Handle the case where jointData might be empty or not properly structured
    adjustedAngles = []; % Or any other fallback initialization
end

% Create a map from 'jointData'
jointIndexMap = containers.Map('KeyType', 'char', 'ValueType', 'int32');

for i = 1:length(jointData)
    jointIndexMap(jointData(i).jointName) = i;
end

for theta_iterator = 1:size(jointData(1))
    for rowNum = 1:5
        % Determine the offset based on jointID. Example adjustments:
        if theta_iterator == 1 % For joint A with respect to another joint
            % Pull the appropriate joint values 
            A = table2array(jointData(jointIndexMap('A')).data(theta_iterator,:));
            B = table2array(jointData(jointIndexMap('B')).data(theta_iterator,:));
            angle = atan2(B(2) - A(2), B(1) - A(1));
            % TODO: Do this process for all desired joint positions
            adjustedAngles(rowNum, theta_iterator) = 180 - angle;
        elseif theta_iterator == 2 % For joint B
            A = table2array(jointData(jointIndexMap('A')).data(theta_iterator,:));
            B = table2array(jointData(jointIndexMap('B')).data(theta_iterator,:));
            angle = atan2(B(2) - A(2), B(1) - A(1));
%CHECK ON THIS PART
            your_offset_AB = 180-angle;

            adjustedAngles(theta_iterator) = theta + your_offset_AB;
        elseif theta_iterator == 3 % For joint C
            % Adjust angle based on your criteria for joint B
            adjustedAngles(theta_iterator) = theta + your_offset_AB;
        elseif theta_iterator == 4 % For joint D
            % Adjust angle based on your criteria for joint B
            adjustedAngles(theta_iterator) = theta + your_offset_AB;
        elseif theta_iterator == 5 % For joint E
            % Adjust angle based on your criteria for joint B
            adjustedAngles(theta_iterator) = theta + your_offset_AB;
        elseif theta_iterator == 6 % For joint F
            % Default case if no specific offset criteria are met
            adjustedAngles(theta_iterator) = theta; % No adjustment
        end
        % Add more conditions as needed for other joints
    end
end
end

function data =  readExperimentalCoolTermData(CoolTermExperimentalPath)
    % Directory and filenames setup
    dirPath = fullfile(CoolTermExperimentalPath); % Adjust base path as necessary
    filenames = {'10RPM.xlsx', '20RPM.xlsx', '30RPM.xlsx'};

    % Initialize an empty table for concatenated results
    concatenatedData = [];

    % Loop to Process Each File
    for i = 1:length(filenames)
        fullPath = fullfile(dirPath, filenames{i});
        
        % Check if the file exists before attempting to read
        if isfile(fullPath)
            % Detect import options based on the current file and sheet
            opts = detectImportOptions(fullPath, 'NumHeaderLines', 3);
            % Read the data into a table
            tempData = readtable(fullPath, opts);

            % Concatenate the new data table to the existing data
            if isempty(concatenatedData)
                concatenatedData = tempData;
            else
                concatenatedData = [concatenatedData; tempData]; % Assuming the data structure is the same across files
            end
        else
            warning('File does not exist: %s', fullPath);
        end
    end


    % Process concatenated data to find specific sensor values
    % Note: The following processing assumes 'concatenatedData' structure is consistent across files
    
    % Adjust column name as per your actual data structure
    hallSensorColumn = concatenatedData.Var3;
    oneIndices = find(hallSensorColumn == 1);

    if length(oneIndices) < 2
        disp('Not enough data points where Hall sensor equals 1.');
        return; % Exit if not enough data points
    end

    % Find the correct indices as per your logic
    secondOneIndex = oneIndices(2);
    nextOneIndexArray = oneIndices(oneIndices > secondOneIndex + 1);
    
    x = length(oneIndices);
    i=1;
    while (i < x)
        indexDifference = oneIndices(i+1) - oneIndices(i);
        if (indexDifference ==1)
            oneIndices = [oneIndices(1:i); oneIndices(i+2:end)];
        end
        x = length(oneIndices);
        i = i + 1;
    end

    if isempty(nextOneIndexArray)
        disp('No subsequent non-consecutive 1 found after the second occurrence.');
        return; % Exit if no subsequent non-consecutive 1 found
    else
        nextOneIndex = nextOneIndexArray(1);
    end

    % Assuming 'OrientationX', 'OrientationY', 'OrientationZ', and 'GyroY' are column names
    % Extract and store required data from concatenatedData based on identified indices

    data.timestep = concatenatedData.Var2(secondOneIndex:nextOneIndex);
    data.MPUOrientationX = concatenatedData.Var5(secondOneIndex:nextOneIndex);
    data.MPUOrientationY = concatenatedData.Var6(secondOneIndex:nextOneIndex);
    data.MPUOrientationZ = concatenatedData.Var7(secondOneIndex:nextOneIndex);
    data.MPUGyroY = concatenatedData.Var4(secondOneIndex:nextOneIndex);

    data.BNOCouplerOrientationX = concatenatedData.Var8(secondOneIndex:nextOneIndex);
    data.BNOCouplerOrientationY = concatenatedData.Var9(secondOneIndex:nextOneIndex);
    data.BNOCouplerOrientationZ = concatenatedData.Var10(secondOneIndex:nextOneIndex);
    data.BNOCouplerGyroX = concatenatedData.Var11(secondOneIndex:nextOneIndex);
    data.BNOCouplerGyroY = concatenatedData.Var12(secondOneIndex:nextOneIndex);
    data.BNOCouplerGyroZ = concatenatedData.Var13(secondOneIndex:nextOneIndex);

    data.BNORockerOrientationX = concatenatedData.Var14(secondOneIndex:nextOneIndex);
    data.BNORockerOrientationY = concatenatedData.Var15(secondOneIndex:nextOneIndex);
    data.BNORockerOrientationZ = concatenatedData.Var16(secondOneIndex:nextOneIndex);
    data.BNORockerGyroX = concatenatedData.Var17(secondOneIndex:nextOneIndex);
    data.BNORockerGyroY = concatenatedData.Var18(secondOneIndex:nextOneIndex);
    data.BNORockerGyroZ = concatenatedData.Var19(secondOneIndex:nextOneIndex);

end

function WMdata = readExperimentalWitMotionData(WitMotionExperimentalPath)
    dirPath = fullfile(WitMotionExperimentalPath); % Adjust base path as necessary
    filenames = {'10RPM.csv', '20RPM.csv', '30RPM.csv'};

    % Initialize an empty table for concatenated results
    concatenatedData = [];

    % Loop to Process Each File
    for i = 1:length(filenames)
        fullPath = fullfile(dirPath, filenames{i});
        
        % Check if the file exists before attempting to read
        if isfile(fullPath)
            % Detect import options based on the current file and sheet
            opts = detectImportOptions(fullPath, 'NumHeaderLines', 1);
            % Read the data into a table
            tempData = readtable(fullPath, opts);

            % Concatenate the new data table to the existing data
            if isempty(concatenatedData)
                concatenatedData = tempData;
            else
                concatenatedData = [concatenatedData; tempData]; % Assuming the data structure is the same across files
            end
        else
            warning('File does not exist: %s', fullPath);
        end
    end
    deviceNames = concatenatedData.Var2;
    % Find unique strings and their first occurrence index
    [uniqueStrings, ia, ~] = unique(deviceNames, 'stable');
    
    % Find indices of each unique string in the original list
    indicesForEachUniqueString = cell(length(uniqueStrings), 1);
    for i = 1:length(uniqueStrings)
        indicesForEachUniqueString{i} = find(ismember(deviceNames, uniqueStrings(i)));
    end

    table2 = indicesForEachUniqueString{2,1};
    table1 = indicesForEachUniqueString{1,1};

    a9concatenatedData = concatenatedData(table2, :);
    twelveconcatenatedData = concatenatedData(table1, :);

    for i  = 1:size(table2)
        a9Index = table2(i);
        a9data(i,:) = concatenatedData.Var11(a9Index);

    end
    % 
    % for i = 1:size(table1)
    %     twelveIndex = table1(i);
    %     twelvedata(i, :) = concatenatedData(twelveIndex);
    % end
   
    % Once this is pulled, then we want to determine the time range. Time
    % range will be defined from the first instance that the YAngle goes
    % from negative to positive to the next instance. 
    oneIndex = -1;
    secondOneIndex = -1;
    for i=1:length(a9data)
        a9current = a9data(i);
        a9next = a9data(i+1);

        if a9current <0 && a9next > 0
            if oneIndex == -1
                oneIndex = i+1;
            elseif secondOneIndex ==-1
                secondOneIndex = i+1;
            break
            end
        end

    end
    cat = 390482;
    % Afterward, we will cut the index values that are not needed. We know
    % what is not needed if the row values precede or exceed the time
    % range. 

    % Lastly, We store the desired angle and angular velocity of the sensor
    % 
     % Assuming 'OrientationX', 'OrientationY', 'OrientationZ', and 'GyroY' are column names
    % Extract and store required data from concatenatedData based on identified indices

    WMdata.WMtimestep = a9concatenatedData.Var1(oneIndex:secondOneIndex);

    WMdata.InputXAngle = a9concatenatedData.Var10(oneIndex:secondOneIndex);
    WMdata.InputYAngle = a9concatenatedData.Var11(oneIndex:secondOneIndex);
    WMdata.InputZAngle = a9concatenatedData.Var12(oneIndex:secondOneIndex);

    WMdata.InputXVel = a9concatenatedData.Var7(oneIndex:secondOneIndex);
    WMdata.InputYVel = a9concatenatedData.Var8(oneIndex:secondOneIndex);
    WMdata.InputZVel = a9concatenatedData.Var9(oneIndex:secondOneIndex);

    WMdata.OutputXAngle = twelveconcatenatedData.Var10(oneIndex:secondOneIndex);
    WMdata.OutputYAngle = twelveconcatenatedData.Var11(oneIndex:secondOneIndex);
    WMdata.OutputZAngle = twelveconcatenatedData.Var12(oneIndex:secondOneIndex);

    WMdata.OutputXVel = twelveconcatenatedData.Var7(oneIndex:secondOneIndex);
    WMdata.OutputYVel = twelveconcatenatedData.Var8(oneIndex:secondOneIndex);
    WMdata.OutputZVel = twelveconcatenatedData.Var9(oneIndex:secondOneIndex);

   

end