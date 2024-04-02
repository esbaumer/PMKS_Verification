clear; close all; clc;

% Define the base path for each type of data
baseTheoreticalPosPath = 'CSVOutput/Pos/Joint/';
baseTheoreticalVelPath = 'CSVOutput/Vel/AngVel/';
baseTheoreticalAccPath = 'CSVOutput/Acc/AngAcc/';

baseExperimentalPath = 'Experimental/';


% Read the desired theoretical files to analyze
theoreticalJointData = readTheoreticalJointData(baseTheoreticalPosPath);
theoreticalAngleData = determineAngles(theoreticalJointData); % Determine the angles based on the passed in joint data
theoreticalAngVelData = readTheoreticalLinkData(baseTheoreticalVelPath);
theoreticalAngAccData = readTheoreticalLinkData(baseTheoreticalAccPath);

% Read the desired experimental files to analyze
experimentalData = readExerimentalData(baseExperimentalPath);
experimentalAngleData = experimentalData.Angle;
experimentalAngVelData = experimentalData.AngVel;
experimentalAngAccData = experimentalData.AngAcc;

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
    numberOfRows = size(jointData(1).data, 1);
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
    for rowNum = 1:6
        % Determine the offset based on jointID. Example adjustments:
        if theta_iterator == 1 % For joint A with respect to another joint
            % Pull the appropriate joint values 
            A = table2array(jointData(jointIndexMap('A')).data(theta_iterator,:));
            B = table2array(jointData(jointIndexMap('B')).data(theta_iterator,:));
            angle = atan2(B(2) - A(2), B(1) - A(1));
            % TODO: Do this process for all desired joint positions
            adjustedAngles(rowNum, theta_iterator) = 180 - angle;
        elseif theta_iterator == 2 % For joint B
            % Adjust angle based on your criteria for joint B
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
