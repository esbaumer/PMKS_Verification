clear;
close all;
clc;

%% TODO: Once all the helper functions have been created, I will utilize this to create files
function generateStaticTable()
end
function generateNewtonTable()
end
function generateKinematicsTable()
    jointNames = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};
    linkNames = {'AB', 'BCD', 'DE', 'EF', 'CFG'};
    
    % Preallocate cell arrays to store joint kinematics
    jointPositions = cell(1, numel(jointNames) * index);
    jointVelocities = cell(1, numel(jointNames) * index);
    jointAccelerations = cell(1, numel(jointNames) *index);

    linkPositions =  cell(1, numel(linkNames) * index);
    linkVelocities =  cell(1, numel(linkNames) * index);
    linkAccelerations =  cell(1, numel(linkNames) * index);

    angularPositions =  cell(1, numel(linkNames) * index);
    angularVelocities =  cell(1, numel(linkNames) * index);
    angularAccelerations =  cell(1, numel(linkNames) * index);
    
    % Get joint kinematics for each joint
    for i = 1:index
        for j = 1:numel(jointNames)
            jointPositions{j + (i - 1)*numel(jointNames)} = Kin.timestep(i).pos.(jointNames{j});
            jointVelocities{j + (i - 1)*numel(jointNames)} = Kin.timestep(i).v.joint.(jointNames{j});
            jointAccelerations{j + (i - 1)*numel(jointNames)} = Kin.timestep(i).a.joint.(jointNames{j});
        end
        
        for l = 1:numel(linkNames)
            linkPositions{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).pos.CoM.(linkNames{l});
            linkVelocities{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).v.link.(linkNames{l});
            linkAccelerations{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).a.link.(linkNames{l});
            angularPositions{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).pos.CoM.(linkNames{l});
            angularVelocities{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).w.(linkNames{l});
            angularAccelerations{l + (i - 1)*numel(linkNames)} = Kin.timestep(i).alpha.(linkNames{l});
        end
    end

    % Create empty tables for each section
    jointTable = createJointTable(jointPositions, jointVelocities, jointAccelerations, jointNames);
    linkTable = createLinkTable(linkPositions, linkVelocities, linkAccelerations, linkNames);
    angularTable = createAngularTable(angularPositions, angularVelocities, angularAccelerations, linkNames);

    % Combine the tables into a single table
    kinematicsTable = combineTables(jointTable, linkTable, angularTable);

    % Save the table as a CSV file
    currDir = pwd;
    directory = currDir + Downloads';  % Specify the directory where you want to save the table
    filename = 'kinematics_table.csv'; % Specify the filename for the table
    fullPath = fullfile(directory, filename); % Create the full path to the file
    writetable(kinematicsTable, fullPath);
end

function jointTable = createJointTable()
end

function jointPosTable = createJointPosTable(jointPositions, jointVelocities, jointAccelerations, jointNames)
% Create an empty table for joint kinematics

% Set column names for the joint table
columnNames = cell(1, numel(jointNames) * 2 * 3); % 3 variables (pos, vel, acc) for each joint, 2 directions (X, Y)
count = 1;
for i = 1:numel(jointNames)
    columnNames{count} = strcat('Joint ', jointNames{i}, ' X');
    columnNames{count+1} = strcat('Joint ', jointNames{i}, ' Y');
    columnNames{count+2} = strcat('Joint ', jointNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Joint ', jointNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Joint ', jointNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Joint ', jointNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(jointPositions) / numel(jointNames));

jointTable = table();

for i = 1:numel(columnNames)
    jointTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for j = 1:numel(jointNames)
        % Get joint kinematic information at the current timestep
        jointPos = jointPositions{j + (t - 1)* numel(jointNames)}(:);
        jointVel = jointVelocities{j + (t - 1)* numel(jointNames)};
        jointAcc = jointAccelerations{j + (t - 1)* numel(jointNames)};
        % Add joint kinematic information to the row
        row = [row; jointPos(1); jointPos(2); jointVel.x; jointVel.y; jointAcc.x; jointAcc.y];
    end
    % Add the row to the joint table
    jointTable(t, :) = sym2cell(row');
    
end
end
function jointVelTable = createJointVelTable()
end
function jointAccTable = createJointAccTable()
end

function linkTable = createLinkTable()
end

function linkLinPosTable = createLinkLinPosTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics

% Set column names for the link table
columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Link ', linkNames{i}, ' X');
    columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
    columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(linkPositions) / numel(linkNames));

linkTable = table();

for i = 1:numel(columnNames)
    linkTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for l = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
        linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
        linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
    end
    % Add the row to the joint table
    linkTable(t, :) = sym2cell(row');
end
end
function linkLinVelTable = createLinkLinVelTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics

% Set column names for the link table
columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Link ', linkNames{i}, ' X');
    columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
    columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(linkPositions) / numel(linkNames));

linkTable = table();

for i = 1:numel(columnNames)
    linkTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for l = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
        linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
        linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
    end
    % Add the row to the joint table
    linkTable(t, :) = sym2cell(row');
end
end
function linkLinAccTable = createLinkLinAccTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics

% Set column names for the link table
columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Link ', linkNames{i}, ' X');
    columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
    columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(linkPositions) / numel(linkNames));

linkTable = table();

for i = 1:numel(columnNames)
    linkTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for l = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
        linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
        linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
    end
    % Add the row to the joint table
    linkTable(t, :) = sym2cell(row');
end
end

% function linkAngPosTable = createLinkPosTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics
% 
% Set column names for the link table
% columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
% count = 1;
% for i = 1:numel(linkNames)
%     columnNames{count} = strcat('Link ', linkNames{i}, ' X');
%     columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
%     columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
%     columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
%     columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
%     columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
%     count = count + 6;
% end
% 
% Iterate over timesteps
% numTimesteps = (numel(linkPositions) / numel(linkNames));
% 
% linkTable = table();
% 
% for i = 1:numel(columnNames)
%     linkTable.(columnNames{i}) = 0;
% end
% 
% for t = 1:numTimesteps
%     Create a row for the current timestep
%     row = {};
%     for l = 1:numel(linkNames)
%         Get joint kinematic information at the current timestep
%         linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
%         linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
%         linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
%         Add joint kinematic information to the row
%         row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
%     end
%     Add the row to the joint table
%     linkTable(t, :) = sym2cell(row');
% end
% end
function linkAngVelTable = createLinkAngVelTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics

% Set column names for the link table
columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Link ', linkNames{i}, ' X');
    columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
    columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(linkPositions) / numel(linkNames));

linkTable = table();

for i = 1:numel(columnNames)
    linkTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for l = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
        linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
        linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
    end
    % Add the row to the joint table
    linkTable(t, :) = sym2cell(row');
end
end
function linkAngAccTable = createLinkAngAccTable(linkPositions, linkVelocities, linkAccelerations, linkNames)
% Create an empty table for link kinematics

% Set column names for the link table
columnNames = cell(1, numel(linkNames) * 2 * 3); % 3 variables (pos, vel, acc) for each link, 2 directions (X, Y)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Link ', linkNames{i}, ' X');
    columnNames{count+1} = strcat('Link ', linkNames{i}, ' Y');
    columnNames{count+2} = strcat('Link ', linkNames{i}, ' Velocity X');
    columnNames{count+3} = strcat('Link ', linkNames{i}, ' Velocity Y');
    columnNames{count+4} = strcat('Link ', linkNames{i}, ' Acceleration X');
    columnNames{count+5} = strcat('Link ', linkNames{i}, ' Acceleration Y');
    count = count + 6;
end

% Iterate over timesteps
numTimesteps = (numel(linkPositions) / numel(linkNames));

linkTable = table();

for i = 1:numel(columnNames)
    linkTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for l = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        linkPos = linkPositions{l + (t - 1)* numel(linkNames)}(:);
        linkVel = linkVelocities{l + (t - 1)* numel(linkNames)};
        linkAcc = linkAccelerations{l + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; linkPos(1); linkPos(2); linkVel(1); linkVel(2); linkAcc(1); linkAcc(2)];
    end
    % Add the row to the joint table
    linkTable(t, :) = sym2cell(row');
end
end

function angTable = createAngularTable(angularPositions, angularVelocities, angularAccelerations, linkNames)
% Create an empty table for joint kinematics

% Set column names for the joint table
columnNames = cell(1, numel(linkNames) * 1 * 3); % 3 variables (pos, vel, acc) for each joint, 1 directions (Z)
count = 1;
for i = 1:numel(linkNames)
    columnNames{count} = strcat('Angle ', linkNames{i});
    columnNames{count+1} = strcat('Angular Velocity ', linkNames{i});
    columnNames{count+2} = strcat('Angular Acceleration ', linkNames{i});
    count = count + 3;
end

% Iterate over timesteps
numTimesteps = (numel(angularPositions) / numel(linkNames));

angTable = table();

for i = 1:numel(columnNames)
    angTable.(columnNames{i}) = 0;
end

for t = 1:numTimesteps
    % Create a row for the current timestep
    row = {};
    for a = 1:numel(linkNames)
        % Get joint kinematic information at the current timestep
        ang = angularPositions{a + (t - 1)* numel(linkNames)};
        angVel = angularVelocities{a + (t - 1)* numel(linkNames)};
        angAcc = angularAccelerations{a + (t - 1)* numel(linkNames)};
        % Add joint kinematic information to the row
        row = [row; ang(3); angVel.z; angAcc.z];
    end
    % Add the row to the joint table
    angTable(t, :) = row';
end
end

function combinedTable = combineTables(jointTable, linkTable, angularTable)
    % Combine the joint, link, and angular tables into a single table
    % Check if any of the tables is empty
    filler = table(cell(height(jointTable), 1), 'VariableNames',{' '});
    filler2 = table(cell(height(jointTable), 1), 'VariableNames',{'  '});
    if isempty(jointTable)
        combinedTable = linkTable;
        if ~isempty(angularTable)
            combinedTable = [combinedTable, filler, angularTable];
        end
    elseif isempty(linkTable)
        combinedTable = jointTable;
        if ~isempty(angularTable)
            combinedTable = [combinedTable, filler, angularTable];
        end
    elseif isempty(angularTable)
        combinedTable = [jointTable, filler, linkTable];
    else
        combinedTable = [jointTable, filler, linkTable, filler2, angularTable];
    end
end
