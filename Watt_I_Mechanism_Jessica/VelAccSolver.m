% Initialization
clear; close all; clc;


% function Mechanism = updateMechanismKinematics()
% Load the Mechanism structure
loadedData = load('Mechanism.mat', 'Mechanism');
Mechanism = loadedData.Mechanism;

% Determine the number of iterations (rows in Joints)
numIterations = size(Mechanism.Joint.A, 1); % Assuming 'A' is a joint in Mechanism.Joints

% Initialize fields for storing kinematics data
Mechanism.AngVel = struct();
Mechanism.LinVel = struct();
Mechanism.AngAcc = struct();
Mechanism.LinAcc = struct();

blankVector = [0 0 0];
initialBlankJointVector = struct('A', blankVector, ...
    'B', blankVector, 'C', blankVector, 'D', blankVector, ...
    'E', blankVector, 'F', blankVector, 'G', blankVector);

initialBlankLinkVector= struct('AB', blankVector, ...
    'BCD', blankVector, 'DE', blankVector, ...
    'EF', blankVector, 'CFG', blankVector);

[Mechanism] = initializeAngVels(Mechanism, initialBlankLinkVector, numIterations);
[Mechanism] = initializeLinVels(Mechanism, initialBlankJointVector, initialBlankLinkVector, numIterations);
[Mechanism] = initializeAngAccs(Mechanism, initialBlankLinkVector, numIterations);
[Mechanism] = initializeLinAccs(Mechanism, initialBlankJointVector, initialBlankLinkVector, numIterations);

% Iterate through all iterations
for iter = 1:numIterations
    % Extract joint positions for this iteration
    JointPos = extractJointPositions(Mechanism, iter);
    LinkCoMPos = extractLinkCoMPositions(Mechanism, iter);

    % Assuming input_speed is defined or extracted from Mechanism
    input_speed = Mechanism.inputSpeed(iter); % Placeholder, adjust based on your Mechanism structure

    % Calculate kinematics for the current iteration and store within the Mechanism 
    Mechanism = determineKinematics(Mechanism, iter, JointPos, LinkCoMPos, input_speed);
end
% end

% Save the updated Mechanism
save('Mechanism.mat', 'Mechanism');

% Define the base folder name for Velocities and Accelerations
baseVelFolder = 'Kin/Vel';
baseAccFolder = 'Kin/Acc';

% Directories for velocities
linVelJointFolder = fullfile(baseVelFolder, 'LinVel', 'Joint');
linVelLinkCoMFolder = fullfile(baseVelFolder, 'LinVel', 'LinkCoM');
angVelFolder = fullfile(baseVelFolder, 'AngVel');

% Directories for accelerations
linAccJointFolder = fullfile(baseAccFolder, 'LinAcc', 'Joint');
linAccLinkCoMFolder = fullfile(baseAccFolder, 'LinAcc', 'LinkCoM');
angAccFolder = fullfile(baseAccFolder, 'AngAcc');

% Create the directories if they don't exist
folders = {linVelJointFolder, linVelLinkCoMFolder, angVelFolder, linAccJointFolder, linAccLinkCoMFolder, angAccFolder};
for i = 1:length(folders)
    if ~exist(folders{i}, 'dir')
        mkdir(folders{i});
    end
end

% Example usage:
saveData(linVelJointFolder, Mechanism.LinVel.Joint);
saveData(linVelLinkCoMFolder, Mechanism.LinVel.LinkCoM);
saveData(angVelFolder, Mechanism.AngVel);
saveData(linAccJointFolder, Mechanism.LinAcc.Joint);
saveData(linAccLinkCoMFolder, Mechanism.LinAcc.LinkCoM);
saveData(angAccFolder, Mechanism.AngAcc);

function JointPos = extractJointPositions(Mechanism, iteration)
% Extract joint positions for a specific iteration
JointPos = struct();
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    JointPos.(jointNames{i}) = Mechanism.Joint.(jointNames{i})(iteration, :);
end
end
function LinkCoMPos = extractLinkCoMPositions(Mechanism, iteration)
% Extract link center of mass positions for a specific iteration
LinkCoMPos = struct();
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    LinkCoMPos.(linkNames{i}) = Mechanism.LinkCoM.(linkNames{i})(iteration, :);
end
end

function Mechanism = determineKinematics(Mechanism, iter, JointPos, LinkCoMPos, input_speed)
% Placeholder for your logic to calculate kinematics
[Mechanism, AngVel] = determineAngVel(Mechanism, iter, JointPos, input_speed);
[Mechanism] = determineLinVel(Mechanism, iter, JointPos, LinkCoMPos, AngVel);
[Mechanism, AngAcc] = determineAngAcc(Mechanism, iter, JointPos, AngVel);
[Mechanism] = determineLinAcc(Mechanism, iter, JointPos, LinkCoMPos, AngVel, AngAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, JointPos, input_speed)
%velocity equations from given loops
syms wBCD wDE wEF wCFG
omegaAB=[0 0 input_speed];
omegaBCD=[0 0 wBCD];
omegaDE=[0 0 wDE];
omegaEF=[0 0 wEF];
omegaCFG=[0 0 wCFG];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

% A->B->C->G->A
% V_ba + V_cb + V_gc + V_ag = 0
eqn1=velSolver(omegaAB,B-A)+velSolver(omegaBCD,C-B)+velSolver(omegaCFG,G-C)==0;

% A->B->D->E->F->G->A
% V_ba + V_ba + V_db + V_ed + V_fe + V_gf + V_ag = 0
eqn2=velSolver(omegaAB,B-A)+velSolver(omegaBCD,D-B)+velSolver(omegaDE,E-D)+velSolver(omegaEF,F-E)+velSolver(omegaCFG,G-F)==0;

solution=solve([eqn1, eqn2],[wBCD wDE wEF wCFG]);

% Store all the determined angular velocities
AngVel.AB=[0 0 input_speed];
AngVel.BCD=[0 0 double(solution.wBCD)]; %angular velocity of BCD
AngVel.DE=[0 0 double(solution.wDE)]; %angular velocity of DE
AngVel.EF=[0 0 double(solution.wEF)]; %angular velocity of EF
AngVel.CFG=[0 0 double(solution.wCFG)]; %angular velocity of CFG

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngVel.(linkNames{i})(iter,:) = AngVel.(linkNames{i});
end
end
function [Mechanism] = determineLinVel(Mechanism, iter, JointPos, LinkCoMPos, AngVel)
% Determine the velocities at each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

AB_com = LinkCoMPos.AB;
BCD_com = LinkCoMPos.BCD;
DE_com = LinkCoMPos.DE;
EF_com = LinkCoMPos.EF;
CFG_com = LinkCoMPos.CFG;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = velSolver(AngVel.AB,B-A);
LinVel.Joint.C = velSolver(AngVel.BCD,C-B) + LinVel.Joint.B;
LinVel.Joint.D = velSolver(AngVel.BCD,D-B) + LinVel.Joint.B;
LinVel.Joint.E = velSolver(AngVel.DE,E-D) + LinVel.Joint.D;
LinVel.Joint.F = velSolver(AngVel.CFG,F-G);
LinVel.Joint.G = [0 0 0];

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.AB = velSolver(AngVel.AB,AB_com - A);
LinVel.LinkCoM.BCD= velSolver(AngVel.BCD,BCD_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.DE= velSolver(AngVel.DE,DE_com - D) + LinVel.Joint.D;
LinVel.LinkCoM.EF = velSolver(AngVel.EF,EF_com - E) + LinVel.Joint.E;
LinVel.LinkCoM.CFG = velSolver(AngVel.CFG,CFG_com - G);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinVel.Joint.(jointNames{i})(iter,:) = LinVel.Joint.(jointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinVel.LinkCoM.(linkNames{i})(iter,:) = LinVel.LinkCoM.(linkNames{i});
end

end
function [Mechanism, AngAcc] = determineAngAcc(Mechanism, iter, Pos, AngVel)
A = Pos.A;
B = Pos.B;
C = Pos.C;
D = Pos.D;
E = Pos.E;
F = Pos.F;
G = Pos.G;
%% Acceleration loops
%acceleration equations from given loops
syms aBCD aDE aEF aCFG
alphaAB=[0 0 0];
alphaBCD=[0 0 aBCD];
alphaDE=[0 0 aDE];
alphaEF=[0 0 aEF];
alphaCFG=[0 0 aCFG];

% A->B->C->G->A
% A_ba + A_cb + A_gc + A_ag = 0
eqn1=accSolver(AngVel.AB,alphaAB, B-A)+accSolver(AngVel.BCD,alphaBCD,C-B)+accSolver(AngVel.CFG,alphaCFG,G-C)==0;

% A->B->D->E->F->G->A
% A_ba + A_ba + A_db + A_ed + A_fe + A_gf + A_ag = 0
eqn2=accSolver(AngVel.AB,alphaAB,B-A)+accSolver(AngVel.BCD,alphaBCD,D-B)+accSolver(AngVel.DE,alphaDE,E-D)+accSolver(AngVel.EF,alphaEF,F-E)+accSolver(AngVel.CFG,alphaCFG,G-F)==0;

solution=solve([eqn1, eqn2],[aBCD aDE aEF aCFG]);

% Store all the determined angular accelerations
AngAcc.AB=[0 0 0];
AngAcc.BCD=[0 0 double(solution.aBCD)]; %angular acceleration of BCE
AngAcc.DE=[0 0 double(solution.aDE)]; %angular acceleration of CD
AngAcc.EF=[0 0 double(solution.aEF)]; %angular acceleration of EF
AngAcc.CFG=[0 0 double(solution.aCFG)]; %angular acceleration of FG

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngAcc.(linkNames{i})(iter,:) = AngAcc.(linkNames{i});
end
end
function [Mechanism] = determineLinAcc(Mechanism, iter, JointPos, LinkCoMPos, AngVel, AngAcc)
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

AB_com = LinkCoMPos.AB;
BCD_com = LinkCoMPos.BCD;
DE_com = LinkCoMPos.DE;
EF_com = LinkCoMPos.EF;
CFG_com = LinkCoMPos.CFG;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = accSolver(AngVel.AB, AngAcc.AB,B-A);
LinAcc.Joint.C = accSolver(AngVel.BCD, AngAcc.BCD,C-B) + LinAcc.Joint.B;
LinAcc.Joint.D = accSolver(AngVel.BCD, AngAcc.BCD,D-B) + LinAcc.Joint.B;
LinAcc.Joint.E = accSolver(AngVel.DE, AngAcc.DE,E-D) + LinAcc.Joint.D;
LinAcc.Joint.F = accSolver(AngVel.CFG,AngAcc.CFG, F-G);
LinAcc.Joint.G = [0 0 0];

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.AB = accSolver(AngVel.AB,AngAcc.AB,AB_com - A);
LinAcc.LinkCoM.BCD= accSolver(AngVel.BCD,AngAcc.BCD,BCD_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.DE= accSolver(AngVel.DE,AngAcc.DE,DE_com - D) + LinAcc.Joint.D;
LinAcc.LinkCoM.EF= accSolver(AngVel.EF,AngAcc.EF,EF_com - E) + LinAcc.Joint.E;
LinAcc.LinkCoM.CFG = accSolver(AngVel.CFG,AngAcc.CFG,CFG_com - G);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i})(iter,:) = LinAcc.Joint.(jointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinAcc.LinkCoM.(linkNames{i})(iter,:) = LinAcc.LinkCoM.(linkNames{i});
end
end

% Desired functions to do cross products appropriately
function vel = velSolver(w, r)
vel = cross(w,r);
end
function acc = accSolver(w,a,r)
acc = cross(w,cross(w,r)) + cross(a,r);
end

% Initialize AngVel, LinVel, AngAcc, LinAcc
function Mechanism = initializeAngVels(Mechanism, initialBlankLinkVector, max_iterations)
    angVelNames = fieldnames(initialBlankLinkVector);
    for i = 1:length(angVelNames)
        Mechanism.AngVel.(angVelNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
end
function Mechanism = initializeLinVels(Mechanism, initialBlankJointVector, initialBlankLinkVector, max_iterations)
    linJointVelNames = fieldnames(initialBlankJointVector);
    for i = 1:length(linJointVelNames)
        Mechanism.LinVel.Joint.(linJointVelNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
    linLinkCoMVelNames = fieldnames(initialBlankLinkVector);
    for i = 1:length(linLinkCoMVelNames)
        Mechanism.LinVel.LinkCoM.(linLinkCoMVelNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
end
function Mechanism = initializeAngAccs(Mechanism, initialBlankLinkVector, max_iterations)
    angAccNames = fieldnames(initialBlankLinkVector);
    for i = 1:length(angAccNames)
        Mechanism.AngAcc.(angAccNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
end
function Mechanism = initializeLinAccs(Mechanism, initialBlankJointVector, initialBlankLinkVector, max_iterations)
    linJointNames = fieldnames(initialBlankJointVector);
    for i = 1:length(linJointNames)
        Mechanism.LinAcc.Joint.(linJointNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
    linLinkCoMAccNames = fieldnames(initialBlankLinkVector);
    for i = 1:length(linLinkCoMAccNames)
        Mechanism.LinAcc.LinkCoM.(linLinkCoMAccNames{i}) = zeros(max_iterations, 3); % Initialize with zeros for each dimension (assuming 3D angular velocities)
    end
end

% Save function for clarity and reusability
function saveData(folder, dataStruct)
    % Ensure the folder exists
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    
    names = fieldnames(dataStruct); % Get all field names of the structure
    for i = 1:length(names)
        name = names{i};
        data = dataStruct.(name); % Extract data
        
        % Create a temporary struct with the extracted data
        tempStruct = struct(name, data);
        
        % Correctly use the -struct option by providing the variable name of the temporary struct
        save(fullfile(folder, name), '-struct', 'tempStruct', name);
    end
end