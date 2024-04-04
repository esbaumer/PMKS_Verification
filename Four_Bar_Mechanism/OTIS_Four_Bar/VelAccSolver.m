function Mechanism = VelAccSolver(Mechanism)

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
    'E', blankVector, 'F', blankVector, 'G', blankVector, 'H', blankVector);

initialBlankLinkVector= struct('ABE', blankVector, ...
    'BCFG', blankVector, 'CDH', blankVector);

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

end

function JointPos = extractJointPositions(Mechanism, iteration)
% Extract joint positions for a specific iteration
JointPos = struct();
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    JointPos.(jointNames{i}) = Mechanism.Joint.(jointNames{i})(iteration, :);
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    JointPos.(tracerPointNames{i}) = Mechanism.TracerPoint.(tracerPointNames{i})(iteration, :);
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
syms wBCFG wCDH 
omegaABE=[0 0 input_speed];
omegaBCFG=[0 0 wBCFG];
omegaCDH=[0 0 wCDH];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=velSolver(omegaABE,B-A)+velSolver(omegaBCFG,C-B)+velSolver(omegaCDH,D-C)==0;

solution=solve(eqn1,[wBCFG wCDH]);

% Store all the determined angular velocities
AngVel.ABE=[0 0 input_speed];
AngVel.BCFG=[0 0 double(solution.wBCFG)]; %angular velocity of BCFG
AngVel.CDH=[0 0 double(solution.wCDH)]; %angular velocity of DE

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
H = JointPos.H;

ABE_com = LinkCoMPos.ABE;
BCFG_com = LinkCoMPos.BCFG;
CDH_com = LinkCoMPos.CDH;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = velSolver(AngVel.ABE,B-A);
LinVel.Joint.C = velSolver(AngVel.CDH,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = velSolver(AngVel.ABE,E-A);
LinVel.Joint.F = velSolver(AngVel.BCFG,F-C) + LinVel.Joint.C;
LinVel.Joint.G = velSolver(AngVel.BCFG,G-C) + LinVel.Joint.C;
LinVel.Joint.H = velSolver(AngVel.CDH,H-D);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.ABE = velSolver(AngVel.ABE,ABE_com - A);
LinVel.LinkCoM.BCFG= velSolver(AngVel.BCFG,BCFG_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDH= velSolver(AngVel.CDH,CDH_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinVel.Joint.(jointNames{i})(iter,:) = LinVel.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinVel.Joint.(tracerPointNames{i})(iter,:) = LinVel.Joint.(tracerPointNames{i});
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

%% Acceleration loops
%acceleration equations from given loops
syms aBCFG aCDH
alphaABE=[0 0 0];
alphaBCFG=[0 0 aBCFG];
alphaCDH=[0 0 aCDH];

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=accSolver(AngVel.ABE,alphaABE, B-A)+accSolver(AngVel.BCFG,alphaBCFG,C-B)+accSolver(AngVel.CDH,alphaCDH,D-C)==0;

solution=solve(eqn1,[aBCFG aCDH]);

% Store all the determined angular accelerations
AngAcc.ABE=[0 0 0];
AngAcc.BCFG=[0 0 double(solution.aBCFG)]; %angular acceleration of BCFG
AngAcc.CDH=[0 0 double(solution.aCDH)]; %angular acceleration of CDH

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
H = JointPos.H;

ABE_com = LinkCoMPos.ABE;
BCFG_com = LinkCoMPos.BCFG;
CDH_com = LinkCoMPos.CDH;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = accSolver(AngVel.ABE, AngAcc.ABE,B-A);
LinAcc.Joint.C = accSolver(AngVel.CDH, AngAcc.CDH,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = accSolver(AngVel.ABE, AngAcc.ABE,E-A);
LinAcc.Joint.F = accSolver(AngVel.BCFG, AngAcc.BCFG,F-B) + LinAcc.Joint.B;
LinAcc.Joint.G = accSolver(AngVel.BCFG, AngAcc.BCFG,G-B) + LinAcc.Joint.B;
LinAcc.Joint.H = accSolver(AngVel.CDH, AngAcc.BCFG,H-D);

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.ABE = accSolver(AngVel.ABE,AngAcc.ABE,ABE_com - A);
LinAcc.LinkCoM.BCFG= accSolver(AngVel.BCFG,AngAcc.BCFG,BCFG_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDH= accSolver(AngVel.CDH,AngAcc.CDH,CDH_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i})(iter,:) = LinAcc.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinAcc.Joint.(tracerPointNames{i})(iter,:) = LinAcc.Joint.(tracerPointNames{i});
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
