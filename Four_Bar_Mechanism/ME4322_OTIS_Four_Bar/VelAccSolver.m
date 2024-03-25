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
    'E', blankVector, 'F', blankVector, 'G', blankVector);

initialBlankLinkVector= struct('AB', blankVector, ...
    'BCE', blankVector, 'CD', blankVector, ...
    'EF', blankVector, 'FG', blankVector);

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
syms wBCE wCD wEF wFG
omegaAB=[0 0 input_speed];
omegaBCE=[0 0 wBCE];
omegaCD=[0 0 wCD];
omegaEF=[0 0 wEF];
omegaFG=[0 0 wFG];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=velSolver(omegaAB,B-A)+velSolver(omegaBCE,C-B)+velSolver(omegaCD,D-C)==0;

% A->B->E->F->G->A
% V_ba + V_eb + V_fe + V_gf + V_ag = 0
eqn2=velSolver(omegaAB,B-A)+velSolver(omegaBCE,E-B)+velSolver(omegaEF,F-E)+velSolver(omegaFG,G-F)==0;

solution=solve([eqn1, eqn2],[wBCE wCD wEF wFG]);

% Store all the determined angular velocities
AngVel.AB=[0 0 input_speed];
AngVel.BCE=[0 0 double(solution.wBCE)]; %angular velocity of BCE
AngVel.CD=[0 0 double(solution.wCD)]; %angular velocity of DE
AngVel.EF=[0 0 double(solution.wEF)]; %angular velocity of EF
AngVel.FG=[0 0 double(solution.wFG)]; %angular velocity of CFG

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
BCE_com = LinkCoMPos.BCE;
CD_com = LinkCoMPos.CD;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = velSolver(AngVel.AB,B-A);
LinVel.Joint.C = velSolver(AngVel.CD,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = velSolver(AngVel.BCE,E-C) + LinVel.Joint.C;
LinVel.Joint.F = velSolver(AngVel.FG,F-G);
LinVel.Joint.G = [0 0 0];

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.AB = velSolver(AngVel.AB,AB_com - A);
LinVel.LinkCoM.BCE= velSolver(AngVel.BCE,BCE_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CD= velSolver(AngVel.CD,CD_com - D);
LinVel.LinkCoM.EF = velSolver(AngVel.EF,EF_com - E) + LinVel.Joint.E;
LinVel.LinkCoM.FG = velSolver(AngVel.FG,FG_com - G);

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
syms aBCE aCD aEF aFG
alphaAB=[0 0 0];
alphaBCE=[0 0 aBCE];
alphaCD=[0 0 aCD];
alphaEF=[0 0 aEF];
alphaFG=[0 0 aFG];

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=accSolver(AngVel.AB,alphaAB, B-A)+accSolver(AngVel.BCE,alphaBCE,C-B)+accSolver(AngVel.CD,alphaCD,D-C)==0;

% A->B->D->E->F->G->A
% A_ba + A_ba + A_db + A_ed + A_fe + A_gf + A_ag = 0
eqn2=accSolver(AngVel.AB,alphaAB,B-A)+accSolver(AngVel.BCE,alphaBCE,E-B)+accSolver(AngVel.EF,alphaEF,F-E)+accSolver(AngVel.FG,alphaFG,G-F)==0;

solution=solve([eqn1, eqn2],[aBCE aCD aEF aFG]);

% Store all the determined angular accelerations
AngAcc.AB=[0 0 0];
AngAcc.BCE=[0 0 double(solution.aBCE)]; %angular acceleration of BCE
AngAcc.CD=[0 0 double(solution.aCD)]; %angular acceleration of CD
AngAcc.EF=[0 0 double(solution.aEF)]; %angular acceleration of EF
AngAcc.FG=[0 0 double(solution.aFG)]; %angular acceleration of FG

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
BCE_com = LinkCoMPos.BCE;
CD_com = LinkCoMPos.CD;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = accSolver(AngVel.AB, AngAcc.AB,B-A);
LinAcc.Joint.C = accSolver(AngVel.CD, AngAcc.CD,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = accSolver(AngVel.BCE, AngAcc.BCE,E-B) + LinAcc.Joint.B;
LinAcc.Joint.F = accSolver(AngVel.FG,AngAcc.FG, F-G);
LinAcc.Joint.G = [0 0 0];

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.AB = accSolver(AngVel.AB,AngAcc.AB,AB_com - A);
LinAcc.LinkCoM.BCE= accSolver(AngVel.BCE,AngAcc.BCE,BCE_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CD= accSolver(AngVel.CD,AngAcc.CD,CD_com - D);
LinAcc.LinkCoM.EF= accSolver(AngVel.EF,AngAcc.EF,EF_com - E) + LinAcc.Joint.E;
LinAcc.LinkCoM.FG = accSolver(AngVel.FG,AngAcc.FG,FG_com - G);

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
