function Mechanism = ForceSolver(Mechanism)

% Assuming numIterations is defined by the size of an array in Mechanism
numIterations = size(Mechanism.Joint.A, 1);

% Initialize fields for storing static analysis data
[Mechanism] = initializeForceSolvers(Mechanism, numIterations);

% Iterate through all iterations for static analysis
for iter = 1:numIterations
    % Extract joint and link center of mass positions for this iteration
    JointPos = extractJointPositions(Mechanism, iter);
    LinkCoMPos = extractLinkCoMPositions(Mechanism, iter);

    % Scenarios: [newtonFlag, gravityFlag, frictionFlag]
    % scenarios = [0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1];
    scenarios = [1 1 0];

    for scenario = scenarios.'
        Mechanism = updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, scenario(1), scenario(2), scenario(3));
    end
end

% Save the updated Mechanism with static analysis results
save('Mechanism.mat', 'Mechanism');

baseFolder = 'Force';
% Save Force Data
saveForceData(baseFolder, Mechanism);
end

function Mechanism = updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, newtonFlag, gravityFlag, frictionFlag)
% Define the suffix based on the provided flags for readability
suffix = '';
if newtonFlag
    suffix = [suffix, 'NewtonForce'];
else
    suffix = [suffix, 'StaticForce'];
end
if gravityFlag
    suffix = [suffix, 'Grav'];
else
    suffix = [suffix, 'NoGrav'];
end
if frictionFlag
    suffix = [suffix, 'Friction'];
else
    suffix = [suffix, 'NoFriction'];
end

% Perform force analysis
solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, newtonFlag, gravityFlag, frictionFlag);
jointNames = fieldnames(Mechanism.Joint);

% Update forces and torques in the mechanism structure
for i = 1:length(jointNames)
    jointName = jointNames{i};
    Mechanism.(suffix).Joint.(jointName)(iter, :) = [double(solution.([jointName, 'x'])), double(solution.([jointName, 'y'])), 0];
end
Mechanism.(suffix).Torque(iter,:) = [0 0 double(solution.T)];
end


function [Mechanism] = initializeForceSolvers(Mechanism, numIterations)
% Initialize with zeros for storing forces and moments
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.StaticForceGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.StaticForceNoGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceNoGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.StaticForceGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.StaticForceNoGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceNoGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
end
Mechanism.StaticForceGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceNoGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceNoGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceNoGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceNoGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces

% Only for the slider will we need to include the normal force
Mechanism.StaticForceGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceNoGravNoFriction.NormalForce= zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceNoGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceNoGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceNoGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
end

function solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, newton, grav, friction)
% Pull the mass of each component
massABE = Mechanism.Mass.ABE;
massBCFG = Mechanism.Mass.BCFG;
massCDH = Mechanism.Mass.CDH;

% Pull the mass moment of inertia of each component
massMoIABE = Mechanism.MassMoI.ABE;
massMoIBCFG = Mechanism.MassMoI.BCFG;
massMoICDH = Mechanism.MassMoI.CDH;

% Pull the angular acceleration of each link
A_abe = Mechanism.AngAcc.ABE(iter,:);
A_bcfg = Mechanism.AngAcc.BCFG(iter,:);
A_cdh = Mechanism.AngAcc.CDH(iter,:);

% Pull the acceleration of each link at its center of mass
A_abe_com = Mechanism.LinAcc.LinkCoM.ABE(iter,:);
A_bcfg_com = Mechanism.LinAcc.LinkCoM.BCFG(iter,:);
A_cdh_com = Mechanism.LinAcc.LinkCoM.CDH(iter,:);

% Extract positions for each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

mu = 0.20;

% Extract positions for each link's center of mass
ABE_com = LinkCoMPos.ABE;
BCFG_com = LinkCoMPos.BCFG;
CDH_com = LinkCoMPos.CDH;

% Define all the unknown variables to solve for
syms Ax Ay Bx By Cx Cy Dx Dy T

%defining gravity to find weight of each link m/s^2
g = [0 -9.81 0];

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];

% Weight of each link
wABE=massABE*g*grav;
wBCFG=massBCFG*g*grav;
wCDH=massCDH*g*grav;

% Unknown torque of the system
tT=[0 0 T];

% Torque provided by friction on Joint A
r_abe_com_a = norm(ABE_com - A);
r_bcfg_com_b = norm(BCFG_com - B);
r_cdh_com_c = norm(CDH_com - C);

A_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.A(iter,1);
A_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.A(iter,2);
B_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.B(iter,1); 
B_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.B(iter,2);
C_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.C(iter,1); 
C_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.C(iter,2);

% Correcting angle calculation
A_theta = atan2(A_noFriction_y, A_noFriction_x);
B_theta = atan2(B_noFriction_y, B_noFriction_x);
C_theta = atan2(C_noFriction_y, C_noFriction_x);

A_friction_Mag = norm([A_noFriction_x, A_noFriction_y]);
B_friction_Mag = norm([B_noFriction_x, B_noFriction_y]);
C_friction_Mag = norm([C_noFriction_x, C_noFriction_y]);

% Correct normal force direction
F_normal_A = [A_friction_Mag*cos(A_theta), A_friction_Mag*sin(A_theta), 0];
F_normal_B = [B_friction_Mag*cos(B_theta), B_friction_Mag*sin(B_theta), 0];
F_normal_C = [C_friction_Mag*cos(C_theta), C_friction_Mag*sin(C_theta), 0];

% Friction forces (assuming directions are appropriately chosen)
F_friction_A = mu * norm(F_normal_A) * [-sin(A_theta), cos(A_theta), 0] * friction; % Perpendicular to normal force
F_friction_B = mu * norm(F_normal_B) * [-sin(B_theta), cos(B_theta), 0] * friction; % Perpendicular to normal force
F_friction_C = mu * norm(F_normal_C) * [-sin(C_theta), cos(C_theta), 0] * friction; % Perpendicular to normal force

% Assuming r_abe_com_a and r_bcfg_com_b are correctly calculated lever arms
T_fr_A = [0,0, mu * norm(F_normal_A) * r_abe_com_a * friction]; % Torque due to friction at A
T_fr_B = [0,0, mu * norm(F_normal_B) * r_bcfg_com_b * friction]; % Torque due to friction at B
T_fr_C = [0,0, mu * norm(F_normal_B) * r_cdh_com_c * friction]; % Torque due to friction at B

%% FBD Equations
%Link ABE
eqn1=fA+fB+wABE+F_friction_A+F_friction_B==massABE*A_abe_com*newton;
eqn2=momentVec(A, ABE_com, fA) + momentVec(B,  ABE_com,fB)+tT+T_fr_A+T_fr_B==massMoIABE * A_abe*newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCFG
eqn3=-fB+fC+wBCFG-F_friction_B+F_friction_C==massBCFG*A_bcfg_com*newton;
eqn4=momentVec(B, BCFG_com, -fB)+momentVec(C, BCFG_com, fC)-T_fr_B+T_fr_C==massMoIBCFG * A_bcfg*newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDH
eqn5=-fC+fD+wCDH-F_friction_C==massCDH*A_cdh_com*newton;
eqn6=momentVec(C, CDH_com, -fC)+momentVec(D, CDH_com, fD)-T_fr_C==massMoICDH * A_cdh*newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,T]));
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

function pos = momentVec(pos, fixPos, force)
% Position Vector
r = pos - fixPos;
pos = cross(r,force);
end

function saveForceData(baseFolder, Mechanism)
% Define categories, conditions, and friction states
categories = {'Static', 'Newton'};
conditions = {'Grav', 'NoGrav'};
frictions = {'Friction', 'NoFriction'};

% Iterate through each combination of categories, conditions, and frictions
for iCategory = 1:length(categories)
    for iCondition = 1:length(conditions)
        for iFriction = 1:length(frictions)
            category = categories{iCategory};
            condition = conditions{iCondition};
            friction = frictions{iFriction};

            % Construct force field name e.g., StaticForceGravFriction
            forceFieldName = [category 'Force' condition friction];

            % Prepare folders for Joint and Torque
            jointFolder = fullfile(baseFolder, [category 'Force'], condition, friction, 'Joint');

            % Ensure folders exist
            if ~exist(jointFolder, 'dir')
                mkdir(jointFolder);
            end

            % Process and save Joint data
            if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Joint')
                jointNames = fieldnames(Mechanism.(forceFieldName).Joint);
                for iJoint = 1:length(jointNames)
                    jointName = jointNames{iJoint};
                    tempStruct = struct(jointName, Mechanism.(forceFieldName).Joint.(jointName));
                    save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
                end
            end

            % Process and save Torque data
            if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Torque')
                torqueFilePath = fullfile(baseFolder, [category 'Force'], condition, friction, 'Torque.mat');
                Torque = Mechanism.(forceFieldName).Torque;
                save(torqueFilePath, 'Torque');
            end

            % Process and save Normal Force data
            if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'NormalForce')
                normalForceFilePath = fullfile(baseFolder, [category 'Force'], condition, friction, 'NormalForce.mat');
                NormalForce = Mechanism.(forceFieldName).NormalForce; % Extract normal force data
                save(normalForceFilePath, 'NormalForce');
            end
        end
    end
end
end