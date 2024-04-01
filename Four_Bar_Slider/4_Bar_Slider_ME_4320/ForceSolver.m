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


    % Extract joint and link center of mass positions for this iteration
    JointPos = extractJointPositions(Mechanism, iter);
    LinkCoMPos = extractLinkCoMPositions(Mechanism, iter);

    % Scenarios: [newtonFlag, gravityFlag, frictionFlag]
    % scenarios = [0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1];
    scenarios = [1 1 0; 1 1 1];

    for scenario = scenarios.'
        updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, scenario(1), scenario(2), scenario(3));
    end
end

% Save the updated Mechanism with static analysis results
save('Mechanism.mat', 'Mechanism');

baseFolder = 'Force';
% Save Force Data
saveForceData(baseFolder, Mechanism);
end

function updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, newtonFlag, gravityFlag, frictionFlag)
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
theta = Mechanism.Theta;
Mechanism.(suffix).Torque(iter, :) = [0, 0, double(solution.T)];
Mechanism.(suffix).NormalForce(iter, :) = [double(solution.N)*cos(theta),double(solution.N)*sin(theta),0];
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
massAB = Mechanism.Mass.AB;
massBC = Mechanism.Mass.BC;
massPiston = Mechanism.Mass.Piston;

% Pull the mass moment of inertia of each component
massMoIAB = Mechanism.MassMoI.AB;
massMoIBC = Mechanism.MassMoI.BC;

% Pull the angular acceleration of each link
A_ab = Mechanism.AngAcc.AB(iter,:);
A_bc = Mechanism.AngAcc.BC(iter,:);

% Pull the acceleration of each link at its center of mass
A_ab_com = Mechanism.LinAcc.LinkCoM.AB(iter,:);
A_bc_com = Mechanism.LinAcc.LinkCoM.BC(iter,:);
A_piston = Mechanism.LinAcc.Joint.C(iter,:);

% Extract positions for each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;

% Extract positions for each link's center of mass
AB_com = LinkCoMPos.AB;
BC_com = LinkCoMPos.BC;

% Extract the angle that the slider travels on
theta = Mechanism.Theta;

% Define all the unknown variables to solve for
syms Ax Ay Bx By Cx Cy N T

%defining gravity to find weight of each link m/s^2
g = [0 -9.81 0];

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];

% Weight of each link
wAB=massAB*g*grav;
wBC=massBC*g*grav;
wPiston = massPiston*g*grav;

% Unknown torque of the system
tT=[0 0 T];

mu = 0.34; % Coefficient of friction

% Normal Force
F_N = [N*cos(theta) N*sin(theta) 0];

% Determine the direction of the friction force at the slider-cylinder interface
if Mechanism.LinVel.Joint.C(iter,:) > 0
    F_fr = mu * F_N * -1; % Assuming horizontal motion
elseif Mechanism.LinVel.Joint.C(iter,:) < 0
    F_fr = mu * F_N * 1;
else
    F_fr = mu * F_N * 0;
end

% Torque provided by friction
% N1 = Mechanism.NewtonForceGravFriction.NormalForce(iter,:);
% r_ab_com_a = norm(AB_com - A);
% T_fb = mu*N1*r_ab_com_a * friction;
% T_fr_crank = mu_crank * F_normal_crank * crank_radius;

%% FBD Equations
%Link AB
eqn1=fA+fB+wAB==massAB*A_ab_com*newton;
eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT==massMoIAB * A_ab*newton; %only change the ==0 appropriately for newtons 2nd law
% eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+T_fb+tT==massMoIAB * A_ab*newton; %only change the ==0 appropriately for newtons 2nd law
%Link BC
eqn3=-fB+fC+wBC==massBC*A_bc_com*newton;
eqn4=momentVec(B, BC_com, -fB)+momentVec(C, BC_com, fC)==massMoIBC * A_bc*newton; %only change the ==0 appropriately for newtons 2nd law
% eqn4=momentVec(B, BC_com, -fB)+momentVec(C, BC_com, fC)-T_fb==massMoIBC * A_bc*newton; %only change the ==0 appropriately for newtons 2nd law
% Piston
eqn5=-fC+F_fr+F_N+wPiston==massPiston*A_piston*newton;

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5],[Ax,Ay,Bx,By,Cx,Cy,N,T]));
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


% function saveForceData(baseFolder, Mechanism)
% % Define categories, conditions, and dataTypes
% categories = {'Static', 'Newton'};
% conditions = {'Grav', 'NoGrav'};
% conditions = {'Friction', 'NoFriction'};
%
% % Iterate through each combination of categories and conditions
% for iCategory = 1:length(categories)
%     for iCondition = 1:length(conditions)
%         category = categories{iCategory};
%         condition = conditions{iCondition};
%
%         % Construct force field name e.g., StaticForceGrav
%         forceFieldName = [category 'Force' condition];
%
%         % Prepare folders for both Joint and Torque (if applicable)
%         jointFolder = fullfile(baseFolder, category, condition, 'Joint');
%         torqueFolder = fullfile(baseFolder, category, condition, 'Torque');
%
%         % Ensure folders exist
%         if ~exist(jointFolder, 'dir')
%             mkdir(jointFolder);
%         end
%
%         % Process and save Joint data
%         if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Joint')
%             jointNames = fieldnames(Mechanism.(forceFieldName).Joint);
%             for iJoint = 1:length(jointNames)
%                 jointName = jointNames{iJoint};
%                 % Create temporary struct with joint data
%                 tempStruct = struct(jointName, Mechanism.(forceFieldName).Joint.(jointName));
%                 save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
%             end
%         end
%
%         % Process and save Torque data directly without creating a Torque directory
%         if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Torque')
%             % Define the torque data file path
%             torqueFilePath = fullfile(baseFolder, category, condition, 'Torque.mat');
%             % Extract torque data
%             Torque = Mechanism.(forceFieldName).Torque; % Make sure Torque is the correct field
%             save(torqueFilePath, 'Torque');
%         end
%     end
% end
% end

% function saveForceData(baseFolder, Mechanism)
% % Define categories, conditions, and friction states
% categories = {'Static', 'Newton'};
% conditions = {'Grav', 'NoGrav'};
% frictions = {'Friction', 'NoFriction'};
%
% % Iterate through each combination of categories, conditions, and frictions
% for iCategory = 1:length(categories)
%     for iCondition = 1:length(conditions)
%         for iFriction = 1:length(frictions)
%             category = categories{iCategory};
%             condition = conditions{iCondition};
%             friction = frictions{iFriction};
%
%             % Construct force field name e.g., StaticForceGravFriction
%             forceFieldName = [category 'Force' condition friction];
%
%             % Construct normal force field name e.g., StaticForceGrav.NormalForce
%             normalForceFieldName = [category 'Force' condition '.NormalForce'];
%
%             % Prepare folders for Joint, Torque, and NormalForce
%             jointFolder = fullfile(baseFolder, 'Force', category, condition, friction, 'Joint');
%             normalForceFolder = fullfile(baseFolder, 'Force', category, condition, friction); % Decision needed
%
%             % Ensure folders exist
%             if ~exist(jointFolder, 'dir')
%                 mkdir(jointFolder);
%             end
%             if ~exist(normalForceFolder, 'dir') && exist('normalForceFolder', 'var')
%                 mkdir(normalForceFolder);
%             end
%
%             % Process and save Joint data
%             if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Joint')
%                 jointNames = fieldnames(Mechanism.(forceFieldName).Joint);
%                 for iJoint = 1:length(jointNames)
%                     jointName = jointNames{iJoint};
%                     tempStruct = struct(jointName, Mechanism.(forceFieldName).Joint.(jointName));
%                     save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
%                 end
%             end
%
%             % Process and save Torque data
%             if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Torque')
%                 torqueFilePath = fullfile(baseFolder, 'Force', category, condition, friction, 'Torque.mat');
%                 Torque = Mechanism.(forceFieldName).Torque;
%                 save(torqueFilePath, 'Torque');
%             end
%
%             % Process and save Normal Force data
%             if isfield(Mechanism, normalForceFieldName) % Check if the normal force data exists
%                 normalForceFilePath = fullfile(normalForceFolder, 'NormalForce.mat');
%                 NormalForce = Mechanism.(normalForceFieldName); % Extract normal force data
%                 save(normalForceFilePath, 'NormalForce');
%             end
%         end
%     end
% end
% end