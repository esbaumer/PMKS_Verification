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

    % Perform static analysis calculations for the current iteration
    solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 0, 0);
    jointNames = fieldnames(Mechanism.Joint);
    for i = 1:length(jointNames)
        Mechanism.StaticForceNoGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    end
    Mechanism.StaticForceNoGrav.Torque(iter,:) = [0 0 double(solution.T)];

    solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 0, 1);
    jointNames = fieldnames(Mechanism.Joint);
    for i = 1:length(jointNames)
        Mechanism.StaticForceGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    end
    Mechanism.StaticForceGrav.Torque(iter,:) = [0 0 double(solution.T)];

    solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 1, 0);
    jointNames = fieldnames(Mechanism.Joint);
    for i = 1:length(jointNames)
        Mechanism.NewtonForceNoGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    end
    Mechanism.NewtonForceNoGrav.Torque(iter,:) = [0 0 double(solution.T)];

    solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 1, 1);
    jointNames = fieldnames(Mechanism.Joint);
    for i = 1:length(jointNames)
        Mechanism.NewtonForceGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    end
    Mechanism.NewtonForceGrav.Torque(iter,:) = [0 0 double(solution.T)];
end

% Save the updated Mechanism with static analysis results
save('Mechanism.mat', 'Mechanism');

baseFolder = 'Force';
% Save Force Data
saveForceData(baseFolder, Mechanism);
end

function [Mechanism] = initializeForceSolvers(Mechanism, numIterations)
% Initialize with zeros for storing forces and moments
jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.StaticForceGrav.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.StaticForceNoGrav.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceGrav.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
    Mechanism.NewtonForceNoGrav.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
end
Mechanism.StaticForceGrav.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.StaticForceNoGrav.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceGrav.Torque = zeros(numIterations, 3); % Assuming 3D forces
Mechanism.NewtonForceNoGrav.Torque = zeros(numIterations, 3); % Assuming 3D forces
end

function solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, newton, grav)
% Here, you'd implement your equations based on static conditions
% For each joint and link, calculate forces and moments ensuring sum of forces = 0 and sum of moments = 0

massAB = Mechanism.Mass.AB;
massBCD = Mechanism.Mass.BCD;
massDE = Mechanism.Mass.DE;
massEF = Mechanism.Mass.EF;
massCFG = Mechanism.Mass.CFG;

massMoIAB = Mechanism.MassMoI.AB;
massMoIBCD = Mechanism.MassMoI.BCD;
massMoIDE = Mechanism.MassMoI.DE;
massMoIEF = Mechanism.MassMoI.EF;
massMoICFG = Mechanism.MassMoI.CFG;

A_ab = Mechanism.AngAcc.AB(iter,:);
A_bcd = Mechanism.AngAcc.BCD(iter,:);
A_de = Mechanism.AngAcc.DE(iter,:);
A_ef = Mechanism.AngAcc.EF(iter,:);
A_cfg = Mechanism.AngAcc.CFG(iter,:);

A_ab_com = Mechanism.LinAcc.LinkCoM.AB(iter,:);
A_bcd_com = Mechanism.LinAcc.LinkCoM.BCD(iter,:);
A_de_com = Mechanism.LinAcc.LinkCoM.DE(iter,:);
A_ef_com = Mechanism.LinAcc.LinkCoM.EF(iter,:);
A_cfg_com = Mechanism.LinAcc.LinkCoM.CFG(iter,:);

% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
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

syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy T

g = [0 -9.81 0]; %defining gravity to find weight of each link m/s^2

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];
fE=[Ex Ey 0];
fF=[Fx Fy 0];
fG=[Gx Gy 0];

% Weight of each link
wAB=massAB *g * grav;
wBCD=massBCD *g * grav;
wDE=massDE *g * grav;
wEF=massEF *g * grav;
wCFG=massCFG *g * grav;

% Unknown torque of the system
tT=[0 0 T];

% Force of the applied load
LoadForce=[50 25 0];
LoadPos = (C + F + G) / 3;

%% Static Equilibrium Equations
%Link AB
eqn1=fA+fB+wAB==massAB*A_ab_com * newton;
eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT==massMoIAB * A_ab * newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCD
eqn3=-fB+fC+fD+wBCD==massBCD*A_bcd_com * newton;
eqn4=momentVec(B, BCD_com, -fB)+momentVec(C, BCD_com, fC) + momentVec(D, BCD_com, fD) ==massMoIBCD * A_bcd * newton; %only change the ==0 appropriately for newtons 2nd law
%Link DE
eqn5=-fD+fE+wDE==massDE*A_de_com * newton;
eqn6=momentVec(D, DE_com, -fD)+momentVec(E, DE_com, fE)==massMoIDE * A_de * newton; %only change the ==0 appropriately for newtons 2nd law
%Link EF
eqn7=-fE+fF+wEF==massEF*A_ef_com * newton;
eqn8=momentVec(E, EF_com, -fE)+momentVec(F, EF_com, fF)==massMoIEF * A_ef * newton; %only change the ==0 appropriately for newtons 2nd law
%Link CFG
eqn9=-fC-fF+fG+wCFG+LoadForce==massCFG*A_cfg_com * newton;
eqn10=momentVec(C, CFG_com, -fC)+momentVec(F, CFG_com, -fF)+momentVec(G, CFG_com, fG)+momentVec(LoadPos, CFG_com, LoadForce)==massMoICFG * A_cfg * newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,T]));
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
% Define categories, conditions, and dataTypes
categories = {'Static', 'Newton'};
conditions = {'Grav', 'NoGrav'};

% Iterate through each combination of categories and conditions
for iCategory = 1:length(categories)
    for iCondition = 1:length(conditions)
        category = categories{iCategory};
        condition = conditions{iCondition};

        % Construct force field name e.g., StaticForceGrav
        forceFieldName = [category 'Force' condition];

        % Prepare folders for both Joint and Torque (if applicable)
        jointFolder = fullfile(baseFolder, category, condition, 'Joint');
        torqueFolder = fullfile(baseFolder, category, condition, 'Torque');

        % Ensure folders exist
        if ~exist(jointFolder, 'dir')
            mkdir(jointFolder);
        end

        % Process and save Joint data
        if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Joint')
            jointNames = fieldnames(Mechanism.(forceFieldName).Joint);
            for iJoint = 1:length(jointNames)
                jointName = jointNames{iJoint};
                % Create temporary struct with joint data
                tempStruct = struct(jointName, Mechanism.(forceFieldName).Joint.(jointName));
                save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
            end
        end

        % Process and save Torque data directly without creating a Torque directory
        if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Torque')
            % Define the torque data file path
            torqueFilePath = fullfile(baseFolder, category, condition, 'Torque.mat');
            % Extract torque data
            Torque = Mechanism.(forceFieldName).Torque; % Make sure Torque is the correct field
            save(torqueFilePath, 'Torque');
        end
    end
end
end