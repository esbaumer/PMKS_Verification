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
massBC = Mechanism.Mass.BC;
massCDE = Mechanism.Mass.CDE;
massEF = Mechanism.Mass.EF;
massFG = Mechanism.Mass.FG;

massMoIAB = Mechanism.MassMoI.AB;
massMoIBC = Mechanism.MassMoI.BC;
massMoICDE = Mechanism.MassMoI.CDE;
massMoIEF = Mechanism.MassMoI.EF;
massMoIFG = Mechanism.MassMoI.FG;

A_ab = Mechanism.AngAcc.AB(iter,:);
A_bc = Mechanism.AngAcc.BC(iter,:);
A_cde = Mechanism.AngAcc.CDE(iter,:);
A_ef = Mechanism.AngAcc.EF(iter,:);
A_fg = Mechanism.AngAcc.FG(iter,:);

A_ab_com = Mechanism.LinAcc.LinkCoM.AB(iter,:);
A_bc_com = Mechanism.LinAcc.LinkCoM.BC(iter,:);
A_cde_com = Mechanism.LinAcc.LinkCoM.CDE(iter,:);
A_ef_com = Mechanism.LinAcc.LinkCoM.EF(iter,:);
A_fg_com = Mechanism.LinAcc.LinkCoM.FG(iter,:);

% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;
H = Mechanism.TracerPoint.H(iter, :);

AB_com = LinkCoMPos.AB;
BC_com = LinkCoMPos.BC;
CDE_com = LinkCoMPos.CDE;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

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
wBC=massBC *g * grav;
wCDE=massCDE *g * grav;
wEF=massEF *g * grav;
wFG=massFG *g * grav;

% Unknown torque of the system
tT=[0 0 T];

% Force of the applied load
LoadForce=[0 -200 0];
LoadPos = H;

%% Static Equilibrium Equations
%Link AB
eqn1=fA+fB+wAB==massAB*A_ab_com * newton;
eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT==massMoIAB * A_ab * newton; %only change the ==0 appropriately for newtons 2nd law
%Link BC
eqn3=-fB+fC+wBC==massBC*A_bc_com * newton;
eqn4=momentVec(B, BC_com, -fB)+momentVec(C, BC_com, fC) + momentVec(D, BC_com, fD) ==massMoIBC * A_bc * newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDE
eqn5=-fC+fD+fE+wCDE==massCDE*A_cde_com * newton;
eqn6=momentVec(C, CDE_com, -fC)+momentVec(D, CDE_com, fD)+momentVec(E, CDE_com, fE)==massMoICDE * A_cde * newton; %only change the ==0 appropriately for newtons 2nd law
%Link EF
eqn7=-fE+fF+wEF==massEF*A_ef_com * newton;
eqn8=momentVec(E, EF_com, -fE)+momentVec(F, EF_com, fF)==massMoIEF * A_ef * newton; %only change the ==0 appropriately for newtons 2nd law
%Link FG
eqn9=-fF+fG+wFG+LoadForce==massFG*A_fg_com * newton;
eqn10=momentVec(F, FG_com, -fF)+momentVec(G, FG_com, fG)+momentVec(LoadPos, FG_com, LoadForce)==massMoIFG * A_fg * newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,T]));

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.StaticForce.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    % Mechanism.ForceNewton.Joint.(jointNames{i})(iter,:) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
end
Mechanism.StaticForce.Torque(iter,:) = [0 0 double(solution.T)];
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

% clc;
% clear all;
% close all;
% 
% %%
% load('Kin/Pos/A')
% load('Kin/Pos/B')
% load('Kin/Pos/C')
% load('Kin/Pos/D')
% load('Kin/Pos/E')
% load('Kin/Pos/F')
% load('Kin/Pos/G')
% 
% massAB = 13.68;
% massBC = 33.07;
% massDCE = 57.7;
% massEF = 27.54;
% massFG = 59.94;
% 
% F_a_vec = zeros(10,3);
% F_b_vec = zeros(10,3);
% F_c_vec = zeros(10,3);
% F_d_vec = zeros(10,3);
% F_e_vec = zeros(10,3);
% F_f_vec = zeros(10,3);
% F_g_vec = zeros(10,3);
% Torque_vec = zeros(10,3);
% 
% for theta=1:1:10
% 
%     g = [0 -9.81 0]; %defining gravity to find weight of each link m/s^2
% 
%     A = A_vec(theta,:);
%     B = B_vec(theta,:);
%     C = C_vec(theta,:);
%     D = D_vec(theta,:);
%     E = E_vec(theta,:);
%     F = F_vec(theta,:);
%     G = G_vec(theta,:);
% 
%     % Centers of Mass of each Link
%     AB_com = (A+B)/2;
%     BC_com = (B+C)/2;
%     DCE_com = (C+D+E)/3;
%     EF_com = (E+F)/2;
%     FG_com = (F+G)/2;
% 
%     syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy T
%     % Forces at each joint
%     fA=[Ax Ay 0];
%     fB=[-Bx By 0];
%     fC=[-Cx Cy 0];
%     fD=[Dx Dy 0];
%     fE=[-Ex Ey 0];
%     fF=[-Fx Fy 0];
%     fG=[-Gx Gy 0];
%     Tt=[0 0 T];
% 
%     % Weight of each link
%     wAB=massAB *g;
%     wBC=massBC *g;
%     wDCE=massDCE *g;
%     wEF=massEF *g;
%     wFG=massFG *g;
% 
%     % Unknown torque of the system
%     tT=[0 0 T];
% 
%     % Force of the applied load
%     LoadForce=[0 -200 0];
%     LoadPos = [-1.71434 4.26299 0]; % +1.843 m from joint F. Calculation by hand
% 
%     %% Static Equilibrium Equations
%     %Link AB
%     eqn1=fA+fB+wAB==0;
%     eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT==0; %only change the ==0 appropriately for newtons 2nd law
%     %Link BC
%     eqn3=-fB+fC+wBC==0;
%     eqn4=momentVec(B, BC_com, -fB)+momentVec(C, BC_com, fC) ==0; %only change the ==0 appropriately for newtons 2nd law
%     %Link CD
%     eqn5=-fC+fD+fE+wDCE==0;
%     eqn6=momentVec(C, DCE_com, -fC)+momentVec(D, DCE_com, fD)==0+momentVec(E, DCE_com, fE); %only change the ==0 appropriately for newtons 2nd law
%     %Link EF
%     eqn7=-fE+fF+wEF==0;
%     eqn8=momentVec(E, EF_com, -fE)+momentVec(F, EF_com, fF)==0; %only change the ==0 appropriately for newtons 2nd law
%     %Link FG
%     eqn9=-fF+fG+wFG+LoadForce==0;
%     eqn10=momentVec(F, FG_com, -fF)+momentVec(G, FG_com, fG)+momentVec(LoadPos, FG_com, LoadForce)==0; %only change the ==0 appropriately for newtons 2nd law
% 
%     solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,T]));
% 
%     F_a_vec(theta,:) = [double(solution.Ax) double(solution.Ay) 0];
%     F_b_vec(theta,:) = [double(solution.Bx) double(solution.By) 0];
%     F_c_vec(theta,:) = [double(solution.Cx) double(solution.Cy) 0];
%     F_d_vec(theta,:) = [double(solution.Dx) double(solution.Dy) 0];
%     F_e_vec(theta,:) = [double(solution.Ex) double(solution.Ey) 0];
%     F_f_vec(theta,:) = [double(solution.Fx) double(solution.Fy) 0];
%     F_g_vec(theta,:) = [double(solution.Gx) double(solution.Gy) 0];
%     Torque_vec(theta,:)=[0 0 double(solution.T)];
% end
% 
% % Create Directory for Saving Results
% folderName = 'Stat';  % Directory name
% if ~exist(folderName, 'dir')
%    mkdir(folderName);  % Create the directory if it doesn't exist
% end
% 
% 
% save('Stat/Joint_A', 'F_a_vec')
% save('Stat/Joint_B', 'F_b_vec')
% save('Stat/Joint_C', 'F_c_vec')
% save('Stat/Joint_D', 'F_d_vec')
% save('Stat/Joint_E', 'F_e_vec')
% save('Stat/Joint_F', 'F_f_vec')
% save('Stat/Joint_G', 'F_g_vec')
% save('Stat/Torque', 'Torque_vec')
% 
% function pos = momentVec(pos, fixPos, force)
%     % Position Vector
%     r = pos - fixPos;
%     pos = cross(r,force);
% end
