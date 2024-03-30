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

    % % Perform static analysis calculations for the current iteration
    % solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 0, 0);
    % jointNames = fieldnames(Mechanism.Joint);
    % for i = 1:length(jointNames)
    %     Mechanism.StaticForceNoGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    % end
    % Mechanism.StaticForceNoGrav.Torque(iter,:) = [0 0 double(solution.T)];

    % solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 0, 1);
    % jointNames = fieldnames(Mechanism.Joint);
    % for i = 1:length(jointNames)
    %     Mechanism.StaticForceGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    % end
    % Mechanism.StaticForceGrav.Torque(iter,:) = [0 0 double(solution.T)];

    solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 1, 0);
    jointNames = fieldnames(Mechanism.Joint);
    for i = 1:length(jointNames)
        Mechanism.NewtonForceNoGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    end
    Mechanism.NewtonForceNoGrav.Torque(iter,:) = [0 0 double(solution.T)];

    % solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, 1, 1);
    % jointNames = fieldnames(Mechanism.Joint);
    % for i = 1:length(jointNames)
    %     Mechanism.NewtonForceGrav.Joint.(jointNames{i})(iter, :) = [double(solution.([jointNames{i}, 'x'])), double(solution.([jointNames{i}, 'y'])), 0];
    % end
    % Mechanism.NewtonForceGrav.Torque(iter,:) = [0 0 double(solution.T)];
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
massBCEF = Mechanism.Mass.BCEF;
massCD = Mechanism.Mass.CD;

massMoIAB = Mechanism.MassMoI.AB;
massMoIBCEF = Mechanism.MassMoI.BCEF;
massMoICD = Mechanism.MassMoI.CD;

A_ab = Mechanism.AngAcc.AB(iter,:);
A_bcef = Mechanism.AngAcc.BCEF(iter,:);
A_cd = Mechanism.AngAcc.CD(iter,:);

A_ab_com = Mechanism.LinAcc.LinkCoM.AB(iter,:);
A_bcef_com = Mechanism.LinAcc.LinkCoM.BCEF(iter,:);
A_cd_com = Mechanism.LinAcc.LinkCoM.CD(iter,:);


% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;

AB_com = LinkCoMPos.AB;
BCEF_com = LinkCoMPos.BCEF;
CD_com = LinkCoMPos.CD;

syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy T

g = [0 -9.81 0]; %defining gravity to find weight of each link m/s^2

mu = 0.3;

% General Helpful equations to do analysis
theta1 = atan2(Ay, Ax);
theta2 = atan2(By, Bx);
theta3 = atan2(Cy, Cx);

N1 = sqrt(Ax^2 + Ay^2); 
N2 = sqrt(Bx^2 + By^2); 
N3 = sqrt(Bx^2 + By^2); 

r_ab_com_a = norm(AB_com - A);
r_ab_com_b = norm(AB_com - B);
r_bcef_com_c = norm(BCEF_com - C);

F_fb_x = mu*N2*cos(theta2);
F_fb_y = mu*N2*sin(theta2);
F_fc_x = mu*N3*cos(theta3);
F_fc_y = mu*N3*sin(theta3);

T_fa_z = mu*N1*r_ab_com_a;
T_fb_z = mu*N1*r_ab_com_b;
T_fc_z = mu*N2*r_bcef_com_c;

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];
F_fb = [-F_fb_x -F_fb_y 0];
F_fc = [-F_fc_x -F_fc_y 0];
T_fa = [0 0 T_fa_z];
T_fb = [0 0 T_fb_z];
T_fc = [0 0 T_fc_z];

% Weight of each link
wAB=massAB *g * grav;
wBCEF=massBCEF *g * grav;
wCD=massCD *g * grav;

% Unknown torque of the system
tT=[0 0 T];

%% Equations from FBD
%Link AB
eqn1=fA+fB+F_fb+wAB==massAB*A_ab_com * newton;
eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT+T_fa+T_fb==massMoIAB * A_ab * newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCEF
eqn3=-fB+fC-F_fb+F_fc+wBCEF==massBCEF*A_bcef_com * newton;
eqn4=momentVec(B, BCEF_com, -fB)+momentVec(C, BCEF_com, fC)-T_fb+T_fc==massMoIBCEF * A_bcef * newton; %only change the ==0 appropriately for newtons 2nd law
%Link CD
eqn5=-fC+fD-F_fc+wCD==massCD*A_cd_com * newton;
eqn6=momentVec(C, CD_com, -fC)+momentVec(D, CD_com, fD)-T_fc==massMoICD * A_cd * newton; %only change the ==0 appropriately for newtons 2nd law

% Define initial guesses for your variables if known
% guess = [Ax, Ay, Bx, By, Cx, Cy_guess, Dx_guess, Dy_guess, T_guess];
% Use vpasolve directly
% [solution, parameters, conditions] = vpasolve([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6], [Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, T]);
solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,T]));
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