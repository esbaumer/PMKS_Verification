function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, JointPos, input_speed)
%velocity equations from given loops
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;

theta = Mechanism.Theta;
%% Velocity loops
%velocity equations from given loops
syms wBC V_c
omegaAB=[0 0 input_speed];
omegaBC=[0 0 wBC];

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=VelAccSolverUtils.velSolver(omegaAB,B-A)+VelAccSolverUtils.velSolver(omegaBC,C-B)-[V_c*cos(theta) V_c*sin(theta) 0]==0;
solution=solve(eqn1,[wBC, V_c]);

% Store all the determined angular velocities
AngVel.AB=[0 0 input_speed];
AngVel.BC=[0 0 double(solution.wBC)]; %angular velocity of BC

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngVel.(linkNames{i})(iter,:) = AngVel.(linkNames{i});
end
Mechanism.LinVel.Joint.C(iter,:) = [solution.V_c*cos(theta) solution.V_c*sin(theta) 0];
end
function [Mechanism] = determineLinVel(Mechanism, iter, JointPos, LinkCoMPos, AngVel)
% Determine the velocities at each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;

AB_com = LinkCoMPos.AB;
BC_com = LinkCoMPos.BC;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.AB,B-A);
LinVel.Joint.C = Mechanism.LinVel.Joint.C(iter,:);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.AB = VelAccSolverUtils.velSolver(AngVel.AB,AB_com - A);
LinVel.LinkCoM.BC = VelAccSolverUtils.velSolver(AngVel.BC,BC_com - B) + LinVel.Joint.B;

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinVel.Joint.(jointNames{i})(iter,:) = LinVel.Joint.(jointNames{i});
end
% Mechanism.LinVel.Joint.D(iter,:) = LinVel.Joint.D;
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinVel.LinkCoM.(linkNames{i})(iter,:) = LinVel.LinkCoM.(linkNames{i});
end

end
function [Mechanism, AngAcc] = determineAngAcc(Mechanism, iter, Pos, AngVel)
%acceleration equations from given loops
syms aBC A_c
alphaAB=[0 0 0];
alphaBC=[0 0 aBC];

A = Pos.A;
B = Pos.B;
C = Pos.C;
% D = Mechanism.TracerPoint.D(iter,:);
theta = Mechanism.Theta;
%% Acceleration loops

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=VelAccSolverUtils.accSolver(AngVel.AB,alphaAB, B-A)+VelAccSolverUtils.accSolver(AngVel.BC,alphaBC,C-B)-[A_c*cos(theta) A_c*sin(theta) 0]==0;

solution=solve(eqn1,[aBC A_c]);

% Store all the determined angular accelerations
AngAcc.AB=[0 0 0];
AngAcc.BC=[0 0 double(solution.aBC)]; %angular acceleration of BC

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngAcc.(linkNames{i})(iter,:) = AngAcc.(linkNames{i});
end
theta = Mechanism.Theta;
Mechanism.LinAcc.Joint.C(iter,:) = [solution.A_c*cos(theta) solution.A_c*sin(theta) 0];
end
function [Mechanism] = determineLinAcc(Mechanism, iter, JointPos, LinkCoMPos, AngVel, AngAcc)
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
% D = Mechanism.TracerPoint.D(iter,:);

AB_com = LinkCoMPos.AB;
BC_com = LinkCoMPos.BC;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.AB, AngAcc.AB,B-A);
LinAcc.Joint.C = Mechanism.LinAcc.Joint.C(iter,:);

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.AB = VelAccSolverUtils.accSolver(AngVel.AB,AngAcc.AB,AB_com - A);
LinAcc.LinkCoM.BC= VelAccSolverUtils.accSolver(AngVel.BC,AngAcc.BC,BC_com - B) + LinAcc.Joint.B;

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i})(iter,:) = LinAcc.Joint.(jointNames{i});
end
% Mechanism.LinAcc.Joint.D(iter,:) = LinAcc.Joint.D;
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinAcc.LinkCoM.(linkNames{i})(iter,:) = LinAcc.LinkCoM.(linkNames{i});
end
end
