function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, JointPos, input_speed)
%velocity equations from given loops
syms wBCEF wCDG 
omegaAB=[0 0 input_speed];
omegaBCEF=[0 0 wBCEF];
omegaCDG=[0 0 wCDG];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=VelAccSolverUtils.velSolver(omegaAB,B-A)+VelAccSolverUtils.velSolver(omegaBCEF,C-B)+VelAccSolverUtils.velSolver(omegaCDG,D-C)==0;

solution=solve(eqn1,[wBCEF wCDG]);

% Store all the determined angular velocities
AngVel.AB=[0 0 input_speed];
AngVel.BCEF=[0 0 double(solution.wBCEF)]; %angular velocity of BCEF
AngVel.CDG=[0 0 double(solution.wCDG)]; %angular velocity of DE

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
BCEF_com = LinkCoMPos.BCEF;
CDG_com = LinkCoMPos.CDG;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.AB,B-A);
LinVel.Joint.C = VelAccSolverUtils.velSolver(AngVel.CDG,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = VelAccSolverUtils.velSolver(AngVel.BCEF,E-C) + LinVel.Joint.C;
LinVel.Joint.F = VelAccSolverUtils.velSolver(AngVel.BCEF,F-C) + LinVel.Joint.C;
LinVel.Joint.G = VelAccSolverUtils.velSolver(AngVel.CDG, G-D);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.AB = VelAccSolverUtils.velSolver(AngVel.AB,AB_com - A);
LinVel.LinkCoM.BCEF= VelAccSolverUtils.velSolver(AngVel.BCEF,BCEF_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDG= VelAccSolverUtils.velSolver(AngVel.CDG,CDG_com - D);

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
E = Pos.E;
F = Pos.F;
G = Pos.G;
%% Acceleration loops
%acceleration equations from given loops
syms aBCEF aCDG
alphaAB=[0 0 0];
alphaBCEF=[0 0 aBCEF];
alphaCDG=[0 0 aCDG];

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=VelAccSolverUtils.accSolver(AngVel.AB,alphaAB, B-A)+VelAccSolverUtils.accSolver(AngVel.BCEF,alphaBCEF,C-B)+VelAccSolverUtils.accSolver(AngVel.CDG,alphaCDG,D-C)==0;

solution=solve(eqn1,[aBCEF aCDG]);

% Store all the determined angular accelerations
AngAcc.AB=[0 0 0];
AngAcc.BCEF=[0 0 double(solution.aBCEF)]; %angular acceleration of BCEF
AngAcc.CDG=[0 0 double(solution.aCDG)]; %angular acceleration of CDG

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
BCEF_com = LinkCoMPos.BCEF;
CDG_com = LinkCoMPos.CDG;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.AB, AngAcc.AB,B-A);
LinAcc.Joint.C = VelAccSolverUtils.accSolver(AngVel.CDG, AngAcc.CDG,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = VelAccSolverUtils.accSolver(AngVel.BCEF, AngAcc.BCEF,E-B) + LinAcc.Joint.B;
LinAcc.Joint.F = VelAccSolverUtils.accSolver(AngVel.BCEF, AngAcc.BCEF,F-B) + LinAcc.Joint.B;
LinAcc.Joint.G = VelAccSolverUtils.accSolver(AngVel.CDG, AngAcc.CDG,G-C) + LinAcc.Joint.C;

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.AB = VelAccSolverUtils.accSolver(AngVel.AB,AngAcc.AB,AB_com - A);
LinAcc.LinkCoM.BCEF= VelAccSolverUtils.accSolver(AngVel.BCEF,AngAcc.BCEF,BCEF_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDG= VelAccSolverUtils.accSolver(AngVel.CDG,AngAcc.CDG,CDG_com - D);

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
