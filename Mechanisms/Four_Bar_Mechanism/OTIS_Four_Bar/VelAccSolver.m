function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, speedIndex, JointPos, input_speed)
%velocity equations from given loops
syms wBCFG wCDI 
omegaABEH=[0 0 input_speed];
omegaBCFG=[0 0 wBCFG];
omegaCDI=[0 0 wCDI];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=VelAccSolverUtils.velSolver(omegaABEH,B-A)+VelAccSolverUtils.velSolver(omegaBCFG,C-B)+VelAccSolverUtils.velSolver(omegaCDI,D-C)==0;

solution=solve(eqn1,[wBCFG wCDI]);

% Store all the determined angular velocities
AngVel.ABEH=[0 0 input_speed];
AngVel.BCFG=[0 0 double(solution.wBCFG)]; %angular velocity of BCFG
AngVel.CDI=[0 0 double(solution.wCDI)]; %angular velocity of DE

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngVel.(linkNames{i})(iter,:, speedIndex) = AngVel.(linkNames{i});
end
end
function [Mechanism] = determineLinVel(Mechanism, iter, speedIndex, JointPos, LinkCoMPos, AngVel)
% Determine the velocities at each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;
H = JointPos.H;

ABEH_com = LinkCoMPos.ABEH;
BCFG_com = LinkCoMPos.BCFG;
CDI_com = LinkCoMPos.CDI;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.ABEH,B-A);
LinVel.Joint.C = VelAccSolverUtils.velSolver(AngVel.CDI,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = VelAccSolverUtils.velSolver(AngVel.ABEH,E-A);
LinVel.Joint.F = VelAccSolverUtils.velSolver(AngVel.BCFG,F-C) + LinVel.Joint.C;
LinVel.Joint.G = VelAccSolverUtils.velSolver(AngVel.BCFG,G-C) + LinVel.Joint.C;
LinVel.Joint.H = VelAccSolverUtils.velSolver(AngVel.CDI,H-D);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.ABEH = VelAccSolverUtils.velSolver(AngVel.ABEH,ABEH_com - A);
LinVel.LinkCoM.BCFG= VelAccSolverUtils.velSolver(AngVel.BCFG,BCFG_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDI= VelAccSolverUtils.velSolver(AngVel.CDI,CDI_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinVel.Joint.(jointNames{i})(iter,:, speedIndex) = LinVel.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinVel.Joint.(tracerPointNames{i})(iter,:, speedIndex) = LinVel.Joint.(tracerPointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinVel.LinkCoM.(linkNames{i})(iter,:, speedIndex) = LinVel.LinkCoM.(linkNames{i});
end

end
function [Mechanism, AngAcc] = determineAngAcc(Mechanism, iter, speedIndex, Pos, AngVel)
A = Pos.A;
B = Pos.B;
C = Pos.C;
D = Pos.D;

%% Acceleration loops
%acceleration equations from given loops
syms aBCFG aCDI
alphaABEH=[0 0 0];
alphaBCFG=[0 0 aBCFG];
alphaCDI=[0 0 aCDI];

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=VelAccSolverUtils.accSolver(AngVel.ABEH,alphaABEH, B-A)+VelAccSolverUtils.accSolver(AngVel.BCFG,alphaBCFG,C-B)+VelAccSolverUtils.accSolver(AngVel.CDI,alphaCDI,D-C)==0;

solution=solve(eqn1,[aBCFG aCDI]);

% Store all the determined angular accelerations
AngAcc.ABEH=[0 0 0];
AngAcc.BCFG=[0 0 double(solution.aBCFG)]; %angular acceleration of BCFG
AngAcc.CDI=[0 0 double(solution.aCDI)]; %angular acceleration of CDI

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngAcc.(linkNames{i})(iter,:, speedIndex) = AngAcc.(linkNames{i});
end
end
function [Mechanism] = determineLinAcc(Mechanism, iter, speedIndex, JointPos, LinkCoMPos, AngVel, AngAcc)
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;
H = JointPos.H;

ABEH_com = LinkCoMPos.ABEH;
BCFG_com = LinkCoMPos.BCFG;
CDI_com = LinkCoMPos.CDI;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.ABEH, AngAcc.ABEH,B-A);
LinAcc.Joint.C = VelAccSolverUtils.accSolver(AngVel.CDI, AngAcc.CDI,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = VelAccSolverUtils.accSolver(AngVel.ABEH, AngAcc.ABEH,E-A);
LinAcc.Joint.F = VelAccSolverUtils.accSolver(AngVel.BCFG, AngAcc.BCFG,F-B) + LinAcc.Joint.B;
LinAcc.Joint.G = VelAccSolverUtils.accSolver(AngVel.BCFG, AngAcc.BCFG,G-B) + LinAcc.Joint.B;
LinAcc.Joint.H = VelAccSolverUtils.accSolver(AngVel.CDI, AngAcc.BCFG,H-D);

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.ABEH = VelAccSolverUtils.accSolver(AngVel.ABEH,AngAcc.ABEH,ABEH_com - A);
LinAcc.LinkCoM.BCFG= VelAccSolverUtils.accSolver(AngVel.BCFG,AngAcc.BCFG,BCFG_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDI= VelAccSolverUtils.accSolver(AngVel.CDI,AngAcc.CDI,CDI_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i})(iter,:, speedIndex) = LinAcc.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinAcc.Joint.(tracerPointNames{i})(iter,:, speedIndex) = LinAcc.Joint.(tracerPointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinAcc.LinkCoM.(linkNames{i})(iter,:, speedIndex) = LinAcc.LinkCoM.(linkNames{i});
end
end
