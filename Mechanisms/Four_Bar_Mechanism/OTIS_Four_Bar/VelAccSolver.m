function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, speedStr, JointPos, input_speed)
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
eqn1=VelAccSolverUtils.velSolver(omegaABE,B-A)+VelAccSolverUtils.velSolver(omegaBCFG,C-B)+VelAccSolverUtils.velSolver(omegaCDH,D-C)==0;

solution=solve(eqn1,[wBCFG wCDH]);

% Store all the determined angular velocities
AngVel.ABE=[0 0 input_speed];
AngVel.BCFG=[0 0 double(solution.wBCFG)]; %angular velocity of BCFG
AngVel.CDH=[0 0 double(solution.wCDH)]; %angular velocity of DE

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngVel.(linkNames{i}).(speedStr)(iter,:) = AngVel.(linkNames{i});
end
end
function [Mechanism] = determineLinVel(Mechanism, iter, speedStr, JointPos, LinkCoMPos, AngVel)
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
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.ABE,B-A);
LinVel.Joint.C = VelAccSolverUtils.velSolver(AngVel.CDH,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = VelAccSolverUtils.velSolver(AngVel.ABE,E-A);
LinVel.Joint.F = VelAccSolverUtils.velSolver(AngVel.BCFG,F-C) + LinVel.Joint.C;
LinVel.Joint.G = VelAccSolverUtils.velSolver(AngVel.BCFG,G-C) + LinVel.Joint.C;
LinVel.Joint.H = VelAccSolverUtils.velSolver(AngVel.CDH,H-D);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.ABE = VelAccSolverUtils.velSolver(AngVel.ABE,ABE_com - A);
LinVel.LinkCoM.BCFG= VelAccSolverUtils.velSolver(AngVel.BCFG,BCFG_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDH= VelAccSolverUtils.velSolver(AngVel.CDH,CDH_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinVel.Joint.(jointNames{i}).(speedStr)(iter,:) = LinVel.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinVel.Joint.(tracerPointNames{i}).(speedStr)(iter,:) = LinVel.Joint.(tracerPointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinVel.LinkCoM.(linkNames{i}).(speedStr)(iter,:) = LinVel.LinkCoM.(linkNames{i});
end

end
function [Mechanism, AngAcc] = determineAngAcc(Mechanism, iter, speedStr, Pos, AngVel)
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
eqn1=VelAccSolverUtils.accSolver(AngVel.ABE,alphaABE, B-A)+VelAccSolverUtils.accSolver(AngVel.BCFG,alphaBCFG,C-B)+VelAccSolverUtils.accSolver(AngVel.CDH,alphaCDH,D-C)==0;

solution=solve(eqn1,[aBCFG aCDH]);

% Store all the determined angular accelerations
AngAcc.ABE=[0 0 0];
AngAcc.BCFG=[0 0 double(solution.aBCFG)]; %angular acceleration of BCFG
AngAcc.CDH=[0 0 double(solution.aCDH)]; %angular acceleration of CDH

linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.AngAcc.(linkNames{i}).(speedStr)(iter,:) = AngAcc.(linkNames{i});
end
end
function [Mechanism] = determineLinAcc(Mechanism, iter, speedStr, JointPos, LinkCoMPos, AngVel, AngAcc)
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
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.ABE, AngAcc.ABE,B-A);
LinAcc.Joint.C = VelAccSolverUtils.accSolver(AngVel.CDH, AngAcc.CDH,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = VelAccSolverUtils.accSolver(AngVel.ABE, AngAcc.ABE,E-A);
LinAcc.Joint.F = VelAccSolverUtils.accSolver(AngVel.BCFG, AngAcc.BCFG,F-B) + LinAcc.Joint.B;
LinAcc.Joint.G = VelAccSolverUtils.accSolver(AngVel.BCFG, AngAcc.BCFG,G-B) + LinAcc.Joint.B;
LinAcc.Joint.H = VelAccSolverUtils.accSolver(AngVel.CDH, AngAcc.BCFG,H-D);

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.ABE = VelAccSolverUtils.accSolver(AngVel.ABE,AngAcc.ABE,ABE_com - A);
LinAcc.LinkCoM.BCFG= VelAccSolverUtils.accSolver(AngVel.BCFG,AngAcc.BCFG,BCFG_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDH= VelAccSolverUtils.accSolver(AngVel.CDH,AngAcc.CDH,CDH_com - D);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i}).(speedStr)(iter,:) = LinAcc.Joint.(jointNames{i});
end
tracerPointNames = fieldnames(Mechanism.TracerPoint);
for i = 1:length(tracerPointNames)
    Mechanism.LinAcc.Joint.(tracerPointNames{i}).(speedStr)(iter,:) = LinAcc.Joint.(tracerPointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinAcc.LinkCoM.(linkNames{i}).(speedStr)(iter,:) = LinAcc.LinkCoM.(linkNames{i});
end
end
