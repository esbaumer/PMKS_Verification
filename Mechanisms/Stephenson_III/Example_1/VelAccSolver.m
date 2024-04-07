function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, JointPos, input_speed)
%velocity equations from given loops
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;

%% Velocity loops
%velocity equations from given loops
syms wBC wCDE wEF wFG
omegaAB=[0 0 input_speed];
omegaBC=[0 0 wBC];
omegaCDE=[0 0 wCDE];
omegaEF=[0 0 wEF];
omegaFG=[0 0 wFG];

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=VelAccSolverUtils.velSolver(omegaAB,B-A)+VelAccSolverUtils.velSolver(omegaBC,C-B)+VelAccSolverUtils.velSolver(omegaCDE,D-C)==0;

% D->E->F->G->D
% V_ed + V_fe + V_gf + V_dg = 0
eqn2=VelAccSolverUtils.velSolver(omegaCDE,E-D)+VelAccSolverUtils.velSolver(omegaEF,F-E)+VelAccSolverUtils.velSolver(omegaFG,G-F)==0;

solution=solve([eqn1, eqn2],[wBC wCDE wEF wFG]);

% Store all the determined angular velocities
AngVel.AB=[0 0 input_speed];
AngVel.BC=[0 0 double(solution.wBC)]; %angular velocity of BC
AngVel.CDE=[0 0 double(solution.wCDE)]; %angular velocity of DE
AngVel.EF=[0 0 double(solution.wEF)]; %angular velocity of EF
AngVel.FG=[0 0 double(solution.wFG)]; %angular velocity of FG

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
BC_com = LinkCoMPos.BC;
CDE_com = LinkCoMPos.CDE;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.AB,B-A);
LinVel.Joint.C = VelAccSolverUtils.velSolver(AngVel.CDE,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = VelAccSolverUtils.velSolver(AngVel.CDE,E-D);
LinVel.Joint.F = VelAccSolverUtils.velSolver(AngVel.FG,F-G);
LinVel.Joint.G = [0 0 0];

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.AB = VelAccSolverUtils.velSolver(AngVel.AB,AB_com - A);
LinVel.LinkCoM.BC= VelAccSolverUtils.velSolver(AngVel.BC,BC_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDE= VelAccSolverUtils.velSolver(AngVel.CDE,CDE_com - D);
LinVel.LinkCoM.EF = VelAccSolverUtils.velSolver(AngVel.EF,EF_com - E) + LinVel.Joint.E;
LinVel.LinkCoM.FG = VelAccSolverUtils.velSolver(AngVel.FG,FG_com - G);

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
%acceleration equations from given loops
syms aBC a_CDE aEF aFG
alphaAB=[0 0 0];
alphaBC=[0 0 aBC];
alphaCDE=[0 0 a_CDE];
alphaEF=[0 0 aEF];
alphaFG=[0 0 aFG];

A = Pos.A;
B = Pos.B;
C = Pos.C;
D = Pos.D;
E = Pos.E;
F = Pos.F;
G = Pos.G;
%% Acceleration loops

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=VelAccSolverUtils.accSolver(AngVel.AB,alphaAB, B-A)+VelAccSolverUtils.accSolver(AngVel.BC,alphaBC,C-B)+VelAccSolverUtils.accSolver(AngVel.CDE,alphaCDE,D-C)==0;

% D->E->F->G->D
% A_cde + A_fe + A_gf + A_dg = 0
eqn2=VelAccSolverUtils.accSolver(AngVel.CDE,alphaCDE,E-D)+VelAccSolverUtils.accSolver(AngVel.EF,alphaEF,F-E)+VelAccSolverUtils.accSolver(AngVel.FG,alphaFG,G-F)==0;

solution=solve([eqn1, eqn2],[aBC a_CDE aEF aFG]);

% Store all the determined angular accelerations
AngAcc.AB=[0 0 0];
AngAcc.BC=[0 0 double(solution.aBC)]; %angular acceleration of BC
AngAcc.CDE=[0 0 double(solution.a_CDE)]; %angular acceleration of CDE
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
BC_com = LinkCoMPos.BC;
CDE_com = LinkCoMPos.CDE;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.AB, AngAcc.AB,B-A);
LinAcc.Joint.C = VelAccSolverUtils.accSolver(AngVel.BC, AngAcc.BC,C-B) + LinAcc.Joint.B;
LinAcc.Joint.D = VelAccSolverUtils.accSolver(AngVel.BC, AngAcc.BC,D-B) + LinAcc.Joint.B;
LinAcc.Joint.E = VelAccSolverUtils.accSolver(AngVel.CDE, AngAcc.CDE,E-D) + LinAcc.Joint.D;
LinAcc.Joint.F = VelAccSolverUtils.accSolver(AngVel.FG,AngAcc.FG, F-G);
LinAcc.Joint.G = [0 0 0];

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.AB = VelAccSolverUtils.accSolver(AngVel.AB,AngAcc.AB,AB_com - A);
LinAcc.LinkCoM.BC= VelAccSolverUtils.accSolver(AngVel.BC,AngAcc.BC,BC_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDE= VelAccSolverUtils.accSolver(AngVel.CDE,AngAcc.CDE,CDE_com - D) + LinAcc.Joint.D;
LinAcc.LinkCoM.EF= VelAccSolverUtils.accSolver(AngVel.EF,AngAcc.EF,EF_com - E) + LinAcc.Joint.E;
LinAcc.LinkCoM.FG = VelAccSolverUtils.accSolver(AngVel.FG,AngAcc.FG,FG_com - G);

jointNames = fieldnames(Mechanism.Joint);
for i = 1:length(jointNames)
    Mechanism.LinAcc.Joint.(jointNames{i})(iter,:) = LinAcc.Joint.(jointNames{i});
end
linkNames = fieldnames(Mechanism.LinkCoM);
for i = 1:length(linkNames)
    Mechanism.LinAcc.LinkCoM.(linkNames{i})(iter,:) = LinAcc.LinkCoM.(linkNames{i});
end
end
