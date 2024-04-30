function Mechanism = VelAccSolver(Mechanism)
    Mechanism = VelAccSolverUtils.VelAccSolver(Mechanism, @determineAngVel, @determineLinVel, @determineAngAcc, @determineLinAcc);
end

%% Velocity loops
function [Mechanism, AngVel] = determineAngVel(Mechanism, iter, speedStr, JointPos, input_speed)

% velocity equations from given loops
syms wBCEF wCDGI 
omegaABH=[0 0 input_speed];
omegaBCEF=[0 0 wBCEF];
omegaCDGI=[0 0 wCDGI];

A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

% A->B->C->D->A
% V_ba + V_cb + V_dc + V_ad = 0
eqn1=VelAccSolverUtils.velSolver(omegaABH,B-A)+VelAccSolverUtils.velSolver(omegaBCEF,C-B)+VelAccSolverUtils.velSolver(omegaCDGI,D-C)==0;

solution=solve(eqn1,[wBCEF wCDGI]);

% Store all the determined angular velocities
AngVel.ABH=[0 0 input_speed];
AngVel.BCEF=[0 0 double(solution.wBCEF)]; %angular velocity of BCEF
AngVel.CDGI=[0 0 double(solution.wCDGI)]; %angular velocity of DE

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
I = JointPos.I;

ABH_com = LinkCoMPos.ABH;
BCEF_com = LinkCoMPos.BCEF;
CDGI_com = LinkCoMPos.CDGI;

LinVel.Joint.A = [0 0 0];
LinVel.Joint.B = VelAccSolverUtils.velSolver(AngVel.ABH,B-A);
LinVel.Joint.C = VelAccSolverUtils.velSolver(AngVel.CDGI,C-D);
LinVel.Joint.D = [0 0 0];
LinVel.Joint.E = VelAccSolverUtils.velSolver(AngVel.BCEF,E-A);
LinVel.Joint.F = VelAccSolverUtils.velSolver(AngVel.BCEF,F-C) + LinVel.Joint.C;
LinVel.Joint.G = VelAccSolverUtils.velSolver(AngVel.CDGI, G-C) + LinVel.Joint.C;
LinVel.Joint.H = VelAccSolverUtils.velSolver(AngVel.CDGI, H-A);
LinVel.Joint.I = VelAccSolverUtils.velSolver(AngVel.CDGI, I-D);

% Determine the velocities at each link's center of mass
LinVel.LinkCoM.ABH = VelAccSolverUtils.velSolver(AngVel.ABH,ABH_com - A);
LinVel.LinkCoM.BCEF= VelAccSolverUtils.velSolver(AngVel.BCEF,BCEF_com - B) + LinVel.Joint.B;
LinVel.LinkCoM.CDGI= VelAccSolverUtils.velSolver(AngVel.CDGI,CDGI_com - D);

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
syms aBCEF aCDGI
alphaABH=[0 0 0];
alphaBCEF=[0 0 aBCEF];
alphaCDGI=[0 0 aCDGI];

% A->B->C->D->A
% A_ba + A_cb + A_dc + A_ad = 0
eqn1=VelAccSolverUtils.accSolver(AngVel.ABH,alphaABH, B-A)+VelAccSolverUtils.accSolver(AngVel.BCEF,alphaBCEF,C-B)+VelAccSolverUtils.accSolver(AngVel.CDGI,alphaCDGI,D-C)==0;

solution=solve(eqn1,[aBCEF aCDGI]);

% Store all the determined angular accelerations
AngAcc.ABH=[0 0 0];
AngAcc.BCEF=[0 0 double(solution.aBCEF)]; %angular acceleration of BCEF
AngAcc.CDGI=[0 0 double(solution.aCDGI)]; %angular acceleration of CDGI

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
I = JointPos.I;

ABH_com = LinkCoMPos.ABH;
BCEF_com = LinkCoMPos.BCEF;
CDGI_com = LinkCoMPos.CDGI;

% Determine the accelerations at each joint
LinAcc.Joint.A = [0 0 0];
LinAcc.Joint.B = VelAccSolverUtils.accSolver(AngVel.ABH, AngAcc.ABH,B-A);
LinAcc.Joint.C = VelAccSolverUtils.accSolver(AngVel.CDGI, AngAcc.CDGI,C-D);
LinAcc.Joint.D = [0 0 0];
LinAcc.Joint.E = VelAccSolverUtils.accSolver(AngVel.BCEF, AngAcc.BCEF,E-A);
LinAcc.Joint.F = VelAccSolverUtils.accSolver(AngVel.BCEF, AngAcc.BCEF,F-B) + LinAcc.Joint.B;
LinAcc.Joint.G = VelAccSolverUtils.accSolver(AngVel.CDGI, AngAcc.CDGI,G-B) + LinAcc.Joint.B;
LinAcc.Joint.H = VelAccSolverUtils.accSolver(AngVel.BCEF, AngAcc.BCEF,F-A);
LinAcc.Joint.I = VelAccSolverUtils.accSolver(AngVel.CDGI, AngAcc.CDGI,G-D);

% Determine the accelerations at each link's center of mass
LinAcc.LinkCoM.ABH = VelAccSolverUtils.accSolver(AngVel.ABH,AngAcc.ABH,ABH_com - A);
LinAcc.LinkCoM.BCEF= VelAccSolverUtils.accSolver(AngVel.BCEF,AngAcc.BCEF,BCEF_com - B) + LinAcc.Joint.B;
LinAcc.LinkCoM.CDGI= VelAccSolverUtils.accSolver(AngVel.CDGI,AngAcc.CDGI,CDGI_com - D);

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
