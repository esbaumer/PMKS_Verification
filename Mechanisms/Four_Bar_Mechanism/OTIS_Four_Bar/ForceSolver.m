function Mechanism = ForceSolver(Mechanism, scenarios)
    Mechanism = ForceSolverUtils.ForceSolver(Mechanism, scenarios, @performForceAnalysis);
end

function solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, newton, grav, friction)
% Pull the mass of each component
massABE = Mechanism.Mass.ABE;
massBCFG = Mechanism.Mass.BCFG;
massCDH = Mechanism.Mass.CDH;

% Pull the mass moment of inertia of each component
massMoIABE = Mechanism.MassMoI.ABE;
massMoIBCFG = Mechanism.MassMoI.BCFG;
massMoICDH = Mechanism.MassMoI.CDH;

% Pull the angular acceleration of each link
A_abe = Mechanism.AngAcc.ABE(iter,:);
A_bcfg = Mechanism.AngAcc.BCFG(iter,:);
A_cdh = Mechanism.AngAcc.CDH(iter,:);

% Pull the acceleration of each link at its center of mass
A_abe_com = Mechanism.LinAcc.LinkCoM.ABE(iter,:);
A_bcfg_com = Mechanism.LinAcc.LinkCoM.BCFG(iter,:);
A_cdh_com = Mechanism.LinAcc.LinkCoM.CDH(iter,:);

% Extract positions for each joint
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

mu = 0.20;

% Extract positions for each link's center of mass
ABE_com = LinkCoMPos.ABE;
BCFG_com = LinkCoMPos.BCFG;
CDH_com = LinkCoMPos.CDH;

% Define all the unknown variables to solve for
syms Ax Ay Bx By Cx Cy Dx Dy T

%defining gravity to find weight of each link m/s^2
g = [0 -9.81 0];

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];

% Weight of each link
wABE=massABE*g*grav;
wBCFG=massBCFG*g*grav;
wCDH=massCDH*g*grav;

% Unknown torque of the system
tT=[0 0 T];

% Torque provided by friction on Joint A
r_abe_com_a = norm(ABE_com - A);
r_bcfg_com_b = norm(BCFG_com - B);
r_cdh_com_c = norm(CDH_com - C);

A_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.A(iter,1);
A_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.A(iter,2);
B_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.B(iter,1); 
B_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.B(iter,2);
C_noFriction_x = Mechanism.NewtonForceGravNoFriction.Joint.C(iter,1); 
C_noFriction_y = Mechanism.NewtonForceGravNoFriction.Joint.C(iter,2);

% Correcting angle calculation
A_theta = atan2(A_noFriction_y, A_noFriction_x);
B_theta = atan2(B_noFriction_y, B_noFriction_x);
C_theta = atan2(C_noFriction_y, C_noFriction_x);

A_friction_Mag = norm([A_noFriction_x, A_noFriction_y]);
B_friction_Mag = norm([B_noFriction_x, B_noFriction_y]);
C_friction_Mag = norm([C_noFriction_x, C_noFriction_y]);

% Correct normal force direction
F_normal_A = [A_friction_Mag*cos(A_theta), A_friction_Mag*sin(A_theta), 0];
F_normal_B = [B_friction_Mag*cos(B_theta), B_friction_Mag*sin(B_theta), 0];
F_normal_C = [C_friction_Mag*cos(C_theta), C_friction_Mag*sin(C_theta), 0];

% Friction forces (assuming directions are appropriately chosen)
F_friction_A = mu * norm(F_normal_A) * [-sin(A_theta), cos(A_theta), 0] * friction; % Perpendicular to normal force
F_friction_B = mu * norm(F_normal_B) * [-sin(B_theta), cos(B_theta), 0] * friction; % Perpendicular to normal force
F_friction_C = mu * norm(F_normal_C) * [-sin(C_theta), cos(C_theta), 0] * friction; % Perpendicular to normal force

% Assuming r_abe_com_a and r_bcfg_com_b are correctly calculated lever arms
T_fr_A = [0,0, mu * norm(F_normal_A) * r_abe_com_a * friction]; % Torque due to friction at A
T_fr_B = [0,0, mu * norm(F_normal_B) * r_bcfg_com_b * friction]; % Torque due to friction at B
T_fr_C = [0,0, mu * norm(F_normal_B) * r_cdh_com_c * friction]; % Torque due to friction at B

%% FBD Equations
%Link ABE
eqn1=fA+fB+wABE+F_friction_A+F_friction_B==massABE*A_abe_com*newton;
eqn2=ForceSolverUtils.momentVec(A, ABE_com, fA) + ForceSolverUtils.momentVec(B,  ABE_com,fB)+tT+T_fr_A+T_fr_B==massMoIABE * A_abe*newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCFG
eqn3=-fB+fC+wBCFG-F_friction_B+F_friction_C==massBCFG*A_bcfg_com*newton;
eqn4=ForceSolverUtils.momentVec(B, BCFG_com, -fB)+ForceSolverUtils.momentVec(C, BCFG_com, fC)-T_fr_B+T_fr_C==massMoIBCFG * A_bcfg*newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDH
eqn5=-fC+fD+wCDH-F_friction_C==massCDH*A_cdh_com*newton;
eqn6=ForceSolverUtils.momentVec(C, CDH_com, -fC)+ForceSolverUtils.momentVec(D, CDH_com, fD)-T_fr_C==massMoICDH * A_cdh*newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,T]));
end