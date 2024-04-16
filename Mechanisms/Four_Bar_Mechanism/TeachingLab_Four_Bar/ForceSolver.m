function Mechanism = ForceSolver(Mechanism, scenarios)
    Mechanism = ForceSolverUtils.ForceSolver(Mechanism, scenarios, @performForceAnalysis);
end

function solution = performForceAnalysis(Mechanism, iter, speedStr, JointPos, LinkCoMPos, newton, grav, friction)
% Here, you'd implement your equations based on static conditions
% For each joint and link, calculate forces and moments ensuring sum of forces = 0 and sum of moments = 0

massABEH = Mechanism.Mass.ABEH;
massBCFG = Mechanism.Mass.BCFG;
massCDI = Mechanism.Mass.CDI;

massMoIABEH = Mechanism.MassMoI.ABEH;
massMoIBCFG = Mechanism.MassMoI.BCFG;
massMoICDI = Mechanism.MassMoI.CDI;

A_abeh = Mechanism.AngAcc.ABEH.(speedStr)(iter,:);
A_bcfg = Mechanism.AngAcc.BCFG.(speedStr)(iter,:);
A_cdi = Mechanism.AngAcc.CDI.(speedStr)(iter,:);

A_abeh_com = Mechanism.LinAcc.LinkCoM.ABEH.(speedStr)(iter,:);
A_bcfg_com = Mechanism.LinAcc.LinkCoM.BCFG.(speedStr)(iter,:);
A_cdi_com = Mechanism.LinAcc.LinkCoM.CDI.(speedStr)(iter,:);

% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

ABEH_com = LinkCoMPos.ABEH;
BCFG_com = LinkCoMPos.BCFG;
CDI_com = LinkCoMPos.CDI;

mu = 0.20;

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
wABEH=massABEH*g*grav;
wBCFG=massBCFG*g*grav;
wCDI=massCDI*g*grav;

% Unknown torque of the system
tT=[0 0 T];

% Torque provided by friction on Joint A
r_abeh_com_a = norm(ABEH_com - A);
r_bcfg_com_b = norm(BCFG_com - B);
r_cdi_com_c = norm(CDI_com - C);

A_noFriction_x = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.A(iter,1);
A_noFriction_y = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.A(iter,2);
B_noFriction_x = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.B(iter,1); 
B_noFriction_y = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.B(iter,2);
C_noFriction_x = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.C(iter,1); 
C_noFriction_y = Mechanism.ForceAnalysis.Newton.Grav.NoFriction.(speedStr).Joint.C(iter,2);

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

% Assuming r_abeh_com_a and r_bcfg_com_b are correctly calculated lever arms
T_fr_A = [0,0, mu * norm(F_normal_A) * r_abeh_com_a * friction]; % Torque due to friction at A
T_fr_B = [0,0, mu * norm(F_normal_B) * r_bcfg_com_b * friction]; % Torque due to friction at B
T_fr_C = [0,0, mu * norm(F_normal_B) * r_cdi_com_c * friction]; % Torque due to friction at B

%% FBD Equations
%Link ABEH
eqn1=fA+fB+wABEH+F_friction_A+F_friction_B==massABEH*A_abeh_com*newton;
eqn2=ForceSolverUtils.momentVec(A, ABEH_com, fA) + ForceSolverUtils.momentVec(B,  ABEH_com,fB)+tT+T_fr_A+T_fr_B==massMoIABEH * A_abeh*newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCFG
eqn3=-fB+fC+wBCFG-F_friction_B+F_friction_C==massBCFG*A_bcfg_com*newton;
eqn4=ForceSolverUtils.momentVec(B, BCFG_com, -fB)+ForceSolverUtils.momentVec(C, BCFG_com, fC)-T_fr_B+T_fr_C==massMoIBCFG * A_bcfg*newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDI
eqn5=-fC+fD+wCDI-F_friction_C==massCDI*A_cdi_com*newton;
eqn6=ForceSolverUtils.momentVec(C, CDI_com, -fC)+ForceSolverUtils.momentVec(D, CDI_com, fD)-T_fr_C==massMoICDI * A_cdi*newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,T]));
end