function Mechanism = ForceSolver(Mechanism, scenarios)
    Mechanism = ForceSolverUtils.ForceSolver(Mechanism, scenarios, @performForceAnalysis);
end

function solution = performForceAnalysis(Mechanism, iter, speedStr, JointPos, LinkCoMPos, newton, grav, friction)
% Here, you'd implement your equations based on static conditions
% For each joint and link, calculate forces and moments ensuring sum of forces = 0 and sum of moments = 0

massABH = Mechanism.Mass.ABH;
massBCEF = Mechanism.Mass.BCEF;
massCDGI = Mechanism.Mass.CDGI;

massMoIABH = Mechanism.MassMoI.ABH;
massMoIBCEF = Mechanism.MassMoI.BCEF;
massMoICDGI = Mechanism.MassMoI.CDGI;

A_abeh = Mechanism.AngAcc.ABH.(speedStr)(iter,:);
A_bcfg = Mechanism.AngAcc.BCEF.(speedStr)(iter,:);
A_cdgi = Mechanism.AngAcc.CDGI.(speedStr)(iter,:);

A_abeh_com = Mechanism.LinAcc.LinkCoM.ABH.(speedStr)(iter,:);
A_bcfg_com = Mechanism.LinAcc.LinkCoM.BCEF.(speedStr)(iter,:);
A_cdgi_com = Mechanism.LinAcc.LinkCoM.CDGI.(speedStr)(iter,:);

% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;

ABH_com = LinkCoMPos.ABH;
BCEF_com = LinkCoMPos.BCEF;
CDGI_com = LinkCoMPos.CDGI;

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
wABH=massABH*g*grav;
wBCEF=massBCEF*g*grav;
wCDGI=massCDGI*g*grav;

% Unknown torque of the system
tT=[0 0 T];

% Torque provided by friction on Joint A
r_abeh_com_a = norm(ABH_com - A);
r_bcfg_com_b = norm(BCEF_com - B);
r_cdgi_com_c = norm(CDGI_com - C);

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
T_fr_C = [0,0, mu * norm(F_normal_B) * r_cdgi_com_c * friction]; % Torque due to friction at B

%% FBD Equations
%Link ABH
eqn1=fA+fB+wABH+F_friction_A+F_friction_B==massABH*A_abeh_com*newton;
eqn2=ForceSolverUtils.momentVec(A, ABH_com, fA) + ForceSolverUtils.momentVec(B,  ABH_com,fB)+tT+T_fr_A+T_fr_B==massMoIABH * A_abeh*newton; %only change the ==0 appropriately for newtons 2nd law
%Link BCEF
eqn3=-fB+fC+wBCEF-F_friction_B+F_friction_C==massBCEF*A_bcfg_com*newton;
eqn4=ForceSolverUtils.momentVec(B, BCEF_com, -fB)+ForceSolverUtils.momentVec(C, BCEF_com, fC)-T_fr_B+T_fr_C==massMoIBCEF * A_bcfg*newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDGI
eqn5=-fC+fD+wCDGI-F_friction_C==massCDGI*A_cdgi_com*newton;
eqn6=ForceSolverUtils.momentVec(C, CDGI_com, -fC)+ForceSolverUtils.momentVec(D, CDGI_com, fD)-T_fr_C==massMoICDGI * A_cdgi*newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,T]));
end