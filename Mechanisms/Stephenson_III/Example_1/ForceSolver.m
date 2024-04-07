function Mechanism = ForceSolver(Mechanism, scenarios)
    Mechanism = ForceSolverUtils.ForceSolver(Mechanism, scenarios, @performForceAnalysis);
end

function solution = performForceAnalysis(Mechanism, iter, JointPos, LinkCoMPos, newton, grav, friction)
% Here, you'd implement your equations based on static conditions
% For each joint and link, calculate forces and moments ensuring sum of forces = 0 and sum of moments = 0

massAB = Mechanism.Mass.AB;
massBC = Mechanism.Mass.BC;
massCDE = Mechanism.Mass.CDE;
massEF = Mechanism.Mass.EF;
massFG = Mechanism.Mass.FG;

massMoIAB = Mechanism.MassMoI.AB;
massMoIBC = Mechanism.MassMoI.BC;
massMoICDE = Mechanism.MassMoI.CDE;
massMoIEF = Mechanism.MassMoI.EF;
massMoIFG = Mechanism.MassMoI.FG;

A_ab = Mechanism.AngAcc.AB(iter,:);
A_bc = Mechanism.AngAcc.BC(iter,:);
A_cde = Mechanism.AngAcc.CDE(iter,:);
A_ef = Mechanism.AngAcc.EF(iter,:);
A_fg = Mechanism.AngAcc.FG(iter,:);

A_ab_com = Mechanism.LinAcc.LinkCoM.AB(iter,:);
A_bc_com = Mechanism.LinAcc.LinkCoM.BC(iter,:);
A_cde_com = Mechanism.LinAcc.LinkCoM.CDE(iter,:);
A_ef_com = Mechanism.LinAcc.LinkCoM.EF(iter,:);
A_fg_com = Mechanism.LinAcc.LinkCoM.FG(iter,:);

% This is a placeholder for the actual static analysis logic
% You'll need to adapt this to your specific requirements
A = JointPos.A;
B = JointPos.B;
C = JointPos.C;
D = JointPos.D;
E = JointPos.E;
F = JointPos.F;
G = JointPos.G;
H = Mechanism.TracerPoint.H(iter, :);

AB_com = LinkCoMPos.AB;
BC_com = LinkCoMPos.BC;
CDE_com = LinkCoMPos.CDE;
EF_com = LinkCoMPos.EF;
FG_com = LinkCoMPos.FG;

syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy T

g = [0 -9.81 0]; %defining gravity to find weight of each link m/s^2

% Forces at each joint
fA=[Ax Ay 0];
fB=[Bx By 0];
fC=[Cx Cy 0];
fD=[Dx Dy 0];
fE=[Ex Ey 0];
fF=[Fx Fy 0];
fG=[Gx Gy 0];

% Weight of each link
wAB=massAB *g * grav;
wBC=massBC *g * grav;
wCDE=massCDE *g * grav;
wEF=massEF *g * grav;
wFG=massFG *g * grav;

% Unknown torque of the system
tT=[0 0 T];

% Force of the applied load
LoadForce=[0 -200 0];
LoadPos = H;

%% Static Equilibrium Equations
%Link AB
eqn1=fA+fB+wAB==massAB*A_ab_com * newton;
eqn2=ForceSolverUtils.momentVec(A, AB_com, fA) + ForceSolverUtils.momentVec(B,  AB_com,fB)+tT==massMoIAB * A_ab * newton; %only change the ==0 appropriately for newtons 2nd law
%Link BC
eqn3=-fB+fC+wBC==massBC*A_bc_com * newton;
eqn4=ForceSolverUtils.momentVec(B, BC_com, -fB)+ForceSolverUtils.momentVec(C, BC_com, fC) + ForceSolverUtils.momentVec(D, BC_com, fD) ==massMoIBC * A_bc * newton; %only change the ==0 appropriately for newtons 2nd law
%Link CDE
eqn5=-fC+fD+fE+wCDE==massCDE*A_cde_com * newton;
eqn6=ForceSolverUtils.momentVec(C, CDE_com, -fC)+ForceSolverUtils.momentVec(D, CDE_com, fD)+ForceSolverUtils.momentVec(E, CDE_com, fE)==massMoICDE * A_cde * newton; %only change the ==0 appropriately for newtons 2nd law
%Link EF
eqn7=-fE+fF+wEF==massEF*A_ef_com * newton;
eqn8=ForceSolverUtils.momentVec(E, EF_com, -fE)+ForceSolverUtils.momentVec(F, EF_com, fF)==massMoIEF * A_ef * newton; %only change the ==0 appropriately for newtons 2nd law
%Link FG
eqn9=-fF+fG+wFG+LoadForce==massFG*A_fg_com * newton;
eqn10=ForceSolverUtils.momentVec(F, FG_com, -fF)+ForceSolverUtils.momentVec(G, FG_com, fG)+ForceSolverUtils.momentVec(LoadPos, FG_com, LoadForce)==massMoIFG * A_fg * newton; %only change the ==0 appropriately for newtons 2nd law

solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,T]));
end
