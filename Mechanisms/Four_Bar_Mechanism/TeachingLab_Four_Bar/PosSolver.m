function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Points
A = Mechanism.Joint.A(1,:);
B = Mechanism.Joint.B(1,:);
C = Mechanism.Joint.C(1,:);
D = Mechanism.Joint.D(1,:);
E = Mechanism.TracerPoint.E(1,:);
F = Mechanism.TracerPoint.F(1,:);
G = Mechanism.TracerPoint.G(1,:);
H = Mechanism.TracerPoint.H(1,:);
I = Mechanism.TracerPoint.I(1,:);
% Distance Between Points
% Link ABH
Mechanism.LinkLength.AB = norm(A - B);
Mechanism.LinkLength.AH = norm(A - H);
Mechanism.LinkLength.BH = norm(B - H);
% Link BCFG
Mechanism.LinkLength.BC = norm(B - C);
Mechanism.LinkLength.BF = norm(B - F);
Mechanism.LinkLength.BG = norm(B - G);
Mechanism.LinkLength.CF = norm(C - F);
Mechanism.LinkLength.CG = norm(C - G);
Mechanism.LinkLength.FG = norm(F - G);
% Link CDEI
Mechanism.LinkLength.CD = norm(C - D);
Mechanism.LinkLength.CE = norm(C - E);
Mechanism.LinkLength.CI = norm(C - I);
Mechanism.LinkLength.DE = norm(D - E);
Mechanism.LinkLength.DI = norm(D - I);
Mechanism.LinkLength.EI = norm(E - I);

% Link ABH CoM with Joint A
Mechanism.LinkLength.ABH_CoM_A = norm(Mechanism.LinkCoM.ABH(1,:)- A);
Mechanism.LinkLength.ABH_CoM_B = norm(Mechanism.LinkCoM.ABH(1,:)- B);
% Link BCFG CoM with Joint B
Mechanism.LinkLength.BCFG_CoM_B = norm(Mechanism.LinkCoM.BCFG(1,:) - B);
Mechanism.LinkLength.BCFG_CoM_C = norm(Mechanism.LinkCoM.BCFG(1,:) - C);
% Link CDEI CoM with Joint C
Mechanism.LinkLength.CDEI_CoM_C = norm(Mechanism.LinkCoM.CDEI(1,:) - C);
Mechanism.LinkLength.CDEI_CoM_D = norm(Mechanism.LinkCoM.CDEI(1,:) - D);

end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);
D = Mechanism.Joint.D(1, :);

A_orig = Mechanism.Joint.A(1, :);
B_orig = Mechanism.Joint.B(1, :);
C_orig = Mechanism.Joint.C(1, :);
D_orig = Mechanism.Joint.D(1, :);
E_orig = Mechanism.TracerPoint.E(1, :);
F_orig = Mechanism.TracerPoint.F(1, :);
G_orig = Mechanism.TracerPoint.G(1, :);
H_orig = Mechanism.TracerPoint.H(1, :);
I_orig = Mechanism.TracerPoint.I(1, :);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-circle intersections for C, E, F, G
C = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BC, D(1), D(2), Mechanism.LinkLength.CD, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2));
if isempty(C), valid = false; return; end
% E = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BE, C(1), C(2), Mechanism.LinkLength.CE, Mechanism.TracerPoint.E(iteration - 1, 1), Mechanism.TracerPoint.E(iteration - 1, 2));
% if isempty(E), valid = false; return; end
% F = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BF, C(1), C(2), Mechanism.LinkLength.CF, Mechanism.TracerPoint.F(iteration - 1, 1), Mechanism.TracerPoint.F(iteration - 1, 2));
% if isempty(F), valid = false; return; end
% G = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CG, D(1), D(2), Mechanism.LinkLength.DG, Mechanism.TracerPoint.G(iteration - 1, 1), Mechanism.TracerPoint.G(iteration - 1, 2));
% if isempty(G), valid = false; return; end
% H = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.AH, B(1), B(2), Mechanism.LinkLength.BH, Mechanism.TracerPoint.H(iteration - 1, 1), Mechanism.TracerPoint.H(iteration - 1, 2));
% if isempty(H), valid = false; return; end
% I = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CI, D(1), D(2), Mechanism.LinkLength.DI, Mechanism.TracerPoint.I(iteration - 1, 1), Mechanism.TracerPoint.I(iteration - 1, 2));
% if isempty(I), valid = false; return; end
% E = PosSolverUtils.determineTracerJoint(B_orig, C_orig, E_orig, B, C);
% F = PosSolverUtils.determineTracerJoint(B_orig, C_orig, F_orig, B, C);
% G = PosSolverUtils.determineTracerJoint(C_orig, D_orig, G_orig, C, D);
% H = PosSolverUtils.determineTracerJoint(A_orig, B_orig, H_orig, A, B);
% I = PosSolverUtils.determineTracerJoint(C_orig, D_orig, I_orig, C, D);
% Get the closest intersection point index
EIndex = PosSolverUtils.circleCircleIntersectionSelect(C_orig(1), C_orig(2), Mechanism.LinkLength.CE, D_orig(1), D_orig(2), Mechanism.LinkLength.DE, E_orig(1), E_orig(2));
FIndex = PosSolverUtils.circleCircleIntersectionSelect(B_orig(1), B_orig(2), Mechanism.LinkLength.BF, C_orig(1), C_orig(2), Mechanism.LinkLength.CF, F_orig(1), F_orig(2));
GIndex = PosSolverUtils.circleCircleIntersectionSelect(B_orig(1), B_orig(2), Mechanism.LinkLength.BG, C_orig(1), C_orig(2), Mechanism.LinkLength.CG, G_orig(1), G_orig(2));
HIndex = PosSolverUtils.circleCircleIntersectionSelect(A_orig(1), A_orig(2), Mechanism.LinkLength.AH, B_orig(1), B_orig(2), Mechanism.LinkLength.BH, H_orig(1), H_orig(2));
IIndex = PosSolverUtils.circleCircleIntersectionSelect(C_orig(1), C_orig(2), Mechanism.LinkLength.CI, D_orig(1), D_orig(2), Mechanism.LinkLength.DI, I_orig(1), I_orig(2));


% Now retrieve the final intersection point based on the selected index
[E, ~] = PosSolverUtils.circleCircleIntersectionWithPrevious(C(1), C(2), Mechanism.LinkLength.CE, D(1), D(2), Mechanism.LinkLength.DE, EIndex);
if isempty(E), valid = false; return; end

[F, ~] = PosSolverUtils.circleCircleIntersectionWithPrevious(B(1), B(2), Mechanism.LinkLength.BF, C(1), C(2), Mechanism.LinkLength.CF, FIndex);
if isempty(F), valid = false; return; end

[G, ~] = PosSolverUtils.circleCircleIntersectionWithPrevious(B(1), B(2), Mechanism.LinkLength.BG, C(1), C(2), Mechanism.LinkLength.CG, GIndex);
if isempty(G), valid = false; return; end

[H, ~] = PosSolverUtils.circleCircleIntersectionWithPrevious(A(1), A(2), Mechanism.LinkLength.AH, B(1), B(2), Mechanism.LinkLength.BH, HIndex);
if isempty(H), valid = false; return; end

[I, ~] = PosSolverUtils.circleCircleIntersectionWithPrevious(C(1), C(2), Mechanism.LinkLength.CI, D(1), D(2), Mechanism.LinkLength.DI, IIndex);
if isempty(I), valid = false; return; end

% E = PosSolverUtils.determineTracerJoint(B(1), B(2), Mechanism.LinkLength.BE, C(1), C(2), Mechanism.LinkLength.CE, Mechanism.TracerPoint.E(iteration - 1, 1), Mechanism.TracerPoint.E(iteration - 1, 2));
% F = PosSolverUtils.determineTracerJoint(B(1), B(2), Mechanism.LinkLength.BF, C(1), C(2), Mechanism.LinkLength.CF, Mechanism.TracerPoint.F(iteration - 1, 1), Mechanism.TracerPoint.F(iteration - 1, 2));
% G = PosSolverUtils.determineTracerJoint(C(1), C(2), Mechanism.LinkLength.CG, D(1), D(2), Mechanism.LinkLength.DG, Mechanism.TracerPoint.G(iteration - 1, 1), Mechanism.TracerPoint.G(iteration - 1, 2));
% H = PosSolverUtils.determineTracerJoint(A(1), A(2), Mechanism.LinkLength.AH, B(1), B(2), Mechanism.LinkLength.BH, Mechanism.TracerPoint.H(iteration - 1, 1), Mechanism.TracerPoint.H(iteration - 1, 2));
% I = PosSolverUtils.determineTracerJoint(C(1), C(2), Mechanism.LinkLength.CI, D(1), D(2), Mechanism.LinkLength.DI, Mechanism.TracerPoint.I(iteration - 1, 1), Mechanism.TracerPoint.I(iteration - 1, 2));

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;
Mechanism.Joint.D(iteration, :) = D;
Mechanism.TracerPoint.E(iteration, :) = E;
Mechanism.TracerPoint.F(iteration, :) = F;
Mechanism.TracerPoint.G(iteration, :) = G;
Mechanism.TracerPoint.H(iteration, :) = H;
Mechanism.TracerPoint.I(iteration, :) = I;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.ABH(iteration, :) = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.ABH_CoM_A, B(1), B(2), Mechanism.LinkLength.ABH_CoM_B, Mechanism.LinkCoM.ABH(iteration - 1, 1), Mechanism.LinkCoM.ABH(iteration - 1, 2));
Mechanism.LinkCoM.BCFG(iteration, :) = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BCFG_CoM_B, C(1), C(2), Mechanism.LinkLength.BCFG_CoM_C, Mechanism.LinkCoM.BCFG(iteration - 1, 1), Mechanism.LinkCoM.BCFG(iteration - 1, 2));
Mechanism.LinkCoM.CDEI(iteration, :) = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CDEI_CoM_C, D(1), D(2), Mechanism.LinkLength.CDEI_CoM_D, Mechanism.LinkCoM.CDEI(iteration - 1, 1), Mechanism.LinkCoM.CDEI(iteration - 1, 2));

Mechanism.Angle.Link.ABH(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.ABH(iteration,2) - A(2), Mechanism.LinkCoM.ABH(iteration,1) - A(1)))];
Mechanism.Angle.Link.BCFG(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.BCFG(iteration,2) - B(2), Mechanism.LinkCoM.BCFG(iteration,1) - B(1)))];
Mechanism.Angle.Link.CDEI(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.CDEI(iteration,2) - C(2), Mechanism.LinkCoM.CDEI(iteration,1) - C(1)))];

% Define angles for each sensor
Mechanism.Angle.Joint.E(iteration, :) = [0 0 rad2deg(atan2(E(2) - D(2), E(1) - D(1)))];
Mechanism.Angle.Joint.F(iteration, :) = [0 0 rad2deg(atan2(F(2) - C(2), F(1) - C(1)))+180];
Mechanism.Angle.Joint.G(iteration, :) = [0 0 rad2deg(atan2(G(2) - B(2), G(1) - B(1)))];
Mechanism.Angle.Joint.H(iteration, :) = [0 0 rad2deg(atan2(H(2) - A(2), H(1) - A(1)))];
Mechanism.Angle.Joint.I(iteration, :) = [0 0 rad2deg(atan2(I(2) - D(2), I(1) - D(1)))];


for inputSpeedCol = 1:1:length(Mechanism.inputSpeed(1,:))
    if (forwardDir)
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol);
    else
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol) * -1;
    end
end
iteration = iteration + 1;
end