function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Points
A = Mechanism.Joint.A;
B = Mechanism.Joint.B;
C = Mechanism.Joint.C;
D = Mechanism.Joint.D;
E = Mechanism.TracerPoint.E;
F = Mechanism.TracerPoint.F;
G = Mechanism.TracerPoint.G;
H = Mechanism.TracerPoint.H;
I = Mechanism.TracerPoint.I;
% Distance Between Points
% Link ABEH
Mechanism.LinkLength.AB = norm(A - B);
Mechanism.LinkLength.AE = norm(A - E);
Mechanism.LinkLength.AH = norm(A - H);
Mechanism.LinkLength.BE = norm(B - E);
Mechanism.LinkLength.BH = norm(B - H);
Mechanism.LinkLength.EH = norm(E - H);
% Link BCFG
Mechanism.LinkLength.BC = norm(B - C);
Mechanism.LinkLength.BF = norm(B - F);
Mechanism.LinkLength.BG = norm(B - G);
Mechanism.LinkLength.CF = norm(C - F);
Mechanism.LinkLength.CG = norm(C - G);
Mechanism.LinkLength.FG = norm(F - G);
% Link CDI
Mechanism.LinkLength.CD = norm(C - D);
Mechanism.LinkLength.CI = norm(C - I);
Mechanism.LinkLength.DI = norm(D - I);
% Link ABEH CoM with Joint A
Mechanism.LinkLength.ABEH_CoM_A = norm(Mechanism.LinkCoM.ABEH(1,:)- A);
Mechanism.LinkLength.ABEH_CoM_B = norm(Mechanism.LinkCoM.ABEH(1,:)- B);
% Link BCFG CoM with Joint B
Mechanism.LinkLength.BCFG_CoM_B = norm(Mechanism.LinkCoM.BCFG(1,:) - B);
Mechanism.LinkLength.BCFG_CoM_C = norm(Mechanism.LinkCoM.BCFG(1,:) - C);
% Link CDI CoM with Joint C
Mechanism.LinkLength.CDI_CoM_C = norm(Mechanism.LinkCoM.CDI(1,:) - C);
Mechanism.LinkLength.CDI_CoM_D = norm(Mechanism.LinkCoM.CDI(1,:) - D);

end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);
D = Mechanism.Joint.D(1, :);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-circle intersections for C, E, F, G
C = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BC, D(1), D(2), Mechanism.LinkLength.CD, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2));
if isempty(C), valid = false; return; end
E = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.AE, B(1), B(2), Mechanism.LinkLength.BE, Mechanism.TracerPoint.E(iteration - 1, 1), Mechanism.TracerPoint.E(iteration - 1, 2));
if isempty(E), valid = false; return; end
F = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BF, C(1), C(2), Mechanism.LinkLength.CF, Mechanism.TracerPoint.F(iteration - 1, 1), Mechanism.TracerPoint.F(iteration - 1, 2));
if isempty(F), valid = false; return; end
G = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BG, C(1), C(2), Mechanism.LinkLength.CG, Mechanism.TracerPoint.G(iteration - 1, 1), Mechanism.TracerPoint.G(iteration - 1, 2));
if isempty(G), valid = false; return; end
H = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.AH, B(1), B(2), Mechanism.LinkLength.BH, Mechanism.TracerPoint.H(iteration - 1, 1), Mechanism.TracerPoint.H(iteration - 1, 2));
if isempty(H), valid = false; return; end
I = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CI, D(1), D(2), Mechanism.LinkLength.DI, Mechanism.TracerPoint.I(iteration - 1, 1), Mechanism.TracerPoint.I(iteration - 1, 2));
if isempty(I), valid = false; return; end

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

Mechanism.LinkCoM.ABEH(iteration, :) = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.ABEH_CoM_A, B(1), B(2), Mechanism.LinkLength.ABEH_CoM_B, Mechanism.LinkCoM.ABEH(iteration - 1, 2), Mechanism.LinkCoM.ABEH(iteration - 1, 1));
Mechanism.LinkCoM.BCFG(iteration, :) = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BCFG_CoM_B, C(1), C(2), Mechanism.LinkLength.BCFG_CoM_C, Mechanism.LinkCoM.BCFG(iteration - 1, 1), Mechanism.LinkCoM.BCFG(iteration - 1, 2));
Mechanism.LinkCoM.CDI(iteration, :) = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CDI_CoM_C, D(1), D(2), Mechanism.LinkLength.CDI_CoM_D, Mechanism.LinkCoM.CDI(iteration - 1, 1), Mechanism.LinkCoM.CDI(iteration - 1, 2));

Mechanism.Angle.ABEH(iteration, :) = [0,0, [atan2(Mechanism.LinkCoM.ABEH(iteration,2) - A(2), Mechanism.LinkCoM.ABEH(iteration,1) - A(1))]];
Mechanism.Angle.BCFG(iteration, :) = [0,0, [atan2(Mechanism.LinkCoM.BCFG(iteration,2) - B(2), Mechanism.LinkCoM.BCFG(iteration,1) - B(1))]];
Mechanism.Angle.CDI(iteration, :) = [0,0, [atan2(Mechanism.LinkCoM.CDI(iteration,2) - C(2), Mechanism.LinkCoM.CDI(iteration,1) - C(1))]];


for inputSpeedCol = 1:1:length(Mechanism.inputSpeed(1,:))
    if (forwardDir)
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol);
    else
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol) * -1;
    end
end
iteration = iteration + 1;
end