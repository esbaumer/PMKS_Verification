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
% Link BCEF
Mechanism.LinkLength.BC = norm(B - C);
Mechanism.LinkLength.BE = norm(B - E);
Mechanism.LinkLength.BF = norm(B - F);
Mechanism.LinkLength.CE = norm(C - E);
Mechanism.LinkLength.CF = norm(C - F);
Mechanism.LinkLength.EF = norm(E - F);
% Link CDGI
Mechanism.LinkLength.CD = norm(C - D);
Mechanism.LinkLength.CG = norm(C - G);
Mechanism.LinkLength.CI = norm(C - I);
Mechanism.LinkLength.DG = norm(D - G);
Mechanism.LinkLength.DI = norm(D - I);
Mechanism.LinkLength.GI = norm(G - I);

% Link ABH CoM with Joint A
Mechanism.LinkLength.ABH_CoM_A = norm(Mechanism.LinkCoM.ABH(1,:)- A);
Mechanism.LinkLength.ABH_CoM_B = norm(Mechanism.LinkCoM.ABH(1,:)- B);
% Link BCEF CoM with Joint B
Mechanism.LinkLength.BCEF_CoM_B = norm(Mechanism.LinkCoM.BCEF(1,:) - B);
Mechanism.LinkLength.BCEF_CoM_C = norm(Mechanism.LinkCoM.BCEF(1,:) - C);
% Link CDGI CoM with Joint C
Mechanism.LinkLength.CDGI_CoM_C = norm(Mechanism.LinkCoM.CDGI(1,:) - C);
Mechanism.LinkLength.CDGI_CoM_D = norm(Mechanism.LinkCoM.CDGI(1,:) - D);

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
E = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BE, C(1), C(2), Mechanism.LinkLength.CE, Mechanism.TracerPoint.E(iteration - 1, 1), Mechanism.TracerPoint.E(iteration - 1, 2));
if isempty(E), valid = false; return; end
F = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BF, C(1), C(2), Mechanism.LinkLength.CF, Mechanism.TracerPoint.F(iteration - 1, 1), Mechanism.TracerPoint.F(iteration - 1, 2));
if isempty(F), valid = false; return; end
G = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CG, D(1), D(2), Mechanism.LinkLength.DG, Mechanism.TracerPoint.G(iteration - 1, 1), Mechanism.TracerPoint.G(iteration - 1, 2));
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

Mechanism.LinkCoM.ABH(iteration, :) = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.ABH_CoM_A, B(1), B(2), Mechanism.LinkLength.ABH_CoM_B, Mechanism.LinkCoM.ABH(iteration - 1, 1), Mechanism.LinkCoM.ABH(iteration - 1, 2));
Mechanism.LinkCoM.BCEF(iteration, :) = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BCEF_CoM_B, C(1), C(2), Mechanism.LinkLength.BCEF_CoM_C, Mechanism.LinkCoM.BCEF(iteration - 1, 1), Mechanism.LinkCoM.BCEF(iteration - 1, 2));
Mechanism.LinkCoM.CDGI(iteration, :) = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CDGI_CoM_C, D(1), D(2), Mechanism.LinkLength.CDGI_CoM_D, Mechanism.LinkCoM.CDGI(iteration - 1, 1), Mechanism.LinkCoM.CDGI(iteration - 1, 2));

Mechanism.Angle.Link.ABH(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.ABH(iteration,2) - A(2), Mechanism.LinkCoM.ABH(iteration,1) - A(1)))];
Mechanism.Angle.Link.BCEF(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.BCEF(iteration,2) - B(2), Mechanism.LinkCoM.BCEF(iteration,1) - B(1)))];
Mechanism.Angle.Link.CDGI(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.CDGI(iteration,2) - C(2), Mechanism.LinkCoM.CDGI(iteration,1) - C(1)))];

% Define angles for each sensor
Mechanism.Angle.Joint.E(iteration, :) = [0 0 rad2deg(atan2(E(2) - B(2), E(1) - B(1)))];
Mechanism.Angle.Joint.F(iteration, :) = [0 0 rad2deg(atan2(F(2) - C(2), F(1) - C(1)))+180];
Mechanism.Angle.Joint.G(iteration, :) = [0 0 rad2deg(atan2(G(2) - B(2), G(1) - B(1)))];
Mechanism.Angle.Joint.H(iteration, :) = [0 0 rad2deg(atan2(H(2) - A(2), H(1) - A(1)))];
Mechanism.Angle.Joint.I(iteration, :) = [0 0 rad2deg(atan2(I(2) - C(2), I(1) - C(1)))];


for inputSpeedCol = 1:1:length(Mechanism.inputSpeed(1,:))
    if (forwardDir)
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol);
    else
        Mechanism.inputSpeed(iteration, inputSpeedCol) = Mechanism.inputSpeed(1, inputSpeedCol) * -1;
    end
end
iteration = iteration + 1;
end