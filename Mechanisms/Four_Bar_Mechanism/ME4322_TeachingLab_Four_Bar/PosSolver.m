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
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(B - A);
% Link BCEF
Mechanism.LinkLength.BC = norm(B - C);
Mechanism.LinkLength.BE = norm(B - E);
Mechanism.LinkLength.BF = norm(B - F);
Mechanism.LinkLength.CE = norm(C - E);
Mechanism.LinkLength.CF = norm(C - F);
Mechanism.LinkLength.EF = norm(E - F);
% Link CDG
Mechanism.LinkLength.CD = norm(C - D);
Mechanism.LinkLength.CG = norm(C - G);
Mechanism.LinkLength.DG = norm(D - G);
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

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;
Mechanism.Joint.D(iteration, :) = D;
Mechanism.TracerPoint.E(iteration, :) = E;
Mechanism.TracerPoint.F(iteration, :) = F;
Mechanism.TracerPoint.G(iteration, :) = G;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.AB(iteration, :) = GeneralUtils.determineCoM([A; B]);
Mechanism.LinkCoM.BCEF(iteration, :) = GeneralUtils.determineCoM([B; C; E; F]);
Mechanism.LinkCoM.CDG(iteration, :) = GeneralUtils.determineCoM([C; D; G]);

if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end