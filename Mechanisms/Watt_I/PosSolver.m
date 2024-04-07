function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(Mechanism.Joint.B - Mechanism.Joint.A);
% Link BCD
Mechanism.LinkLength.BC = norm(Mechanism.Joint.C - Mechanism.Joint.B);
Mechanism.LinkLength.BD = norm(Mechanism.Joint.D - Mechanism.Joint.B);
Mechanism.LinkLength.CD = norm(Mechanism.Joint.D - Mechanism.Joint.C);
% Link DE
Mechanism.LinkLength.DE = norm(Mechanism.Joint.E - Mechanism.Joint.D);
% Link EF
Mechanism.LinkLength.EF = norm(Mechanism.Joint.F - Mechanism.Joint.E);
% Link FCG
Mechanism.LinkLength.CF = norm(Mechanism.Joint.F - Mechanism.Joint.C);
Mechanism.LinkLength.CG = norm(Mechanism.Joint.G - Mechanism.Joint.C);
Mechanism.LinkLength.FG = norm(Mechanism.Joint.G - Mechanism.Joint.F);
end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);
G = Mechanism.Joint.G(1,:);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-circle intersections for C, D, E, F
C = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BC, G(1), G(2), Mechanism.LinkLength.CG, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2));
if isempty(C), valid = false; return; end

D = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BD, C(1), C(2), Mechanism.LinkLength.CD, Mechanism.Joint.D(iteration - 1, 1), Mechanism.Joint.D(iteration - 1, 2));
if isempty(D), valid = false; return; end

F = PosSolverUtils.circleCircleIntersection(C(1), C(2), Mechanism.LinkLength.CF, G(1), G(2), Mechanism.LinkLength.FG, Mechanism.Joint.F(iteration - 1, 1), Mechanism.Joint.F(iteration - 1, 2));
if isempty(F), valid = false; return; end

E = PosSolverUtils.circleCircleIntersection(D(1), D(2), Mechanism.LinkLength.DE, F(1), F(2), Mechanism.LinkLength.EF, Mechanism.Joint.E(iteration - 1, 1), Mechanism.Joint.E(iteration - 1, 2));
if isempty(E), valid = false; return; end

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;
Mechanism.Joint.D(iteration, :) = D;
Mechanism.Joint.E(iteration, :) = E;
Mechanism.Joint.F(iteration, :) = F;
Mechanism.Joint.G(iteration, :) = G;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.AB(iteration, :) = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BCD(iteration, :) = Utils.determineCoM([B; C; D]);
Mechanism.LinkCoM.DE(iteration, :) = Utils.determineCoM([D; E]);
Mechanism.LinkCoM.EF(iteration, :) = Utils.determineCoM([E; F]);
Mechanism.LinkCoM.CFG(iteration, :) = Utils.determineCoM([C; F; G]);

if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end
