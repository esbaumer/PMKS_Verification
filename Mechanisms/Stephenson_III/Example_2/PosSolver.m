function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(Mechanism.Joint.B - Mechanism.Joint.A);
% Link BCE
Mechanism.LinkLength.BC = norm(Mechanism.Joint.B - Mechanism.Joint.C);
Mechanism.LinkLength.BE = norm(Mechanism.Joint.B - Mechanism.Joint.E);
Mechanism.LinkLength.CE = norm(Mechanism.Joint.C - Mechanism.Joint.E);
% Link CD
Mechanism.LinkLength.CD = norm(Mechanism.Joint.C - Mechanism.Joint.D);
% Link EF
Mechanism.LinkLength.EF = norm(Mechanism.Joint.E - Mechanism.Joint.F);
% Link FG
Mechanism.LinkLength.FG = norm(Mechanism.Joint.F - Mechanism.Joint.G);
end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);
D = Mechanism.Joint.D(1, :);
G = Mechanism.Joint.G(1,:);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-circle intersections for C, E, F
C = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BC, D(1), D(2), Mechanism.LinkLength.CD, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2));
if isempty(C), valid = false; return; end

E = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BE, C(1), C(2), Mechanism.LinkLength.CE, Mechanism.Joint.E(iteration - 1, 1), Mechanism.Joint.E(iteration - 1, 2));
if isempty(D), valid = false; return; end

F = PosSolverUtils.circleCircleIntersection(E(1), E(2), Mechanism.LinkLength.EF, G(1), G(2), Mechanism.LinkLength.FG, Mechanism.Joint.F(iteration - 1, 1), Mechanism.Joint.F(iteration - 1, 2));
if isempty(F), valid = false; return; end

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
Mechanism.LinkCoM.BCE(iteration, :) = Utils.determineCoM([B; C; E]);
Mechanism.LinkCoM.CD(iteration, :) = Utils.determineCoM([C; D]);
Mechanism.LinkCoM.EF(iteration, :) = Utils.determineCoM([E; F]);
Mechanism.LinkCoM.FG(iteration, :) = Utils.determineCoM([F; G]);

if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end
