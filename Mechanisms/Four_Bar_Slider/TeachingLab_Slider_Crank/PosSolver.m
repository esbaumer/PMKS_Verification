function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(Mechanism.Joint.A(1,:) - Mechanism.Joint.B(1,:));
% Link BC
Mechanism.LinkLength.BC = norm(Mechanism.Joint.B(1,:) - Mechanism.Joint.C(1,:));
end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);

% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-line intersections for C
C = circleLineIntersection(B(1), B(2), Mechanism.LinkLength.BC, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2), 0);
if isempty(C), valid = false; return; end

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.AB(iteration, :) = Utils.determineCoM([A; B]);
Mechanism.LinkCoM.BC(iteration, :) = Utils.determineCoM([B; C]);

if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end
