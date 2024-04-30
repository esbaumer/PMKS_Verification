function Mechanism = PosSolver(Mechanism, input_speed)
    Mechanism = PosSolverUtils.PosSolver(Mechanism, input_speed, @calculateDistances, @calculateNewPositions);
end

% Function to calculate distances between joints
function Mechanism = calculateDistances(Mechanism)
% Points
A = Mechanism.Joint.A(1,:);
B = Mechanism.Joint.B(1,:);
C = Mechanism.Joint.C(1,:);
E = Mechanism.TracerPoint.E(1,:);
% Distance Between Points
% Link AB
Mechanism.LinkLength.AB = norm(A - B);
% Link BCE
Mechanism.LinkLength.BC = norm(B - C);
Mechanism.LinkLength.BE = norm(B - E);
Mechanism.LinkLength.CE = norm(C - E);
% Link AB CoM with Joint A
Mechanism.LinkLength.AB_CoM_A = norm(Mechanism.LinkCoM.AB(1,:)- A);
Mechanism.LinkLength.AB_CoM_B = norm(Mechanism.LinkCoM.AB(1,:)- B);
% Link BCE CoM with Joint B
Mechanism.LinkLength.BCE_CoM_B = norm(Mechanism.LinkCoM.BCE(1,:)- B);
Mechanism.LinkLength.BCE_CoM_C = norm(Mechanism.LinkCoM.BCE(1,:)- C);
end

% Function to calculate new positions for the joints
function [Mechanism, valid, theta, iteration] = calculateNewPositions(Mechanism, theta, iteration, forwardDir)
% Initialize validity flag
valid = true;

A = Mechanism.Joint.A(1, :);


% Direct calculation for B
B = [A(1) + Mechanism.LinkLength.AB * cos(theta), A(2) + Mechanism.LinkLength.AB * sin(theta), 0];

% Circle-line intersections for C
C = PosSolverUtils.circleLineIntersection(B(1), B(2), Mechanism.LinkLength.BC, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2), 0);
if isempty(C), valid = false; return; end

E = B;

% Update positions
Mechanism.Joint.A(iteration, :) = A;
Mechanism.Joint.B(iteration, :) = B;
Mechanism.Joint.C(iteration, :) = C;
Mechanism.TracerPoint.E(iteration, :) = E;

utilsFolderPath = fullfile(pwd);
addpath(utilsFolderPath);

Mechanism.LinkCoM.AB(iteration, :) = PosSolverUtils.circleCircleIntersection(A(1), A(2), Mechanism.LinkLength.AB_CoM_A, Mechanism.Joint.B(iteration - 1, 1), Mechanism.Joint.B(iteration - 1, 2), Mechanism.LinkLength.AB_CoM_B, Mechanism.LinkCoM.AB(iteration - 1, 1), Mechanism.LinkCoM.AB(iteration - 1, 2));
Mechanism.LinkCoM.BCE(iteration, :) = PosSolverUtils.circleCircleIntersection(B(1), B(2), Mechanism.LinkLength.BCE_CoM_B, Mechanism.Joint.C(iteration - 1, 1), Mechanism.Joint.C(iteration - 1, 2), Mechanism.LinkLength.BCE_CoM_C, Mechanism.LinkCoM.BCE(iteration - 1, 1), Mechanism.LinkCoM.BCE(iteration - 1, 2));

Mechanism.Angle.AB(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.AB(iteration,2) - A(2), Mechanism.LinkCoM.AB(iteration,1) - A(1)))];
Mechanism.Angle.BCE(iteration, :) = [0,0,rad2deg(atan2(Mechanism.LinkCoM.BCE(iteration,2) - B(2), Mechanism.LinkCoM.BCE(iteration,1) - B(1)))];


if (forwardDir)
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1);
else
    Mechanism.inputSpeed(iteration) = Mechanism.inputSpeed(1) * -1;
end
iteration = iteration + 1;
end
