close all
clear all
clc;

load('Kin/LinVel/Joint/A')
load('Kin/LinVel/Joint/B')
load('Kin/LinVel/Joint/C')
load('Kin/LinVel/Joint/D')
load('Kin/LinVel/Joint/E')
load('Kin/LinVel/Joint/F')
load('Kin/LinVel/Joint/G')

load('Kin/LinVel/Link/AB')
load('Kin/LinVel/Link/BC')
load('Kin/LinVel/Link/DCE')
load('Kin/LinVel/Link/EF')
load('Kin/LinVel/Link/FG')

load('Kin/AngVel/AB')
load('Kin/AngVel/BC')
load('Kin/AngVel/DCE')
load('Kin/AngVel/EF')
load('Kin/AngVel/FG')

load('Kin/LinAcc/Joint/A')
load('Kin/LinAcc/Joint/B')
load('Kin/LinAcc/Joint/C')
load('Kin/LinAcc/Joint/D')
load('Kin/LinAcc/Joint/E')
load('Kin/LinAcc/Joint/F')
load('Kin/LinAcc/Joint/G')

load('Kin/LinAcc/Link/AB')
load('Kin/LinAcc/Link/BC')
load('Kin/LinAcc/Link/DCE')
load('Kin/LinAcc/Link/EF')
load('Kin/LinAcc/Link/FG')

load('Kin/AngAcc/AB')
load('Kin/AngAcc/BC')
load('Kin/AngAcc/DCE')
load('Kin/AngAcc/EF')
load('Kin/AngAcc/FG')

timestep = 1:1:1*10;
timestep= timestep.';

% Plotting Velocities of Each Joint
plotKinematicData([timestep, V_a], 1, [4, 2], 1, 'Velocity of Joint A', 'x', 'Velocity', 'r');
plotKinematicData([timestep, V_b], 1, [4, 2], 2, 'Velocity of Joint B', 'x', 'Velocity', 'g');
plotKinematicData([timestep, V_c], 1, [4, 2], 3, 'Velocity of Joint C', 'x', 'Velocity', 'b');
plotKinematicData([timestep, V_d], 1, [4, 2], 4, 'Velocity of Joint D', 'x', 'Velocity', 'k');
plotKinematicData([timestep, V_e], 1, [4, 2], 5, 'Velocity of Joint E', 'x', 'Velocity', 'm');
plotKinematicData([timestep, V_f], 1, [4, 2], 6, 'Velocity of Joint F', 'x', 'Velocity', 'c');
plotKinematicData([timestep, V_g], 1, [4, 2], 7, 'Velocity of Joint G', 'x', 'Velocity', 'b');

% Plotting Accelerations of Each Joint
plotKinematicData([timestep, A_a], 2, [4, 2], 1, 'Acceleration of Joint A', 'x', 'Velocity', 'r');
plotKinematicData([timestep, A_b], 2, [4, 2], 2, 'Acceleration of Joint B', 'x', 'Velocity', 'g');
plotKinematicData([timestep, A_c], 2, [4, 2], 3, 'Acceleration of Joint C', 'x', 'Velocity', 'b');
plotKinematicData([timestep, A_d], 2, [4, 2], 4, 'Acceleration of Joint D', 'x', 'Velocity', 'k');
plotKinematicData([timestep, A_e], 2, [4, 2], 5, 'Acceleration of Joint E', 'x', 'Velocity', 'm');
plotKinematicData([timestep, A_f], 2, [4, 2], 6, 'Acceleration of Joint F', 'x', 'Velocity', 'c');
plotKinematicData([timestep, A_g], 2, [4, 2], 7, 'Acceleration of Joint G', 'x', 'Velocity', 'b');

% Plotting Velocities at Link's Center of Mass
plotKinematicData([timestep, V_ab_com], 3, [3, 2], 1, 'Velocity of Link AB', 'x', 'Velocity', 'r');
plotKinematicData([timestep, V_bc_com], 3, [3, 2], 2, 'Velocity of Link BC', 'x', 'Velocity', 'g');
plotKinematicData([timestep, V_dce_com], 3, [3, 2], 3, 'Velocity of Link DCE', 'x', 'Velocity', 'b');
plotKinematicData([timestep, V_ef_com], 3, [3, 2], 4, 'Velocity of Link EF', 'x', 'Velocity', 'k');
plotKinematicData([timestep, V_fg_com], 3, [3, 2], 5, 'Velocity of Link FG', 'x', 'Velocity', 'm');

% Plotting Accelerations at Link's Center of Mass
plotKinematicData([timestep, A_ab_com], 4, [3, 2], 1, 'Acceleration of Link AB', 'x', 'Acceleration', 'r');
plotKinematicData([timestep, A_bc_com], 4, [3, 2], 2, 'Acceleration of Link BC', 'x', 'Acceleration', 'g');
plotKinematicData([timestep, A_dce_com], 4, [3, 2], 3, 'Acceleration of Link DCE', 'x', 'Acceleration', 'b');
plotKinematicData([timestep, A_ef_com], 4, [3, 2], 4, 'Acceleration of Link EF', 'x', 'Acceleration', 'k');
plotKinematicData([timestep, A_fg_com], 4, [3, 2], 5, 'Acceleration of Link FG', 'x', 'Acceleration', 'm');% ... and so on for other links

% Plotting Internal Forces and Input Torque
load('Stat/Joint_A')
load('Stat/Joint_B')
load('Stat/Joint_C')
load('Stat/Joint_D')
load('Stat/Joint_E')
load('Stat/Joint_F')
load('Stat/Joint_G')
load('Stat/Torque')

% Plotting Internal Forces and Input Torque
plotForceData(5, [4, 2], 1, [timestep, F_a_vec], 'r', 'Internal Force at Joint A', 'x', 'Force');
plotForceData(5, [4, 2], 2, [timestep, F_b_vec], 'g', 'Internal Force at Joint B', 'x', 'Force');
plotForceData(5, [4, 2], 3, [timestep, F_c_vec], 'b', 'Internal Force at Joint C', 'x', 'Force');
plotForceData(5, [4, 2], 4, [timestep, F_d_vec], 'k', 'Internal Force at Joint D', 'x', 'Force');
plotForceData(5, [4, 2], 5, [timestep, F_e_vec], 'm', 'Internal Force at Joint E', 'x', 'Force');
plotForceData(5, [4, 2], 6, [timestep, F_f_vec], 'c', 'Internal Force at Joint F', 'x', 'Force');
plotForceData(5, [4, 2], 7, [timestep, F_g_vec], 'b', 'Internal Force at Joint G', 'x', 'Force');
plotForceData(5, [4, 2], 8, [timestep, Torque_vec], 'b', 'Input Torque', 'x', 'Torque');

% Finding max values
[indexA_stat, FA_stat_max] = findMaxMagnitude(F_a_vec, 'F_a_stat');
[indexB_stat, FB_stat_max] = findMaxMagnitude(F_b_vec, 'F_b_stat');
[indexC_stat, FC_stat_max] = findMaxMagnitude(F_c_vec, 'F_c_stat');
[indexD_stat, FD_stat_max] = findMaxMagnitude(F_d_vec, 'F_d_stat');
[indexE_stat, FE_stat_max] = findMaxMagnitude(F_e_vec, 'F_e_stat');
[indexF_stat, FF_stat_max] = findMaxMagnitude(F_f_vec, 'F_f_stat');
[indexG_stat, FG_stat_max] = findMaxMagnitude(F_g_vec, 'F_g_stat');

% Finding min values
[indexA_stat_min, FA_stat_min] = findMinMagnitude(F_a_vec, 'F_a_stat');
[indexB_stat_min, FB_stat_min] = findMinMagnitude(F_b_vec, 'F_b_stat');
[indexC_stat_min, FC_stat_min] = findMinMagnitude(F_c_vec, 'F_c_stat');
[indexD_stat_min, FD_stat_min] = findMinMagnitude(F_d_vec, 'F_d_stat');
[indexE_stat_min, FE_stat_min] = findMinMagnitude(F_e_vec, 'F_e_stat');
[indexF_stat_min, FF_stat_min] = findMinMagnitude(F_f_vec, 'F_f_stat');
[indexG_stat_min, FG_stat_min] = findMinMagnitude(F_g_vec, 'F_g_stat');

%cl

% Plotting Internal Forces and Input Torque based from Newton's Second Law
figure(6);

load('Newton/Joint_A')
load('Newton/Joint_B')
load('Newton/Joint_C')
load('Newton/Joint_D')
load('Newton/Joint_E')
load('Newton/Joint_F')
load('Newton/Joint_G')
load('Newton/Torque')

plotForceData(6, [4, 2], 1, [timestep, F_a_vec], 'r', 'Internal Force at Joint A', 'x', 'Force');
plotForceData(6, [4, 2], 2, [timestep, F_b_vec], 'g', 'Internal Force at Joint B', 'x', 'Force');
plotForceData(6, [4, 2], 3, [timestep, F_c_vec], 'b', 'Internal Force at Joint C', 'x', 'Force');
plotForceData(6, [4, 2], 4, [timestep, F_d_vec], 'k', 'Internal Force at Joint D', 'x', 'Force');
plotForceData(6, [4, 2], 5, [timestep, F_e_vec], 'm', 'Internal Force at Joint E', 'x', 'Force');
plotForceData(6, [4, 2], 6, [timestep, F_f_vec], 'c', 'Internal Force at Joint F', 'x', 'Force');
plotForceData(6, [4, 2], 7, [timestep, F_g_vec], 'b', 'Internal Force at Joint G', 'x', 'Force');
plotForceData(6, [4, 2], 8, [timestep, Torque_vec], 'b', 'Input Torque', 'x', 'Torque');

% Finding max values
[indexA_newton, FA_newton_max] = findMaxMagnitude(F_a_vec, 'F_a_newton');
[indexB_newton, FB_newton_max] = findMaxMagnitude(F_b_vec, 'F_b_newton');
[indexC_newton, FC_newton_max] = findMaxMagnitude(F_c_vec, 'F_c_newton');
[indexD_newton, FD_newton_max] = findMaxMagnitude(F_d_vec, 'F_d_newton');
[indexE_newton, FE_newton_max] = findMaxMagnitude(F_e_vec, 'F_e_newton');
[indexF_newton, FF_newton_max] = findMaxMagnitude(F_f_vec, 'F_f_newton');
[indexG_newton, FG_newton_max] = findMaxMagnitude(F_g_vec, 'F_g_newton');

% Finding min values
[indexA_newton_min, FA_newton_min] = findMinMagnitude(F_a_vec, 'F_a_newton');
[indexB_newton_min, FB_newton_min] = findMinMagnitude(F_b_vec, 'F_b_newton');
[indexC_newton_min, FC_newton_min] = findMinMagnitude(F_c_vec, 'F_c_newton');
[indexD_newton_min, FD_newton_min] = findMinMagnitude(F_d_vec, 'F_d_newton');
[indexE_newton_min, FE_newton_min] = findMinMagnitude(F_e_vec, 'F_e_newton');
[indexF_newton_min, FF_newton_min] = findMinMagnitude(F_f_vec, 'F_f_newton');
[indexG_newton_min, FG_newton_min] = findMinMagnitude(F_g_vec, 'F_g_newton');

% For Torque, assuming it's a single-dimensional vector
[Torque_newton_maxIndex, T_max] = findMaxMagnitude(Torque_vec, 'T_newton');  % If Torque_vec is multi-dimensional, adjust accordingly
[Torque_newton_minIndex, T_min] = findMinMagnitude(Torque_vec, 'T_newton');

function plotKinematicData(data, figNum, subplotDims, subplotNum, plotTitle, xLabel, yLabel, color)
    figure(figNum);
    set(gcf, 'Color', 'white');  % Sets the color of the current figure to white
    subplot(subplotDims(1), subplotDims(2), subplotNum);
    plot(data(:,1), sqrt(sum(data(:,2:end).^2, 2)), color);
    title(plotTitle);
    xlabel(xLabel);
    ylabel(yLabel);
end

function plotForceData(figNum, subplotDims, subplotNum, data, color, plotTitle, xLabel, yLabel)
    figure(figNum);
    set(gcf, 'Color', 'white');  % Sets the color of the current figure to white
    subplot(subplotDims(1), subplotDims(2), subplotNum);
    % Calculate the magnitude of the vector for each row
    vectorMagnitude = sqrt(data(:,2).^2 + data(:,3).^2 + data(:,4).^2);

    % Plot the magnitude against the first column (time step)
    plot(data(:,1), vectorMagnitude, color);
    % plot(data(:,1), data(:,2), color);
    title(plotTitle);
    xlabel(xLabel);
    ylabel(yLabel);
end

function [index, maxValue] = findMaxMagnitude(vector, msg)
    magnitudes = sqrt(sum(vector.^2, 2));
    [maxValue, index] = max(magnitudes);
    fprintf('Max %s: %f\n', msg, maxValue);
end

function [index, minValue] = findMinMagnitude(vector, msg)
    magnitudes = sqrt(sum(vector.^2, 2));
    [minValue, index] = min(magnitudes);
    fprintf('Min %s: %f\n', msg, minValue);
end



