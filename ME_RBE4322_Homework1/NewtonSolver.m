clc;
clear all;
close all;

%%
load('Kin/Pos/A')
load('Kin/Pos/B')
load('Kin/Pos/C')
load('Kin/Pos/D')
load('Kin/Pos/E')
load('Kin/Pos/F')
load('Kin/Pos/G')

load('Kin/LinAcc/Link/AB');
load('Kin/LinAcc/Link/BC');
load('Kin/LinAcc/Link/DCE');
load('Kin/LinAcc/Link/EF');
load('Kin/LinAcc/Link/FG');   

load('Kin/AngAcc/AB');
load('Kin/AngAcc/BC');
load('Kin/AngAcc/DCE');
load('Kin/AngAcc/EF');
load('Kin/AngAcc/FG');

massAB = 13.68;
massBC = 33.07;
massDCE = 57.7;
massEF = 27.54;
massFG = 59.94;

massMoIAB = 0.43;
massMoIBC = 5.75;
massMoIDCE = 30.36;
massMoIEF = 3.34;
massMoIFG = 34.03;

F_a_vec = zeros(10,3);
F_b_vec = zeros(10,3);
F_c_vec = zeros(10,3);
F_d_vec = zeros(10,3);
F_e_vec = zeros(10,3);
F_f_vec = zeros(10,3);
F_g_vec = zeros(10,3);
Torque_vec = zeros(10,3);

for theta=1:1:10

    g = [0 -9.81 0]; %defining gravity to find weight of each link m/s^2

    A = A_vec(theta,:);
    B = B_vec(theta,:);
    C = C_vec(theta,:);
    D = D_vec(theta,:);
    E = E_vec(theta,:);
    F = F_vec(theta,:);
    G = G_vec(theta,:);

    % Centers of Mass of each Link
    AB_com = (A+B)/2;
    BC_com = (B+C)/2;
    DCE_com = (C+D+E)/3;
    EF_com = (E+F)/2;
    FG_com = (F+G)/2;

    syms Ax Ay Bx By Cx Cy Dx Dy Ex Ey Fx Fy Gx Gy T
    % Forces at each joint
    fA=[Ax Ay 0];
    fB=[-Bx By 0];
    fC=[-Cx Cy 0];
    fD=[Dx Dy 0];
    fE=[-Ex Ey 0];
    fF=[-Fx Fy 0];
    fG=[-Gx Gy 0];
    Tt=[0 0 T];

    % Weight of each link
    wAB=massAB *g;
    wBC=massBC *g;
    wDCE=massDCE *g;
    wEF=massEF *g;
    wFG=massFG *g;

    % Unknown torque of the system
    tT=[0 0 T];

    % Force of the applied load
    LoadForce=[0 -200 0];
    LoadPos = [-1.71434 4.26299 0]; % +1.843 m from joint F. Calculation by hand

    %% Static Equilibrium Equations
    %Link AB
    eqn1=fA+fB+wAB==massAB * A_ab_com(theta,:);
    eqn2=momentVec(A, AB_com, fA) + momentVec(B,  AB_com,fB)+tT==massMoIAB * AAB(theta,:); %only change the ==0 appropriately for newtons 2nd law
    %Link BCE
    eqn3=-fB+fC+wBC==massBC * A_bc_com(theta,:);
    eqn4=momentVec(B, BC_com, -fB)+momentVec(C, BC_com, fC) ==massMoIBC * ABC(theta,:); %only change the ==0 appropriately for newtons 2nd law
    %Link CD
    eqn5=-fC+fD+fE+wDCE==massDCE * A_dce_com(theta,:);
    eqn6=momentVec(C, DCE_com, -fC)+momentVec(D, DCE_com, fD)==massMoIDCE * ADCE(theta,:); %only change the ==0 appropriately for newtons 2nd law
    %Link EF
    eqn7=-fE+fF+wEF==massEF * A_ef_com(theta,:);
    eqn8=momentVec(E, EF_com, -fE)+momentVec(F, EF_com, fF)==massMoIEF * AEF(theta,:); %only change the ==0 appropriately for newtons 2nd law
    %Link FG
    eqn9=-fF+fG+wFG+LoadForce==massFG * A_fg_com(theta,:);
    eqn10=momentVec(F, FG_com, -fF)+momentVec(G, FG_com, fG)+momentVec(LoadPos, FG_com, LoadForce)==massMoIFG * AFG(theta,:); %only change the ==0 appropriately for newtons 2nd law

    solution = (solve([eqn1,eqn2,eqn3,eqn4,eqn5,eqn6,eqn7,eqn8,eqn9,eqn10],[Ax,Ay,Bx,By,Cx,Cy,Dx,Dy,Ex,Ey,Fx,Fy,Gx,Gy,T]));

    F_a_vec(theta,:) = [double(solution.Ax) double(solution.Ay) 0];
    F_b_vec(theta,:) = [double(solution.Bx) double(solution.By) 0];
    F_c_vec(theta,:) = [double(solution.Cx) double(solution.Cy) 0];
    F_d_vec(theta,:) = [double(solution.Dx) double(solution.Dy) 0];
    F_e_vec(theta,:) = [double(solution.Ex) double(solution.Ey) 0];
    F_f_vec(theta,:) = [double(solution.Fx) double(solution.Fy) 0];
    F_g_vec(theta,:) = [double(solution.Gx) double(solution.Gy) 0];
    Torque_vec(theta,:)=[0 0 double(solution.T)];
end

% Create Directory for Saving Results
folderName = 'Newton';  % Directory name
if ~exist(folderName, 'dir')
   mkdir(folderName);  % Create the directory if it doesn't exist
end


save('Newton/Joint_A', 'F_a_vec')
save('Newton/Joint_B', 'F_b_vec')
save('Newton/Joint_C', 'F_c_vec')
save('Newton/Joint_D', 'F_d_vec')
save('Newton/Joint_E', 'F_e_vec')
save('Newton/Joint_F', 'F_f_vec')
save('Newton/Joint_G', 'F_g_vec')
save('Newton/Torque', 'Torque_vec')

function pos = momentVec(pos, fixPos, force)
    % Position Vector
    r = pos - fixPos;
    pos = cross(r,force);
end
