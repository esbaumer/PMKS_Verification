clc;
close all
clear all;

%%
load('Kin/Pos/A')
load('Kin/Pos/B')
load('Kin/Pos/C')
load('Kin/Pos/D')
load('Kin/Pos/E')
load('Kin/Pos/F')
load('Kin/Pos/G')

WAB=zeros(10,3);
WBC=zeros(10,3);
WDCE=zeros(10,3);
WEF=zeros(10,3);
WFG=zeros(10,3);

V_a=zeros(10,3);
V_b=zeros(10,3);
V_c=zeros(10,3);
V_d=zeros(10,3);
V_e=zeros(10,3);
V_f=zeros(10,3);
V_g=zeros(10,3);

V_ab_com=zeros(10,3);
V_bc_com=zeros(10,3);
V_dce_com=zeros(10,3);
V_ef_com=zeros(10,3);
V_fg_com=zeros(10,3);

AAB=zeros(10,3);
ABC=zeros(10,3);
ADCE=zeros(10,3);
AEF=zeros(10,3);
AFG=zeros(10,3);

A_a=zeros(10,3);
A_b=zeros(10,3);
A_c=zeros(10,3);
A_d=zeros(10,3);
A_e=zeros(10,3);
A_f=zeros(10,3);
A_g=zeros(10,3);

A_ab_com=zeros(10,3);
A_bc_com=zeros(10,3);
A_dce_com=zeros(10,3);
A_ef_com=zeros(10,3);
A_fg_com=zeros(10,3);

for theta=1:1:10
    % Coordinates of joints
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

    %% Velocity loops
    %velocity equations from given loops
    syms wBC wDCE wEF wFG
    omegaAB=[0 0 1.8589];
    omegaBC=[0 0 wBC];
    omegaDCE=[0 0 wDCE];
    omegaEF=[0 0 wEF];
    omegaFG=[0 0 wFG];

    % A->B->C->D->A
    % V_ba + V_cb + V_dc + V_ad = 0
    eqn1=velSolver(omegaAB,B-A)+velSolver(omegaBC,C-B)+velSolver(omegaDCE,D-C)==0;

    % D->E->F->G->D
    % V_ed + V_fe + V_gf + V_dg = 0
    eqn2=velSolver(omegaDCE,E-D)+velSolver(omegaEF,F-E)+velSolver(omegaFG,G-F)==0;

    solution=solve([eqn1, eqn2],[wBC wDCE wEF wFG]);

    % Store all the determined angular velocities
    WAB(theta,:)=[0 0 1.8589];
    WBC(theta,:)=[0 0 solution.wBC]; %angular velocity of BC
    WDCE(theta,:)=[0 0 solution.wDCE]; %angular velocity of DCE
    WEF(theta,:)=[0 0 solution.wEF]; %angular velocity of EF
    WFG(theta,:)=[0 0 solution.wFG]; %angular velocity of FG


    % Determine the velocities at each joint
    V_a(theta,:) = [0 0 0];
    V_b(theta,:) = velSolver(WAB(theta,:),B-A);
    V_c(theta,:) = velSolver(WDCE(theta,:),C-D);
    V_d(theta,:) = [0 0 0];
    V_e(theta,:) = velSolver(WDCE(theta,:),E-D);
    V_f(theta,:) = velSolver(WFG(theta,:),F-G);
    V_g(theta,:) = [0 0 0];

    % Determine the velocities at each link's center of mass
    V_ab_com(theta,:) = velSolver(WAB(theta,:),AB_com - A);
    V_bc_com(theta,:) =  velSolver(WBC(theta,:),BC_com - B) + V_b(theta,:);
    V_dce_com(theta,:) = velSolver(WDCE(theta,:),DCE_com - D);
    V_ef_com(theta,:) =  velSolver(WEF(theta,:),EF_com - E) + V_e(theta,:);
    V_fg_com(theta,:) = velSolver(WFG(theta,:),FG_com - G);

    %% Acceleration loops
    %acceleration equations from given loops
    syms aBC aDCE aEF aFG
    alphaAB=[0 0 0];
    alphaBC=[0 0 aBC];
    alphaDCE=[0 0 aDCE];
    alphaEF=[0 0 aEF];
    alphaFG=[0 0 aFG];

    % A->B->C->D->A
    % A_ba + A_cb + A_dc + A_ad = 0
    eqn1=accSolver(WAB(theta,:),alphaAB, B-A)+accSolver(WBC(theta,:),alphaBC,C-B)+accSolver(WDCE(theta,:),alphaDCE,D-C)==0;

    % D->E->F->G->D
    % A_ed + A_fe + A_gf + A_dg = 0
    eqn2=accSolver(WDCE(theta,:),alphaDCE,E-D)+accSolver(WEF(theta,:),alphaEF,F-E)+accSolver(WFG(theta,:),alphaFG,G-F)==0;
    
    solution=solve([eqn1, eqn2],[aBC aDCE aEF aFG]);

    % Store all the determined angular accelerations
    AAB(theta,:)=[0 0 0];
    ABC(theta,:)=[0 0 solution.aBC]; %angular acceleration of BC
    ADCE(theta,:)=[0 0 solution.aDCE]; %angular acceleration of DCE
    AEF(theta,:)=[0 0 solution.aEF]; %angular acceleration of EF
    AFG(theta,:)=[0 0 solution.aFG]; %angular acceleration of FG


    % Determine the accelerations at each joint
    A_a(theta,:) = [0 0 0];
    A_b(theta,:) = accSolver(WAB(theta,:), AAB(theta,:),B-A);
    A_c(theta,:) = accSolver(WBC(theta,:), ABC(theta,:),C-D);
    A_d(theta,:) = [0 0 0];
    A_e(theta,:) = accSolver(WDCE(theta,:), ADCE(theta,:),E-D);
    A_f(theta,:) = accSolver(WFG(theta,:),AFG(theta,:),F-G);
    A_g(theta,:) = [0 0 0];

    % Determine the accelerations at each link's center of mass
    A_ab_com(theta,:) = accSolver(WAB(theta,:),AAB(theta,:),AB_com - A);
    A_bc_com(theta,:) =  accSolver(WBC(theta,:),ABC(theta,:),BC_com - B) + A_b(theta,:);
    A_dce_com(theta,:) = accSolver(WDCE(theta,:),ADCE(theta,:),DCE_com - D);
    A_ef_com(theta,:) =  accSolver(WEF(theta,:),AEF(theta,:),EF_com - E) + A_e(theta,:);
    A_fg_com(theta,:) = accSolver(WFG(theta,:),AFG(theta,:),FG_com - G);
end

% Directory for saving the results
checkDirectory('Kin/LinVel/Joint');
checkDirectory('Kin/LinVel/Link');
checkDirectory('Kin/AngVel');
checkDirectory('Kin/LinAcc/Joint');
checkDirectory('Kin/LinAcc/Link');
checkDirectory('Kin/AngAcc');

save('Kin/LinVel/Joint/A', 'V_a')
save('Kin/LinVel/Joint/B', 'V_b')
save('Kin/LinVel/Joint/C', 'V_c')
save('Kin/LinVel/Joint/D', 'V_d')
save('Kin/LinVel/Joint/E', 'V_e')
save('Kin/LinVel/Joint/F', 'V_f')
save('Kin/LinVel/Joint/G', 'V_g')

save('Kin/LinVel/Link/AB', 'V_ab_com')
save('Kin/LinVel/Link/BC', 'V_bc_com')
save('Kin/LinVel/Link/DCE', 'V_dce_com')
save('Kin/LinVel/Link/EF', 'V_ef_com')
save('Kin/LinVel/Link/FG', 'V_fg_com')

save('Kin/AngVel/AB', 'WAB')
save('Kin/AngVel/BC', 'WBC')
save('Kin/AngVel/DCE', 'WDCE')
save('Kin/AngVel/EF', 'WEF')
save('Kin/AngVel/FG', 'WFG')

save('Kin/LinAcc/Joint/A', 'A_a')
save('Kin/LinAcc/Joint/B', 'A_b')
save('Kin/LinAcc/Joint/C', 'A_c')
save('Kin/LinAcc/Joint/D', 'A_d')
save('Kin/LinAcc/Joint/E', 'A_e')
save('Kin/LinAcc/Joint/F', 'A_f')
save('Kin/LinAcc/Joint/G', 'A_g')

save('Kin/LinAcc/Link/AB', 'A_ab_com')
save('Kin/LinAcc/Link/BC', 'A_bc_com')
save('Kin/LinAcc/Link/DCE', 'A_dce_com')
save('Kin/LinAcc/Link/EF', 'A_ef_com')
save('Kin/LinAcc/Link/FG', 'A_fg_com')

save('Kin/AngAcc/AB', 'AAB')
save('Kin/AngAcc/BC', 'ABC')
save('Kin/AngAcc/DCE', 'ADCE')
save('Kin/AngAcc/EF', 'AEF')
save('Kin/AngAcc/FG', 'AFG')

function vel = velSolver(w, r)
vel = cross(w,r);
end

function acc = accSolver(w,a,r)
acc = cross(w,cross(w,r)) + cross(a,r);
end

function checkDirectory(saveDir)
% Check if the directory exists, if not, create it
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end
end