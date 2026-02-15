function [KRob, Rated] = Assign_Robot_Constants_and_Rated_Values()

%% Primary reference values
Rated.w_hss = 7580 / 60 * 2 * pi; % From Manufacturer's Datasheet (MD)
Rated.tau_hss = 0.177 ; % N m % From Manufacturer's Datasheet (MD)
Rated.i = 6; % From Manufacturer's Datasheet (MD)
KRob.rgt = 26; % gear box ratio  % From Manufacturer's Datasheet (MD)

Rated.w = Rated.w_hss / KRob.rgt;
Rated.tau = Rated.tau_hss * KRob.rgt;

%% Geometrical parameters
KRob.rdrum = 0.025 ; % drum diameter / 2

%% Parameters
%% Definition of robot 
% Z will gather all parameters and coefficients of the system

KRob.m = 4; % Number of actuators
KRob.n = 3; % TTR: 2 translational DOF + 1 rotational DOF

KRob.W = 3; % cable length from drum to last pulley
KRob.H = 1.6;

% End effector
KRob.mee = 0.5;
KRob.Iee = 0.005;
KRob.Mee = [KRob.mee; KRob.mee; KRob.Iee]; 

KRob.g = 9.81;
KRob.w = 0.30;
KRob.h = 0.30;

% motor 1 is at Low Left. CCW
% Position 0,0 is at the frame center. EE orientation is CCW

% Motors located at frame vertices
KRob.PAP= [-KRob.W/2 -KRob.H/2;
            KRob.W/2 -KRob.H/2;
            KRob.W/2  KRob.H/2;
            -KRob.W/2 KRob.H/2;];

% Vertices of the end-effector in local coordinates
KRob.DAP_local = [ -KRob.w/2 -KRob.h/2;
            KRob.w/2 -KRob.h/2;
            KRob.w/2  KRob.h/2;
           -KRob.w/2  KRob.h/2];

% Initial cable lengths

KRob.L_prev = zeros(KRob.m, 1)*KRob.W;  % Another value may be used considering longer cable segments

[KRob.L0_vector, KRob.L0_mod] = ik_cable_robot(zeros(1,KRob.n), KRob.PAP, KRob.DAP_local, 0);

%% Mechanical and electrical parameters
KRob.kphi = 0.0302 ; % From Manufacturer's Datasheet (MD)

KRob.J_hss = 1.42E-5  ; % From Manufacturer's Datasheet (MD)
KRob.J_gt = 9.1E-7 * KRob.rgt^2; % 6.15e-04 % From Manufacturer's Datasheet (MD) model 203119 gearhead 
KRob.J_drum = 5.04E-5; % Estimated for a steel screw of 1 cm of diameter

KRob.Jeq = KRob.J_gt + KRob.J_drum + KRob.J_hss * KRob.rgt^2;


%% Parameters LuGre model
% Stribeck curve parameters (output shaft torque)
KRob.Ts = 0.62 * 9.81 * KRob.rdrum;     % Static torque [N·m] From experiment
KRob.Tc = KRob.Ts*0.8;     % Coulomb torque [N·m] Estimated
KRob.omega_s = 0.15; % Characteristic speed [rad/s] Small influence. Estimated

% Internal LuGre parameters
KRob.sigma0 = 0.0875;   % stiffness of micro-asperities [N·m/(rad)] Estimated
KRob.sigma1 = 0.0016;    % inner damping [N·m·s/rad] Estimated
KRob.B_gt_drum = 0.0378;   % viscous friction [N·m·s/rad] From experiment
KRob.B_hss = 5.21E-6; % From Manufacturer's Datasheet (MD)
KRob.B_gt = 0.02864; % obtained from MD (max efficiency = 0.81)  
KRob.B_drum = KRob.B_gt_drum - KRob.B_gt;
%% rend_gearhead = 0.81; maximum efficiency of gearhead
    %%1-rend_gearhead = B * Refw /Ref.tau;
    %%B = (1-rend_gearhead)*Ref.tau / Ref.w

KRob.Beq = KRob.B_gt_drum + KRob.B_hss * KRob.rgt^2;

%% Elasticity
Elastic_modulus = 48.9E9; % From experiments for steel ropes
section = 1.89E-6;
KRob.K_elastic_per_length = Elastic_modulus * section;


