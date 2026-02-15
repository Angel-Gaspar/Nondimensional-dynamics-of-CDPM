function [Z, Test_parameters] = Normalize(Z, Ref, Test_parameters)


    Z.rdrum = Z.rdrum / Ref.length;
    Z.W =       Z.W / Ref.length;
    Z.H =       Z.H / Ref.length;
    Z.mee =     Z.mee / Ref.m;
    Z.Iee =     Z.Iee / Ref.J;
    Z.w =       Z.w / Ref.length;
    Z.h =       Z.h / Ref.length;
    Z.g =       Z.g / Ref.acel;

Z.Mee = [Z.mee; Z.mee; Z.Iee];    

% motor 1 is at Low Left. CCW
% Position 0,0 is at the frame center. EE orientation is CCW

% Motors located at frame vertices
Z.PAP= Z.PAP / Ref.length;

% Vertices of the end-effector in local coordinates
Z.DAP_local = Z.DAP_local / Ref.length;

% Initial cable lengths

Z.L_prev = Z.L_prev/Ref.length;

Z.L0_vector = Z.L0_vector / Ref.length;
Z.L0_mod = Z.L0_mod / Ref.length;


Z.kphi = Z.kphi / Ref.kphi;
%Z.J_hss = Z.J_hss / Ref.J_hss;
Z.Jeq = Z.Jeq / Ref.J;



%% Parameters LuGre model

Z.Ts  = Z.Ts / Ref.tau;
Z.Tc  = Z.Tc / Ref.tau;
Z.omega_s = Z.omega_s / Ref.w ;

Z.sigma0 = Z.sigma0 / Ref.tau; % / Ref.B;
Z.sigma1 = Z.sigma1 / Ref.tau; % / Ref.B;
Z.Beq = Z.Beq / Ref.B;
%Z.B_hss = Z.B_hss / Ref.B_hss; 


%% Elasticity

Z.K_elastic_per_length  = Z.K_elastic_per_length / Ref.kel_esp;


%% PID parameters and variables

Test_parameters.derivative_time = Test_parameters.derivative_time / Ref.t;
Test_parameters.integrative_time = Test_parameters.integrative_time / Ref.t;

Test_parameters.Kp = Test_parameters.Kp / Ref.PID_gain;
Test_parameters.u_limit = Test_parameters.u_limit / Ref.i;

Test_parameters.Ki = Test_parameters.Kp / Test_parameters.integrative_time;
Test_parameters.Kd = Test_parameters.Kp * Test_parameters.derivative_time;

%% Pretension

Test_parameters.pt_min = Test_parameters.pt_min / Ref.tension;
Test_parameters.max_tension = Test_parameters.max_tension / Ref.tension;
Test_parameters.pt_time = Test_parameters.pt_time / Ref.t;

%% Speed profile parameters

Test_parameters.V_cruise =Test_parameters.V_cruise ./ [Ref.vel; Ref.vel; Ref.w];
Test_parameters.T_accel = Test_parameters.T_accel / Ref.t;
Test_parameters.time_still = Test_parameters.time_still / Ref.t;

%% Movements

Test_parameters.initial_Pose = Test_parameters.initial_Pose ...
    ./ [Ref.length; Ref.length; Ref.alpha];
Test_parameters.Distance = Test_parameters.Distance ...
    ./ [Ref.length; Ref.length; Ref.alpha];

Test_parameters.dt = Test_parameters.dt / Ref.t;
Z.rgt = 1; % There are no gear trains in the normalized frame
