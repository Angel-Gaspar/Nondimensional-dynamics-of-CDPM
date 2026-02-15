function Test_parameters = Assign_Test_Parameters(Rated, KRob, Test_options)
 % Test operation values
 %% PID parameters and variables
Test_parameters.derivative_time = 0.2  ; % Manual tuning    ToDo   quitar
Test_parameters.integrative_time = 0.4; % Manual tuning
if ~Test_options.include_I_in_PID
    Test_parameters.integrative_time = Inf;
end
Test_parameters.Kp = 4.80 ; % Manual tuning
Test_parameters.u_limit = 10;

Test_parameters.Ki = Test_parameters.Kp / Test_parameters.integrative_time;
Test_parameters.Kd = Test_parameters.Kp * Test_parameters.derivative_time;

%% Speed profile parameters

%Z.V_cruise = [Ref.vel; Ref.vel; Ref.w/100];
rated_v = Rated.w * KRob.rdrum;
Test_parameters.V_cruise = [rated_v; rated_v; 0.6]*0.4;
Test_parameters.V_cruise =  [0.6; 0.6; 0.6]*1;

Test_parameters.T_accel = [0.52; 0.52; 0.52];

%% Pretension
Test_parameters.pt_min = 10;
Test_parameters.max_tension = 40;
Test_parameters.pt_time = 0.5;

%% Movements
Test_parameters.initial_Pose = [-1;-0.5;0]*0;
Test_parameters.Distance = [0.15 * KRob.W; 0.15 * KRob.H; 0];
Test_parameters.Distance = [0.5; 0.3; 0];
Test_parameters.time_still = 0.3;

Test_parameters.dt = 0.001;