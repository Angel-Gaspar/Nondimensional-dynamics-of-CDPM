%%
% Script to compare the results as obtained from the normalized and dimensional formulation of dynamics

clear all
close all


%% Defining the operation conditions
Test_options.model = 2;  % 1. Lugre  2.Dahl
Test_options.include_I_in_PID = 0;
Test_options.include_Kp_in_ref = 1;
Test_options.use_sin2 = 1;
Test_options.include_op_pt = 1;
Test_options.include_traslation = 1;
Test_options.Coulomb_uses_speed = 0;
Test_options.normalize = 1;

Test_options.show_EE_positions = 1;

for index = 0:1
    Test_options.normalize = index;

    [KRob, Rated] = Assign_Robot_Constants_and_Rated_Values( );
    Test_parameters = Assign_Test_Parameters(Rated, KRob, Test_options); % speed, PID gains
    
    if Test_options.normalize        
       Ref = Calculate_Ref(KRob, Rated, Test_options);
       [KRob, Test_parameters] = Normalize(KRob, Ref, Test_parameters);    
    end
    
    [t, Xeeref, alpha_hss_ref, initial_real_angle_hss, initial_set_of_tensions] = ...
        Angle_References_TTR(Test_options, Test_parameters, KRob);
    
    %Test_parameters.t_mov = 0:dt:Test_parameters.t_mov(end)*2+(1+nstill*3)*dt;
    
    %% Solve the dynamics
    
    figure(9)
    subplot(1,2,Test_options.normalize+1)
    plot(alpha_hss_ref')

    result = Dynamics_TTR(t, alpha_hss_ref, initial_real_angle_hss, initial_set_of_tensions, Test_options, Test_parameters, KRob);
    result.Xeeref = Xeeref;
    Plot_result(result, index)
end