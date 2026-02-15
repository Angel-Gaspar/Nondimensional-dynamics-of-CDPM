function [t, Xeeref, alpha_ref, initial_angle, initial_set_of_tensions] = ...
    Angle_References_TTR(Test_options, Test_parameters, Z)

draw_the_CPDR = ~Test_options.normalize && Test_options.show_EE_positions;

include_I_in_PID = Test_options.include_I_in_PID;
use_sin2 = Test_options.use_sin2;
include_op_pt = Test_options.include_op_pt;
include_traslation = Test_options.include_traslation;
model = Test_options.model ;

initial_Pose = Test_parameters.initial_Pose;
Distance = Test_parameters.Distance;
pt_min = Test_parameters.pt_min;

Tension_2_alpha = @(T, KPID) (T * Z.rdrum) /Z.rgt / Z.kphi / KPID ;

if include_I_in_PID
    trust_on_integrative_term_for_pretension = 0; % or 1
else
    trust_on_integrative_term_for_pretension = 0; % always 0
end
include_Kp_in_ref = Test_options.include_Kp_in_ref * (1-trust_on_integrative_term_for_pretension);

%% Initial Pose

%initial_Pose = [-Z.W/2*2/3;0;0] *0 ;

[Lini_vector, Lini_mod] = ik_cable_robot(initial_Pose, Z.PAP, Z.DAP_local, 0);
lengths = Lini_mod + Z.L_prev;
%Kel = Z.K_elastic_per_length ./ lengths;

%% Including the pre-tension

A = structural_matrix_from_lengths_2D(Lini_vector, Z.DAP_local);

wrench = zeros(Z.n,1) + [0 -Z.mee*Z.g 0]';

if include_op_pt
   initial_tension = 0;
   final_tension_operation_pretension = pt_min;
else
   initial_tension = pt_min; %Directly it starts from the initial tension
end
initial_set_of_tensions = compute_tensions(A, wrench, 0, Test_parameters.max_tension);


%% Pretension. Reference
dt = Test_parameters.dt;

nel = 0;
t = [];
Xeeref = [0; 0; 0];
Kp = Test_parameters.Kp;
if include_op_pt
    % Starting the pretension operation

    [alpha_ref_t0_T0_wo_Kp, T_solo_gravity] = get_trajectory_Q(initial_Pose, Z, wrench, 0);  
    
    alpha_ref_t0_T0 = alpha_ref_t0_T0_wo_Kp + include_Kp_in_ref *  Tension_2_alpha (T_solo_gravity,  Kp);
    % Finishing the pretension operation
    [alpha_ref_final_wo_Kp_pt, Tfinal_pt] = get_trajectory_Q(initial_Pose, Z, wrench, final_tension_operation_pretension);

    alpha_ref_final_pt = alpha_ref_final_wo_Kp_pt + include_Kp_in_ref *  Tension_2_alpha (Tfinal_pt, Kp) ;

    time_pt = Test_parameters.pt_time;

    t_pt = 0:dt:time_pt;
    if use_sin2 
        alpha_ref_pretension = alpha_ref_t0_T0 + (alpha_ref_final_pt - alpha_ref_t0_T0) * (sin(t_pt*pi/2 / time_pt)).^2;
    else
        alpha_ref_pretension = alpha_ref_t0_T0 + (alpha_ref_final_pt - alpha_ref_t0_T0) * t_pt / time_pt;
    end
    nel = nel + numel(t_pt);
    t = [t;t_pt];
    Xeeref = [initial_Pose + t*0];
    if draw_the_CPDR
        figure(1)
        subplot(1,2,1)
        drawCDPR(Z.PAP, Z.DAP_local, initial_Pose, Tfinal_pt, wrench);
        title('Initial position')
    end
else 
    Tfinal_pt = initial_set_of_tensions;
end

%% Movements. Reference
if include_traslation
    [t_mov, Xeeref_mov, ~, ~] = Create_Reference_Synchr(dt, Distance, Test_parameters.V_cruise, Test_parameters.T_accel, use_sin2);

    if draw_the_CPDR
        subplot(1,2,2)
        [Lengths, lengths] = ik_cable_robot(Xeeref_mov(:,end), Z.PAP, Z.DAP_local, 0);
        A = structural_matrix_from_lengths_2D(Lengths, Z.DAP_local);
        [tension_at_target, ~] = compute_tensions(A, wrench, Test_parameters.pt_min);
        drawCDPR(Z.PAP, Z.DAP_local, Xeeref_mov(:,end), tension_at_target, wrench);
        title('Target position')
        %h=gcf;
        %set(h,'PaperOrientation','landscape');
        %set(h,'PaperPosition', [1 1 28 10]);
        %print(gcf, '-dpdf', 'EE-positions.pdf');
        fig = gcf;
        fig.Units = 'centimeters';
        fig.Position(3) = 32;   % anchura en cm
        fig.Position(4) = 8;    % altura en cm
        exportgraphics(gcf, "Fig_EE-positions.pdf");%, ...
       % "PreserveAspectRatio", "on")
    end

    nstill = floor(Test_parameters.time_still / Test_parameters.dt);
    % Add still and return

    Xeeref_mov =[zeros(3,nstill), Xeeref_mov, Distance + zeros(3,nstill), Distance - Xeeref_mov, zeros(3,nstill)];
    %Veeref = [zeros(3,nstill),Veeref, zeros(3,nstill), -Veeref, zeros(3,nstill)];
    %Aeeref =[zeros(3,nstill),Aeeref, zeros(3,nstill), -Aeeref, zeros(3,nstill)];
    t_mov = 0:dt:numel(Xeeref_mov)/3*dt-dt;
    
    % Add still instants
    if include_op_pt
        t = [t t_mov+t_pt(end)];
    else 
        t = t_mov;
    end
    Xeeref = [Xeeref, Xeeref_mov + initial_Pose];
    nel = nel + numel(t_mov);
end
%     figure
% plot(t_mov, Xeeref(1,:), t_mov, Xeeref(2,:))
% hold on
% plot(t_mov, Xeeref(3,:))

%% Initial values
%Tmi(:,1) = T_initial*Z.rdrum;


%% Get trajectory of Xee

if include_op_pt
    alpha_ref = alpha_ref_pretension;
else 
    alpha_ref = zeros(Z.m, 0);
end
if include_traslation
    [alpha_ref_mov_wo_Kp, Tension] = get_trajectory_Q(Xeeref_mov, Z, wrench, pt_min);
    alpha_ref_mov = alpha_ref_mov_wo_Kp + include_Kp_in_ref *  Tension_2_alpha (Tension, Kp) ;
    alpha_ref = [alpha_ref, alpha_ref_mov];
end
initial_angle = alpha_ref(:,1) - include_Kp_in_ref * Tension_2_alpha (initial_set_of_tensions, Kp);
% or it can also be obtained from  Kel = Z.K_elastic_per_length ./ lengths;
% alpha(:,1)  = Tmi(:,1) ./ Kel +  (lengths - Z.L0_mod)/Z.rdrum*Z.rgt;

%plot(t, alpha_ref);


end