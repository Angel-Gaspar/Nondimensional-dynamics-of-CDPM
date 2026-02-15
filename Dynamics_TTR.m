function result = Dynamics_TTR(t, alpha_hss_ref, initial_angle, initial_tension, Test_options, Test_parameters, Z)
dt = Test_parameters.dt;

model = Test_options.model ;
initial_Pose = Test_parameters.initial_Pose;
Tension_2_alpha_hss = @(T, KPID) (T * Z.rdrum) /Z.rgt / Z.kphi / KPID ;
%% Including the pre-tension

wrench = zeros(Z.n,1) + [0 -Z.mee*Z.g 0]';

%T_initial_pose_t0 = compute_tensions(A, wrench, initial_tension, max_tension);
integ_error = Test_options.include_I_in_PID * Tension_2_alpha_hss (initial_tension , Test_parameters.Ki);
%drawCDPR(PAP, DAP_local, initial_Pose, tfinal_min, wrench);


%% Allocating variable

nel = numel(t);

zf = zeros(Z.m,nel); % Only to track its evolution

alpha = zeros(Z.m,nel);
alpha_der = zeros(Z.m,nel);
alpha_dder = zeros(Z.m,nel);

Tmi = zeros(Z.m,nel);
Tmi_minus_friction = zeros(Z.m,nel);

Tension = zeros(Z.m,nel);
Tfriction  =  zeros(Z.m,nel);

Aee = zeros(Z.n,nel);
Vee = zeros(Z.n,nel);
Xee = zeros(Z.n,nel); 

%% Initial values
Xee(:,1) = initial_Pose;
Tension(:,1) = initial_tension;
Tmi(:,1) = initial_tension * Z.rdrum;
alpha(:, 1) = initial_angle / Z.rgt;

%% Dynamics Loop

for i = 2: nel
  
    prima_volta = (i == 2);

    %% Positioning error
    error = alpha_hss_ref(:, i-1) - alpha(:, i-1)*Z.rgt;
    
    %%  Control Signal calculation: PID
    [u, integ_error] = PID(error, Test_parameters, dt, integ_error, prima_volta);
    %u = PD(error, Z, dt, prima_volta);
    Tmi(:,i) = Z.kphi * u *Z.rgt;
    
    %% Dynamics at the actuator 
    % Static friction
    if model == 1
        [Tfriction(:,i), zf(:,i)] = friction_LuGre(alpha_der(:,i-1), dt, Z, prima_volta);
        Tmi_minus_friction(:,i) = Tmi(:,i) - Tfriction(:,i);
        Torque_accelerating = Tmi_minus_friction(:,i) -  Tension(:,i-1) * Z.rdrum;
    elseif model == 2            
        [Tfriction(:,i), zf(:,i)] = friction_Dahl(alpha_der(:,i-1), dt, Z, prima_volta);
%Tfriction(:,i) = 0; % ToDo Borrar        
        Tmi_minus_friction(:,i) = Tmi(:,i) - Tfriction(:,i);
        Torque_accelerating = Tmi_minus_friction(:,i) -  Tension(:,i-1) * Z.rdrum;
    elseif model == 0
        Torque_result = Tmi(:,i) -  Tension(:,i-1) * Z.rdrum;
        [Tfriction(:,i)] = friction_Coulomb(alpha_der(:,i-1), Torque_result, Z, Test_parameters.Coulomb_uses_speed);
        Torque_accelerating = Torque_result - Tfriction(:,i);
    end

    alpha_dder(:, i) = Torque_accelerating / Z.Jeq ;
    alpha_der(:, i) = alpha_der(:, i-1)  + alpha_dder(:, i) * dt;
    alpha(:, i) = alpha(:, i-1)  + alpha_der(:, i) * dt;
    
    %% Cable elasticity
    [Lengths, lengths] = ik_cable_robot(Xee(:, i-1), Z.PAP, Z.DAP_local, 0);
    Kel = Z.K_elastic_per_length ./ lengths;

    Tension(:,i) = Kel .* (alpha(:,i)  * Z.rdrum +  (lengths - Z.L0_mod));
    Tension(:,i) = max(0, Tension(:,i));

    %% Dynamics at the EE
    A = structural_matrix_from_lengths_2D(Lengths, Z.DAP_local);
    Resultant_forces = -A * Tension(:,i)  + wrench;
    Aee(:,i) = Resultant_forces ./ Z.Mee;       
    Vee( :,i) = Vee(:, i-1)  + Aee(:, i) * dt;

    Xee(:, i) = Xee(:, i-1)  + Vee( :,i) * dt;

end

result.t = t;
result.Torque = Tmi;
result.Tfriction = Tfriction;
result.Tension = Tension;
result.zf = zf;
result.alpha_hss_ref = alpha_hss_ref;
result.alpha = alpha;
result.Xee = Xee;
result.Vee = Vee;
result.Aee = Aee;
result.alpha_der = alpha_der;
result.alpha_dder = alpha_dder;
alpha_hss_ref_der= zeros(4, numel(t)-1);
alpha_hss_ref_dder= zeros(4, numel(t)-2);

for i = 1:numel(t)-1
    alpha_hss_ref_der(:,i) = alpha_hss_ref(:,i+1)-alpha_hss_ref(:,i);
end
for i = 1:numel(t)-2
    alpha_hss_ref_dder(:,i) = alpha_hss_ref_der(:,i+1)-alpha_hss_ref_der(:,i);
end
%figure(9)
%subplot(1,2,Test_parameters.normalize+1)
%plot(alpha_hss_ref_der')

end