
clear variables
close all

%% Motor profile parameters
distance = 2;
dt = 0.001;
Z.v_cruise = 2; % cruise speed, 2 meters per second
Z.t_accel = 0.5;
[t, qref, vref, aref] = Create_Reference(dt,distance, Z.v_cruise, Z.t_accel,0.2, 2);
nel =  numel(t) ;

%% Geometrical parameters
%rdrum = 0.025; % drum diameter /2
Z.rdrum_pu = 1 ;
Z.rgb = 70; % gear box ratio
Z.rgb_pu = 1;

Z.mEE_pu = 5;
Z.LA0 = 5; % cable length from drum to last pulley
Z.LB0 = 5;
Z.L0 = [Z.LA0; Z.LB0];


%% Mechanical and electrical parameters
Z.kphi_pu = 1;
Z.Jeq = 0.4;


%% Parameters LuGre model
% Parámetros curva de Stribeck (torque en el eje de salida)
Z.Ts = 10;     % Torque estático [N·m]
Z.Tc = Z.Ts*0.8;     % Torque de Coulomb [N·m]
Z.omega_s = 0.5; % Velocidad característica [rad/s]

% Parámetros internos LuGre
Z.sigma0 = 140.0;   % rigidez de micro-asperezas [N·m/(rad)]
Z.sigma1 = 2.5;    % amortiguamiento interno [N·m·s/rad]
Z.Beq = 2.5;   % fricción viscosa [N·m·s/rad]
zf = zeros(2,nel); % Only to track its evolution

%% Elasticity
Z.sgnEQ = [-1; 1]; % Q1 on the left, pointing to -x   Q2 on the right, pointing to +x
Z.K_elastic_per_length = 100000;
% K_elastic = K_elastic_per_length ./ (L0 + sign_q_EE * pos_EE)
K_elastic = @(pos_EE) Z.K_elastic_per_length ./(Z.L0 + Z.sgnEQ * pos_EE);

%% PID parameters and variables
Z.Kp =1000;
Z.Ki = Z.Kp * 0.3;
Z.Kd = Z.Kp * 0.2;
Z.u_limit = 100;

for model = 1:2

%% Allocating variables
q = zeros(2,nel+2);
qd = zeros(2,nel+1);
qdd = zeros(2,nel);

Tmi = q;
Tmi_minus_friction = q;

Tension = zeros(2,nel);
Tfriccion  = Tension;

aee = zeros(1,nel);
vee = zeros(1,nel+1);
xee = zeros(1,nel+2);

%% Auxiliar variables
prev_error = [0; 0];
q_error =  [0; 0];

u = zeros(2, 1);

%% Including the pre-tension
pretension = 40;
integ_error = [0; 0] + pretension / Z.Ki /Z.kphi_pu /Z.rdrum_pu/Z.rgb_pu;

Tension(:,1) = [pretension; pretension];
Kel = K_elastic(0);
q(:,1) = pretension ./ Kel;
qref = qref + q(:,1);

%% Dynamics Loop

for i = 1: numel(t)

    %%  Control Signal calculation: PID
     q_error = qref(:, i) - q(:, i);
    integ_error = integ_error + q_error * dt;
    derivative_error = (q_error - prev_error) / dt;
    prev_error = q_error;
    u  = Z.Kp * q_error + Z.Ki * integ_error + Z.Kd * derivative_error ;
    u  = min(Z.u_limit, max(u, -Z.u_limit));

    Tmi(:,i) = Z.kphi_pu * u ;
    %% Static friction
    if model == 1
        [Tfriccion(:,i), zf(:,i)] = friction_LuGre(qd(:,i), dt, Z, i== 1);
    else             
        [Tfriccion(:,i), zf(:,i)] = friction_Dahl(qd(:,i), dt, Z, i== 1);
    end

    Tmi_minus_friction(:,i) = Tmi(:,i) - Tfriccion(:,i);

    Torque_accelerating = Tmi_minus_friction(:,i) -  Tension(:,i)*Z.rdrum_pu;
    
    qdd(:, i+1) = Torque_accelerating / Z.Jeq;
    qd(:, i+1) = qd(:, i)  + qdd(:, i+1) * dt;
    q(:, i+1) = q(:, i)  + qd(:, i+1) * dt;
    
    Kel = Z.K_elastic_per_length ./ (Z.L0 - Z.sgnEQ * xee(i)) ;

    %Tension(1, i+1) = Kel(1) *(  q(1,i+1) + xee(i));
    %Tension(2, i+1) = Kel(2) * (  q(2,i+1) - xee(i));
    Tension(:,i+1) = Kel .* (q(:,i+1)  -  Z.sgnEQ * xee(i));
    Tension(:,i+1) = max(0, Tension(:,i+1));

    aee(i+1) = (Tension(2,i) - Tension(1,i)) / Z.mEE_pu;
    vee( i+1) = vee( i)  + aee( i+1) * dt;

    xee( i+1) = xee( i)  + vee( i+1) * dt;

    %% Only to observe THIS loop variables
       this_Torque_accelerating = Torque_accelerating;
    this_Tmi_minus_friction = Tmi_minus_friction(:,i);
    this_Tension = Tension(:, i+1);
    this_xee =  xee( i+1) ; 
    this_vee = vee( i+1); 
    this_aee = aee(i+1);
    this_q = q(:, i+1) ;     
    this_qd = qd(:, i+1);    
    this_qdd = qdd(:, i+1);

end
%Tension = Tension - increment_elastic_force.* sign_q_EE;

set(0,'defaultAxesFontSize',14)
%t = t(t < 3.951);
nel = numel(t);
figure(11)

subplot(2,1,1)
plot (t, qref(:,1:nel))
hold on
plot(t, q(:,1:nel))
hold off
subplot(2,1,2)
plot(t, qd(1,1:numel(t)), 'LineWidth', 1.5)
hold  on
plot(t, qd(2,1:numel(t)), 'LineWidth', 1.5)
hold  off



figure(12)
subplot(3,1,1)
plot(t, xee(1:numel(t)), 'LineWidth', 1.5)
subplot(3,1,2)
plot(t, vee(1:numel(t)), 'LineWidth', 1.5)
subplot(3,1,3)
plot(t, aee(1:numel(t)), 'LineWidth', 1.5)
hold on
grid
hold off

figure(13)
%plot(t, aee(1:numel(t)), 'LineWidth', 1.5)
plot(t, Tmi(:,1:nel))
grid
hold on
plot(t, Tmi_minus_friction(1,1:nel))
hold off
%  plot(t, stiction, 'LineWidth', 1.5)
%  plot(t, Torque_fric_din, 'LineWidth', 1.5)
%  plot(t, Torque_inertia, 'LineWidth', 1.5)
%
%  legend('Tmi', 'Stiction', 'Friction', 'Inertia');

figure(14)
plot(qd(1,1:nel),Tfriccion(1,1:nel),'LineWidth',2)
grid on
hold on
title('$\tau_{f}$ vs. $\omega$', 'interpreter','latex')
xlabel('$\omega$ (rad/s)', 'interpreter','latex')
ylabel('$\tau_{f}$ (N m)', 'interpreter','latex')
legend('LuGre', 'Dahl+viscous',  'Location','southeast')
print(gcf, '-dpdf', 'Lugre-Dahl.pdf')

figure(15)
plot(qd(1,1:nel), zf(1,1:nel),'LineWidth',2)
grid on
hold on
%plot(t,zf(2,1:nel))

title('z vs. vel')
end