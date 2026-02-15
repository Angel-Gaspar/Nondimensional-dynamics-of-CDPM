function [tau_friction, zf] = friction_LuGre(omega, dt, Z,  prima_volta )

%% LuGre Model
persistent z
if prima_volta
    z = zeros(size(omega));
end


%% --- Parámetros del modelo ---
% Parámetros curva de Stribeck (torque en el eje de salida)
%Ts = 2.5;     % Torque estático [N·m]
%Tc = 1.5;     % Torque de Coulomb [N·m]

% Parámetros internos LuGre

%B = 0.05;   % fricción viscosa [N·m·s/rad]


%% --- Modelo LuGre rotacional ---
% Función g(omega) curva Stribeck
g_fun = @(w) Z.Tc + (Z.Ts - Z.Tc) .* exp(-(w ./ Z.omega_s).^2);

% Variables
gv = g_fun(omega);

zdot = omega - Z.sigma0 * (abs(omega)./gv) .* z;

tau_friction= Z.sigma0 * z + Z.sigma1 * zdot + omega * Z.Beq;

z = z + dt * zdot;
zf = z;

