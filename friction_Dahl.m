function [T_f, zf] = friction_Dahl(omega, dt, Z,  prima_volta )
% DAHL_FRICTION - Modelo de fricción de Dahl 
%
% Entradas:
%   v      - velocidad relativa (rad/s)
%   z_prev - estado interno en el paso anterior
%   dt     - paso de integración [s]
%   Tc     - fricción de Coulomb [N·m ]
%   sigma  - rigidez de fricción [N·m/rad]
%
% Salidas:
%   T_f    - torque de fricción en este paso
%   z_new  - estado interno actualizado
%
% Ecuaciones:
%   z_dot = v - (|v|/Fc) * sigma * z
%   T_f   = sigma * z

persistent z
if prima_volta
    z = zeros(size(omega));
end
    
    % Dinámica de Dahl
    z_dot = omega - (abs(omega)/Z.Tc) * Z.sigma0 .* z;
    z = z + dt * z_dot;
    
    % Salida
    T_f = Z.sigma0 * z + Z.Beq * omega;
    zf = z;
end