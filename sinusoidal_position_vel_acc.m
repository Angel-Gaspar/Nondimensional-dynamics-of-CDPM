function [x, vel, acc] = sinusoidal_position_vel_acc(t, dist, abs_acc, abs_vc, t_acc, t_dec, t_cruise)
% x = trapezoidal_position(t, d, a, dec, vc)
% Calcula la posición instantánea para un perfil sinusoidal
%
% Entradas:
%   t   = tiempo transcurrido desde el inicio (s) (puede ser escalar o vector)
%   d   = distancia total (m)
%   a   = aceleración (m/s^2)
%   dec = deceleración (m/s^2)
%   vc  = velocidad de crucero (m/s)
%
% Salida:
%   x   = posición instantánea (m)

    % Obtener tiempos de cada fase
    % [t_acc, t_cruise, t_dec] = trapezoidal_times(d, a, dec, vc);

    % Distancias en aceleración y deceleración
    signo = sign(dist);
    aceler = signo * abs_acc;
    vmaxima = abs_vc * signo;
    dt = t(2)-t(1);
    if t_cruise > 0
        % The travelled distance is the same as in the trapezoidal 
        d_acc = 0.5 * aceler * t_acc^2;
        d_dec = 0.5 * aceler * t_dec^2;
    else
        % Perfil triangular
        vmaxima = aceler * t_acc; % o dec * t_dec
        d_acc = 0.5 * aceler * t_acc^2;
        d_dec = 0.5 * aceler * t_dec^2;
    end

    % Inicializar vector de posiciones
    x = zeros(size(t));
    vel = x;
    acc = x;
    dt = t(2) - t(1);
    for i = 1:length(t)
        if t(i) < t_acc + dt
            V0 = 0;
            Vf = vmaxima;
            s_incV = (Vf-V0)/2;
            ssV = (Vf + V0) / 2;
            A = pi * t(i) /t_acc;
            % Fase de aceleración
            vel(i) = ssV - s_incV * cos(A) ;
            if i == 1
                x(i) = 0;
            else
             x(i) = x(i-1) + vel(i)*dt;
            end
            % x(i) = ssV * t(i) - s_incV * t_acc / pi * sin(A) ;
            
            acc(i) = s_incV * sin(A) *pi / t_acc;
            d_start_decel = ssV * t_acc ; % only if there is no cruise
        elseif t(i) < t_acc + t_cruise
            % Fase de crucero
            %x(i) = d_acc + vmaxima * (t(i) - t_acc);
            d_start_decel = x(i);
            vel(i) = vmaxima;
            x(i) = x(i-1) + vel(i)*dt;
            acc(i) = 0;
        elseif t(i) < t_acc + t_cruise + t_dec
            % Fase de deceleración
           Vf = 0;
            V0 = vmaxima;
            s_incV = (Vf-V0)/2;
            ssV = (Vf + V0) / 2; 
            td = t(i)-t_acc - t_cruise;
            A = pi * td /t_acc;
           
            % Fase de aceleración
            %x(i) =  d_start_decel + ssV * td - s_incV * t_acc / pi * sin(A) ;
            vel(i) = ssV - s_incV * cos(A) ;
            x(i) = x(i-1) + vel(i)*dt;
            acc(i) = s_incV * sin(A) *pi / t_acc;
        else
            % Movimiento terminadove
            x(i) = dist;
            vel(i) = 0;
            acc(i) = 0;
        end
    end
end

