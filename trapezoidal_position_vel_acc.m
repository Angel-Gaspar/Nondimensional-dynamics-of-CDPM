function [x, vel, acc] = trapezoidal_position_vel_acc(t, dist, abs_acc, abs_dec, abs_vc, t_acc, t_dec, t_cruise)
% x = trapezoidal_position(t, d, a, dec, vc)
% Calcula la posición instantánea para un perfil trapezoidal
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
    if t_cruise > 0
        % Perfil trapezoidal
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
    
    for i = 1:length(t)
        if t(i) <= t_acc
            Vf = vmaxima;
            % Fase de aceleración
            x(i) = 0.5 * aceler * t(i)^2 ;
            vel(i) = aceler * t(i) ;
            acc(i) = aceler;
        elseif t(i) < t_acc + t_cruise
            % Fase de crucero
            x(i) = d_acc + vmaxima * (t(i) - t_acc);
			x_start_dec = x(i);			 
            vel(i) = vmaxima;
            acc(i) = 0;
        elseif t(i) <= t_acc + t_cruise + t_dec
            % Fase de deceleración
            t_dec_phase = t(i) - t_acc - t_cruise;
            vel(i) = vmaxima - aceler * t_dec_phase;
            x(i) = x_start_dec + vmaxima * t_dec_phase - 0.5 * aceler * t_dec_phase^2;
            acc(i) = abs_dec;
        else
            % Movimiento terminado
            x(i) = dist;
            vel(i) = 0;
            acc(i) = 0;
        end
    end
end

