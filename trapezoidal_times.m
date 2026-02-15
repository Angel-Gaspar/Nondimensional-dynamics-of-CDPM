function [t_acc, t_cruise, t_dec] = trapezoidal_times(d, a, dec, vc)
% [t_acc, t_cruise, t_dec] = trapezoidal_times(d, a, dec, vc)
% Calcula los tiempos de aceleración, crucero y deceleración
% para un perfil trapezoidal (o triangular si no se alcanza vc).
%
% Entradas:
%   d   = distancia total (m)
%   a   = aceleración (m/s^2)
%   dec = deceleración (m/s^2) (valor positivo)
%   vc  = velocidad de crucero (m/s)
%
% Salidas:
%   t_acc    = tiempo de aceleración (s)
%   t_cruise = tiempo de crucero (s)
%   t_dec    = tiempo de deceleración (s)


t_cruise = d*0;
    % Tiempo para acelerar hasta vc
    t_acc = vc ./ a;
    % Distancia recorrida en aceleración
    d_acc = 0.5 * a .* t_acc.^2;

    % Tiempo para decelerar desde vc
    t_dec = vc ./ dec;
    % Distancia recorrida en deceleración
    d_dec = 0.5 * dec .* t_dec.^2;

    % Distancia necesaria para aceleración + deceleración
    d_min = d_acc + d_dec;
for i = 1:numel(d_min)
    
    if d_min(i) >= d(i)
        % Perfil triangular: no se alcanza vc
        % Calcular velocidad máxima alcanzable
        vmax = sqrt((2 * a(i)* dec(i)* d(i)) / (a(i) + dec(i)));

        % Nuevos tiempos
        t_acc(i) = vmax / a(i);
        t_dec(i) = vmax / dec(i);
        t_cruise(i) = 0;
    else
        % Perfil trapezoidal: hay crucero
        d_cruise = d(i) - d_min(i);
        t_cruise(i) = d_cruise / vc(i);
    end
end
