function [lengths_vector, lengths_mod] = ik_cable_robot(Pose, PAP, DAP_local, make_plot)
% ik_cable_robot - Cinemática inversa de robot planar 2T+R con 4 cables
%
% lengths = ik_cable_robot(x, y, theta, L, H, l, h)
%
% Entradas:
%   (x,y)  - centro del efector
%   theta  - orientación [rad]
%   L,H    - dimensiones del frame (ancho, alto)
%   d,h    - dimensiones del efector (ancho, alto)
%
% Salidas:
%   lengths - vector [l1; l2; l3; l4] con las longitudes de los 4 cables

% motor 1 is at Low Left. CCW
% Position 0,0 is at the frame center. EE orientation is CCW

    % We can assume that the center of the absolute coordinate system is
    % at the center of the frame. If not, the center position should be
    % included as input

    max_PAP_x = max(PAP(:,1));
    min_PAP_x = min(PAP(:,1));
    max_PAP_y = max(PAP(:,2));
    min_PAP_y = min(PAP(:,2));
    
    center_x = (max_PAP_x + min_PAP_x)/2;
    center_y = (max_PAP_y + min_PAP_y)/2;

    x = Pose(1);
    y = Pose(2);
    theta = Pose(3);

    % Matriz de rotación
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];

    % Calcular posiciones globales de los vértices del efector
    P_global = (R * DAP_local')' + [x+center_x y+center_y];

    % Vectors corresponding to cables
    lengths_vector = P_global - PAP;
    %lengths = sqrt(sum((P_global - PAP).^2, 2));
    lengths_mod = sqrt(sum(lengths_vector.^2, 2));

    %lengths_vector = -lengths_vector;
   
    if ~make_plot
        return;
    end
         % --- Dibujo ---
    figure; hold on; axis equal;
    % Marco
    rectangle('Position',[min_PAP_x min_PAP_y max_PAP_x-min_PAP_x max_PAP_y-min_PAP_y],'EdgeColor','k','LineWidth',2);
    % Efector
    fill(P_global(:,1), P_global(:,2),'r','FaceAlpha',0.3,'EdgeColor','r','LineWidth',1.5);
    % Cables
    for i=1:4
        plot([PAP(i,1) P_global(i,1)], [PAP(i,2) P_global(i,2)], 'b--','LineWidth',1.2);
        plot(PAP(i,1), PAP(i,2),'ko','MarkerFaceColor','k'); % motores
        plot(P_global(i,1), P_global(i,2),'ro','MarkerFaceColor','r'); % esquinas efector
    end
    % Centro del efector
    plot(x,y,'rx','MarkerSize',10,'LineWidth',2);

    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('Posición del efector: X=%.2f, Y=%.2f, θ=%.2f rad',x,y,theta));
    xlim([min_PAP_x-0.1  max_PAP_x+0.1]); ylim([min_PAP_y-0.1 max_PAP_y+0.1]);
    grid on;
end