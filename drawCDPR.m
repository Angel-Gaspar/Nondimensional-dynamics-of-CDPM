function drawCDPR(PAP, DAP_local, Pose, T, We)
% drawCDPR Dibuja un CDPR 2D con end-effector rectangular
%
% Parámetros:
%   frame_w, frame_h -> dimensiones del frame
%   w, h             -> dimensiones del end-effector
%   x, y             -> posición del centro del end-effector
%   theta            -> ángulo de rotación del end-effector (rad)
%   T                -> vector [4x1] de tensiones en los cables

max_PAP_x = max(PAP(:,1));
min_PAP_x = min(PAP(:,1));
max_PAP_y = max(PAP(:,2));
min_PAP_y = min(PAP(:,2));
W = max_PAP_x - min_PAP_x;
H = max_PAP_y -  min_PAP_y;

x = Pose(1);
y = Pose(2);
theta = Pose(3);
escala = 1 / max(abs(T))*min(W,H)*0.1;


% Matriz de rotación
R = [cos(theta), -sin(theta);
    sin(theta),  cos(theta)];

% Transformar al sistema global
ee_global = (R*DAP_local')' + [x, y];

% Iniciar figura
cla
hold on;
plot([PAP(:,1); PAP(1,1)], [PAP(:,2); PAP(1,2)], 'k--'); % frame
patch(ee_global(:,1), ee_global(:,2), [0.8 0.8 1], 'FaceAlpha',0.6); % end effector

% Dibujar cables y calcular wrench
wrench_force = We(1:2);
wrench_moment = We(3) ;

sign = [1, -1, 1, -1]*0.5;
justification = {'right', 'left', 'left', 'right'};

for i = 1:4
    % cable
    plot([PAP(i,1), ee_global(i,1)], [PAP(i,2), ee_global(i,2)], 'b');

    % dirección del cable
    dir = PAP(i,:) - ee_global(i,:);
    dir = dir / norm(dir);

    % flecha de tensión
    quiver(ee_global(i,1), ee_global(i,2), T(i)*dir(1)*escala, T(i)*dir(2)*escala, ...
        'r', 'LineWidth',1.5, 'MaxHeadSize',2);
    escala2 = min(W,H)*0.1;
    posx = ee_global(i,1) + T(i)*dir(1)*escala + 0* (dir(2)*escala2) * sign(i);
    posy = ee_global(i,2) + T(i)*dir(2)*escala - dir(1)*escala2 * sign(i);
    texto = num2str(T(i), '%.2f N');
    text( posx, posy,texto, ...
        'FontSize', 12, ...
        'HorizontalAlignment', justification{i}, ...   % centrar horizontalmente
        'VerticalAlignment', 'middle')
    % aporte al wrench
    wrench_force = wrench_force + T(i)*dir';
    r = ee_global(i,:)' - [x;y];
    wrench_moment = wrench_moment + det([r, T(i)*dir']);
end
axis tight
% Dibujar fuerzas resultantes por separado
quiver(x, y, wrench_force(1)*escala, 0, 'm', 'LineWidth',2, 'MaxHeadSize',2);
quiver(x, y, 0, wrench_force(2)*escala, 'm', 'LineWidth',2, 'MaxHeadSize',2);

% Dibujar fuerza total (vector resultante)
quiver(x, y, wrench_force(1)*escala, wrench_force(2)*escala, ...
    'k', 'LineWidth',2, 'MaxHeadSize',3);
xlabel('X (m)'); ylabel('Y (m)');
%title('CDPR with Tensions and Wrench');
grid on;
hold off;
if abs(wrench_moment) < 0.0001
    return;
end
% Dibujar par resultante como un arco circular
ang = linspace(0, pi/2, 30);
radius = 0.1*max(W,H);
signo = sign(wrench_moment);

% sentido antihorario
arcX = x + radius*cos(ang*signo);
arcY = y + radius*sin(ang*signo);
plot(arcX, arcY, 'b', 'LineWidth',2);

tip = [arcX(end), arcY(end)];
ang1 = ang(end) + 0.2 - signo * 0.05;
ang2 = ang(end) - 0.2 - signo * 0.05;
base1 = tip - [-signo*0.2*radius*sin(ang1*signo), 0.2*radius*cos(ang1*signo)];
base2 = tip - [-signo*0.2*radius*sin(ang2*signo), 0.2*radius*cos(ang2*signo)];

patch([tip(1) base1(1) base2(1)], [tip(2) base1(2) base2(2)], 'b');

%     quiver(arcX(end-1), arcY(end-1),...
%         -signo*0.2*radius*sin(ang(end)*signo), 0.2*radius*cos(ang(end)*signo), ...
%            'b', 'MaxHeadSize',2, 'LineWidth',4.5);

% Texto del par
%     text(x+radius*0.7, y+radius*0.7, sprintf('M=%.2f', wrench_moment), ...
%          'FontWeight','bold', 'Color','k');


end
