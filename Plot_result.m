function Plot_result(result, index)
to_hd = 1; % Only to avoid converting into pdf
%% Plot Result
t = result.t;

set(0,'defaultAxesFontSize',12)
nel = numel(t);
sep = 0.435;
figure(11)

subplot(3,2,1+index)
plot (result.t, result.alpha, 'LineWidth', 1.5)
axis tight
legend('1. D-L', '2. D-R', '3. U-R', '4. U-L','location','southwest');
if index == 0
    title('${\alpha}\,,\, \dot{\alpha}\,,\, \ddot{\alpha}$', 'interpreter','latex');
    ylabel('$\alpha$ (rad)', 'interpreter','latex');
else
    title('${\alpha}^{\mathrm{pu}}\,,\, \dot{\alpha}^{\mathrm{pu}}\,,\, \ddot{\alpha}^{\mathrm{pu}}$', 'interpreter','latex');
    ylabel('$\alpha^{\textrm{pu}}$ (rad) = $\alpha $', 'interpreter','latex');
    annotation('arrow',0.372 + [sep 0],[0.8956 0.8956]);
    text(22,15,'=','FontSize',14, 'interpreter','latex');
end


subplot(3,2,3+index)
plot(t, result.alpha_der, 'LineWidth', 1.5)
if index == 0
    ylabel('$\dot{\alpha}$ (rad/s)', 'interpreter','latex');
else
    ylabel('$\dot{\alpha}^{\mathrm{pu}}$ (pu) = $\dot{\alpha}\,/\, \omega_{\mathrm{ref}} $', 'interpreter','latex');
    annotation('arrow',0.365 + [sep 0],[0.6 0.6]);
    text(62,0.54,'$\times\; \omega_{\mathrm{ref}}^{-1}$','FontSize',14, 'interpreter','latex');
end
legend('1. D-L', '2. D-R', '3. U-R', '4. U-L','location','southwest');
axis tight

subplot(3,2,5+index)
plot(t, result.alpha_dder, 'LineWidth', 1.5)
axis tight
if index == 0
    xlabel('t (s)');
    ylabel('$\ddot{\alpha}$ (rad/s)', 'interpreter','latex');
else
    ylabel('$\ddot{\alpha}^{\mathrm{pu}}$ (pu) = $\ddot{\alpha}\,/\, \omega_{\mathrm{ref}}^2 $', 'interpreter','latex');
    annotation('arrow',0.30 + [sep 0],[0.3 0.3]);
    text(10,0.054,'$\times\; \omega_{\mathrm{ref}}^{-2}$','FontSize',14, 'interpreter','latex');
    xlabel('t$^{\textrm{\textrm{pu}}}$ (p.u.) = t$\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
end
fig = gcf;
fig.Units    = 'centimeters';
fig.Position = [0 0 30 20];
if index == 1 && to_hd
    exportgraphics(gcf, 'Fig_Traj-Actuator.pdf','ContentType','vector')
end


%
figure(12)
subplot(3,2,1+index)
plot(t, result.Xeeref(1:2,:), 'LineWidth', 1.5)
axis tight
hold on
plot(t, result.Xee(1:2,:), 'LineWidth', 1.5)

if index == 0
    title('Xee, Vee, Aee', 'interpreter','latex');
    ylabel('Xee (m)', 'interpreter','latex');
else
    title('Xee$^{\textrm{pu}}$, Vee$^{\textrm{pu}}$, Aee$^{\textrm{pu}}$', 'interpreter','latex');
    ylabel('Xee$^{\textrm{pu}}$ (p.u.) = Xee / $l_{\mathrm{ref}} $', 'interpreter','latex');
    annotation('arrow',[0.815 0.375],[0.8556 0.8556]);
    text(22,15,'$\times \, l_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
end
legend('ref X','ref Y','X','Y','location','southwest')
grid


subplot(3,2,3+index)
plot(t, result.Vee(1:2,:), 'LineWidth', 1.5)
axis tight
if index == 0
    ylabel('Vee (m/s)', 'interpreter','latex');
else
    ylabel('Vee$^{\mathrm{pu}}$ (p.u.) = Vee $\omega_{\mathrm{ref}}^{-1} \,/\, l_{\mathrm{ref}}$', 'interpreter','latex');
    annotation('arrow',[0.715 0.2805],[0.60 0.60]);
    text(10,0.43,'$\times \,  l_{\mathrm{ref}}\, \,\omega_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
end
legend('X','Y','location','southwest')
grid
axis tight

subplot(3,2,5+index)
plot(t, result.Aee(1:2,:), 'LineWidth', 1.5)
axis tight
if index == 0
    ylabel('Aee\, (m/s$^2$)', 'interpreter','latex');
    xlabel('t (s)');

else
    ylabel('Aee$^{\mathrm{pu}}$ (p.u.) = Aee $\omega_{\mathrm{ref}}^{-2}\,/\, l_{\mathrm{ref}} $', 'interpreter','latex');
    xlabel('t$^{\textrm{\textrm{pu}}}$ (p.u.) = t$\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
    annotation('arrow',[0.87 0.433],[0.295 0.295]);
    text(52,0.045,'$\times \,  l_{\mathrm{ref}}\, \,\omega_{\mathrm{ref}}^{2}$','FontSize',14, 'interpreter','latex');
end
legend('X','Y','location','southwest')
hold on
grid
hold off

fig = gcf;
fig.Units    = 'centimeters';
fig.Position = [0 0 30 20];
if index == 1 && to_hd

    exportgraphics(gcf, 'Fig_Traj-EE.pdf','ContentType','vector')
end
%%

figure(13)
subplot(1,2, index+1)
plot(t, result.Torque','LineWidth',2)
axis tight
grid
legend('1. D-L', '2. D-R', '3. U-R', '4. U-L','location','southwest');

if index == 0
    ylabel('$\tau$ (N m)', 'interpreter','latex');
    title('Inner mechanical torque');
    xlabel('t (s)');
else
    title('Inner mechanical torque in p.u.');
    ylabel('$\tau_{\textrm{pu}}$ (p.u.) = $\tau \,/\, \tau_{\mathrm{ref}}$', 'interpreter','latex');
    xlabel('t$_{\textrm{pu}}$ (p.u.) = t$\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
    annotation('arrow',0.37+[sep -0.007],[0.868 0.868]);
    text(134, 0.06,'$\tau\,^{\textrm{fr}}_{\textrm{UR}} - \tau\,^{\textrm{fr}}_{\textrm{UL}}$', 'Rotation', 90, 'interpreter','latex');
    text(62,0.436,'$\times \, \tau_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
    h = annotation('doublearrow',[0.912 0.912],[0.55 0.5]);
    h.Head1Length = 4;  % valor por defecto ≈ 10
    h.Head2Length = 4;
end
fig = gcf;
fig.Units    = 'centimeters';
fig.Position = [0 0 35 14];

if index == 1 && to_hd

    exportgraphics(gcf, 'Fig_Tmi.pdf','ContentType','vector')
end
%%
figure(14)
subplot(1,2, index+1)
plot(t,result.Tension','LineWidth',2)
axis tight
if index == 0
    ylabel('f (N)', 'interpreter','latex');
    xlabel('t (s)');
    title('Tension');
else
    title('Tension in p.u.');
    ylabel('f$^{\mathrm{pu}}$ (p.u.) = f$ \, l_{\mathrm{ref}}\,/\,\tau_{\mathrm{ref}}$', 'interpreter','latex');
    xlabel('t$^{\mathrm{pu}}$ (p.u.) = t$\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
    annotation('arrow',0.27 + [0.435 0.00],[0. 0.]+0.8);
    text(12,0.154,'$\times \,  \tau_{\mathrm{ref}}\, / \,l_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
end
grid on
legend('1. D-L', '2. D-R', '3. U-R', '4. U-L','location','southwest');

if index == 1 && to_hd
    fig = gcf;
    fig.Units    = 'centimeters';
    fig.Position = [0 0 35 14];
    exportgraphics(gcf, 'Fig_Tension.pdf','ContentType','vector')
end
%
sep = 0.44;
figure(15)

%    %legend('LuGre', 'Dahl+viscous',  'Location','southeast')
%    %print(gcf, 'Fig_-dpdf', 'Lugre-Dahl.pdf')
%
%    % figure(16)
%    %     subplot(1,2, index+1)
%    % plot(result.alpha_der(1,:), result.zf(1,:),'LineWidth',2)
%    % grid on
%    % hold on
%    % %plot(t,zf(2,1:nel))
%    %
%    % title('z vs. vel')

%figure(16)
subplot(2,2, index+1)
plot(t,result.Tfriction','LineWidth',2)
axis tight
if index == 0
    ylabel('$\tau$ (N m)', 'interpreter','latex');
    xlabel('t (s)');
    title('Friction Torque in N m');
else
    title('Friction Torque in p.u.');
    ylabel('$\tau^\textrm{fr}_{\textrm{pu}}$ (p.u.) = $\tau^\textrm{fr} \,/\, \tau_{\mathrm{ref}}$', 'interpreter','latex');
    xlabel('t$_{\textrm{pu}}$ (p.u.) = t$\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
    annotation('arrow', 0.218 + [sep 0], 0.855 +[0 0]);
    text(5,0.2,'$\times \,  \tau_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
    text(134, -0.08,'$\tau\,^{\textrm{fr}}_{\textrm{UR}} - \tau\,^{\textrm{fr}}_{\textrm{UL}}$', 'Rotation', 90, 'interpreter','latex');
    h = annotation('doublearrow',[0.912 0.912],[0.74 0.77]);
    h.Head1Length = 4;  % valor por defecto ≈ 10
    h.Head2Length = 4;
end

grid on
legend('1. D-L', '2. D-R', '3. U-R', '4. U-L','location','southwest');

subplot(2,2, index+1+2)
axis tight
%plot(result.Torque(:,:) -  result.Tension(:,:)*KRob.rdrum - result.alpha_der(1,:)*KRob.Beq*0,result.Tfriction(1,:),'LineWidth',2)
%plot(result.alpha_der(3:4,:)',result.Tfriction(3:4,:)','LineWidth',2)
plot(result.alpha_der(3,:)',result.Tfriction(3,:)','LineWidth',3)
hold on
plot(result.alpha_der(4,:)',result.Tfriction(4,:)','LineWidth',2)
if index == 0
    ylabel('$\tau_\textrm{fr}$ (N m)', 'interpreter','latex')
    xlabel('$\Omega$ (rad/s)', 'interpreter','latex')
    title('Friction torque vs. rot. speed');
else
    title('Friction torque vs. rot. speed, in p.u.');
    ylabel('$\tau^\textrm{fr}_\textrm{\textrm{pu}}$ (p.u.) $= \tau^\textrm{fr} \, / \,\tau_{\mathrm{ref}}$', 'interpreter','latex');
    xlabel('$\Omega^{\mathrm{pu}}$ (p.u.) = $\Omega\,/\,\omega_{\mathrm{ref}}$', 'interpreter','latex');
    axis([-0.3 0.3 -0.1 0.1]/3)
    %annotation('arrow',0.3 + [sep 0],[0 0]+0.345);
    %text(-0.8,0.01605,'$\times \,  \tau_{\mathrm{ref}}$','FontSize',14, 'interpreter','latex');
    text(0.006, -0.004,'$\tau^{\textrm{fr}}_{\textrm{UR,pu}} - \tau^{\textrm{fr}}_{\textrm{UL,pu}}$', 'interpreter','latex');
    annotation('doublearrow',[0.748 0.748],[0.15 0.420])
end
grid on

legend('3. U-R', '4. U-L','location','northwest');
fig = gcf;
fig.Units    = 'centimeters';
fig.Position = [0 0 30 20];
if index == 1 && to_hd

    exportgraphics(gcf, 'Fig_Friction-omega.pdf','ContentType','vector')
end

end
