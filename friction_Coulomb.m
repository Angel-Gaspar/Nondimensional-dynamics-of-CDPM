function [tau_friction] = friction_Coulomb(omega, Tresult, Z, using_speed)

if using_speed
    tau_friction =   Z.Tc *sign(omega) + omega * Z.B;
else
    tau_friction =   Z.Tc *sign(Tresult) + omega * Z.B;
end
for mu = 1:numel(omega)    
    if (Tresult(mu) < tau_friction(mu) && Tresult(mu) > 0)
        tau_friction(mu) = Tresult(mu) ;
    elseif (Tresult(mu) > tau_friction(mu) && Tresult(mu) < 0)
        tau_friction(mu) = Tresult(mu) ;

        %It should be sign(velocity), but this gives rise to oscillations.
        % If using something to avoid oscillations, then there is no
        % dead-zone in the torque, i.e. the pretension does not disappear
        % (which is not realistic)
        %Tmi(:,i) = Tmi(:,i) - stiction .* sign(qd(:,i));sign(Tmi(mu))
    end
end