function [u, integ_error] = PID(error, Z, dt, integ_error, is_prima_volta)

persistent prev_error;

Kd = Z.Kd;
if is_prima_volta 
    prev_error = error * 0;
    Kd = 0;
end

derivative_error = (error - prev_error) / dt;

if Z.Ki == 0 % to avoid NaN problems
    u  = Z.Kp * error + Kd * derivative_error ;
else
    u  = Z.Kp * error + Z.Ki * integ_error + Kd * derivative_error ;
end
u  = min(Z.u_limit, max(u, -Z.u_limit));
integ_error = integ_error + error * dt ; 
prev_error = error;

