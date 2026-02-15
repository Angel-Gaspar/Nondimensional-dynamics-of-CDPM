% Create a trapezoidal profile for the components of a vector
% and synchronize the three actuators, so they arrive at once
function [t, pos, vel, acc] = Create_Reference_Synchr(dt, distance, abs_vmax, t_accel, sinusoidal_ref)


signo = sign(distance);
accel = abs_vmax ./ t_accel ;
vmax = signo .* abs_vmax;

[t_acc, t_cruise, t_dec] = trapezoidal_times(abs(distance), accel, accel, abs_vmax);
accel = accel .* signo;
%% Variables definitions and declarations
ttotal = t_acc + t_cruise+ t_dec;

t = 0:dt: (max(ttotal)+dt);

% ttotal = ttotal / max(ttotal);
unos = t_cruise*0+1;
t_cruise =unos * max(t_cruise);
t_acc =  (max(ttotal) - t_cruise)/2;
t_dec = t_acc;
vmax = distance ./ (t_cruise + t_acc);
accel = vmax ./ t_acc;

numt = numel(t);
num_rows = numel(distance);
pos = zeros(num_rows, numt);
vel = zeros(num_rows, numt);
acc = zeros(num_rows, numt);

for i = 1: numel(distance)
    if sinusoidal_ref
        [pos(i,:), vel(i,:), acc(i,:)] = sinusoidal_position_vel_acc(t, distance(i), accel(i), abs(vmax(i)), t_acc(i), t_dec(i), t_cruise(i));
    else
        [pos(i,:), vel(i,:), acc(i,:)] = trapezoidal_position_vel_acc(t, distance(i), accel(i), accel(i), abs(vmax(i)), t_acc(i), t_dec(i), t_cruise(i));
    end
end