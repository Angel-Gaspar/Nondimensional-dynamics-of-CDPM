% Create a trapezoidal profile, and then add the same but inverted. 
% And repeat both profiles
function [t, pos, vel, acc] = Create_Reference_v2(dt, distance, abs_vmax, t_accel, t_still, num_periodds)

signo = sign(distance);
accel = abs_vmax / t_accel ;
vmax = signo * abs_vmax;


[t_acc, t_cruise, t_dec] = trapezoidal_times(abs(distance), accel, accel, abs_vmax);
accel = accel * signo;
%% Variables definitions and declarations

t = 0:dt: t_acc + t_cruise+ t_dec+0.2;
array_still = zeros(1,t_still/dt);

[qref_1, qdref_1, qddref_1] = trapezoidal_position_vel_acc(t, distance, accel, accel, vmax, t_acc, t_dec, t_cruise);
[qref_2, qdref_2, qddref_2]= trapezoidal_position_vel_acc(t, -distance, accel, accel, vmax, t_acc, t_dec, t_cruise);


qddref_1periodo = [array_still, qddref_1, array_still, qddref_2 + qddref_1(:,end), array_still];
qddref_1periodo =[qddref_1periodo; -qddref_1periodo  ];

qdref_1periodo = [array_still, qdref_1, array_still, qdref_2 + qdref_1(:,end), array_still];
qdref_1periodo =[qdref_1periodo; -qdref_1periodo  ];

qref_1periodo = [array_still, qref_1, array_still+qref_1(end), qref_2 + qref_1(:,end), array_still];
qref_1periodo =[qref_1periodo; -qref_1periodo  ];

pos = repmat(qref_1periodo, 1, num_periodds);
vel = repmat(qdref_1periodo, 1, num_periodds);
acc = repmat(qddref_1periodo, 1, num_periodds);

t = (1:size(vel,2))*dt;