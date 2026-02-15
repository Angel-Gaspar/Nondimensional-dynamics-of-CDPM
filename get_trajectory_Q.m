function [traj_Q, tension] = get_trajectory_Q_v2(traj_Xee, Z, wrench, min_pretension)


num_data = size(traj_Xee,2);
traj_Q = zeros(Z.m, num_data);
tension = zeros(Z.m, num_data);

for i = num_data:-1:1
    [Lengths, lengths] = ik_cable_robot(traj_Xee(:,i), Z.PAP, Z.DAP_local, 0);
    
    A = structural_matrix_from_lengths_2D(Lengths, Z.DAP_local);
    
    % Get Kelastic
    Kel = Z.K_elastic_per_length ./ lengths;

    % Get Tensions
    [tension(:, i), ~] = compute_tensions(A, wrench, min_pretension);
    incr_lengths = lengths - Z.L0_mod ;
    elongation_due_to_elasticity = tension(:, i) ./ Kel;
    total_elongation = elongation_due_to_elasticity - incr_lengths; % alpha positive produce positive tension
    alphas = total_elongation / Z.rdrum * Z.rgt; 
    traj_Q(:,i) = + alphas;
end