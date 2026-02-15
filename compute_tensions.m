function [tfinal_min, tfinal_max] = compute_tensions(A, wrench, min_tension, max_tension)
% Funcion inputs:
%   A : structural matrix (n x m)
%   w : vector wrench (n x 1)
%   tmin : lower llimit for tension (m x 1), default  = 0
%   tmax : límite superior de tensión (m x 1), optional
%
% Outputs:
%   t_pinv : solution with pseudoinverse (no restrictions)
%   t_nn   : solution NNLS nnon negative least squares (t >= 0)
%   t_qp   : solution QP quadratic optimization with limits

%wrench = -wrench;
    m = size(A,2);

    %  Pseudoinverse
    t_pinv = pinv(A) * wrench;

    % NNLS
    %t_nn = lsqnonneg(A, -wrench);

    % Nullspace of A
    vnull = null(A);
    lambda_min = (min_tension - t_pinv)./vnull;
    %lambda_min = min(lambda)
    lambda_min_max = max(lambda_min);
    %tfinal_min = t_nn + lambda_min*vnull
    tfinal_min = t_pinv + lambda_min_max*vnull;
    
    if nargin < 4
        tfinal_max = tfinal_min;
        return;
    end
    lambda_max = (max_tension - t_pinv)./vnull;
    %lambda_min = min(lambda)
    lambda_max_min = min(lambda_max);
    %tfinal_min = t_nn + lambda_min*vnull
    tfinal_max = t_pinv + lambda_max_min*vnull;

end
