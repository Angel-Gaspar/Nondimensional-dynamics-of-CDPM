function A = structural_matrix_from_lengths_2D(L, B)
%   Input arguments:
%       U : matrix 3 x m with unitary director vectors of cables
%       B : matriz 3 x m with the positions of vertices w.r.t. the EE center

 
    m = size(L, 1);         % number of cables
    A = zeros(3, m);

    for i = 1:m
        ui = L(i,:) / norm(L(i,:));       % vector unitario del cable i
        bi = B(i,:);       % posición del anclaje del cable i
        A(:,i) = [ui, -ui(1)*bi(2) + ui(2)*bi(1)];
    end
end