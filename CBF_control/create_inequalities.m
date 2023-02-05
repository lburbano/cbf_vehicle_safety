function [A, B] = create_inequalities( k_1, k_2, lie_derivatives_B_1, lie_derivatives_B_2, M )
    % receives lie derivatives
    % B, LfB, L2fB, L3fB, LgLf2B

    % B_1
    b_1 = lie_derivatives_B_1(1);
    % LfB, L2fB, L3fB
    lies_f_B_1  = lie_derivatives_B_1(2:end-1);
    LgLf2B_1 = lie_derivatives_B_1(end);
    
    % B_2
    b_2 = lie_derivatives_B_2(1);
    % LfB, L2fB, L3fB
    lies_f_B_2  = lie_derivatives_B_2(2:end-1);
    LgLf2B_2 = lie_derivatives_B_2(end);
    
    B_1 = -k_1(2:4) * lies_f_B_1; % - LfB - L2fB - L3fB
    B_2 = -k_2(2:4) * lies_f_B_2; % - LfB - L2fB - L3fB
    A_1 = [k_1(5)*LgLf2B_1,               0, k_1(1) * b_1 + k_1(2:3)*lies_f_B_1(1:2),                                       0];
    A_2 = [              0, k_2(5)*LgLf2B_2,                                       0, k_2(1) * b_2 + k_2(2:3)*lies_f_B_2(1:2)];
    
%     A = [A_1; A_2;0 1 0 0; 0 -1 0 0;1 0 0 0;-1 0 0 0];
%     B = [B_1; B_2; 10; 10; 10; 10];
    A = [A_1; A_2; M*0 zeros(2,2); -M*0 zeros(2,2) ];
    B = [B_1; B_2; 5; 5; 5; 5];
    A = [A_1; A_2];
    B = [B_1; B_2];
    
    
end