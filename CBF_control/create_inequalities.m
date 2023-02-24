function [A, B] = create_inequalities( k_1, k_2, lie_derivatives_B_1, lie_derivatives_B_2, M, Lf3P, Lf3S, x5, x4, saturator_v, saturator_d )
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
    L = [Lf3P; Lf3S];
    M = [M(:, 2) M(:, 1)];
    L = M*L;
    A = [A_1; A_2; M zeros(2,2); -M zeros(2,2) ];
    
    max_acc = 100;
    max_heading_vel = 1;
    B = [B_1; B_2; 0; 0; 0; 0];
    if x5 > saturator_v - 0.05 % If 
%         B = [B_1; B_2; 0 + L(1); max_heading_vel + L(2); max_acc - L(1); max_heading_vel - L(2)];
        B(4, :) = 0 + L(1) - x5; % Acceleration should be negative
        B(6, :) = max_acc - L(1);
    elseif x5 < 0.05
%         B = [B_1; B_2; max_acc + L(1); max_heading_vel + L(2); 0 - L(1); max_heading_vel - L(2)];
        B(4, :) = max_acc + L(1); % Acceleration should be positive
        B(6, :) = 0 - L(1);
    else
%         B = [B_1; B_2; max_acc + L(1); max_heading_vel + L(2); max_acc - L(1); max_heading_vel - L(2)];
        B(4, :) = max_acc + L(1); % Acceleration should be positive
        B(6, :) = max_acc - L(1);
    end
    if x4 > saturator_d - 0.01
        B(3, :) = 0 + L(1);
        B(5, :) = max_heading_vel - L(2);
    elseif x4 < -saturator_d + 0.01
        B(3, :) = max_heading_vel + L(2);
        B(5, :) = 0 - L(1);
    else
        B(3, :) = max_heading_vel + L(2);
        B(5, :) = max_heading_vel - L(2);
    end
    A = [A_1; A_2];
    B = [B_1; B_2];
    
end