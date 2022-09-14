function [B, LfB, L2fB, L3fB, LgLf2B] = lie_derivatives_B_2(x, epsilon2, xi0)
    B = epsilon2^2 - (x(1) - xi0)^2;
    LfB = x(2) * (2*xi0 - 2*x(1));
    L2fB = - 2 * x(2)^2 + x(3)*(2*xi0 - 2*x(1));
    L3fB = -6 * x(2) * x(3);
    LgLf2B = 2*xi0 - 2*x(1);
end