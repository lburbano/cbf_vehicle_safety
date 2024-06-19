function [B, LfB, L2fB, L3fB, LgLf2B] = lie_derivatives_B_1(x, epsilon1, n0)
    B = epsilon1^2 - wrapToPi(n0 - x(1))^2;
    LfB = x(2) * wrapToPi(2*n0 - 2*x(1));
    L2fB = -2 * x(2)^2 + 2 * x(3) * wrapToPi(n0 - x(1));
    L3fB = -6 * x(2) * x(3);
    LgLf2B = 2 * wrapToPi(n0 - x(1));
end