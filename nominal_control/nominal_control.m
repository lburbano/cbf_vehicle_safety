function [v_tang, v_tran, v1_fbl_new, v2_fbl_new, M, xi, eta] = nominal_control(x, kc, L, r, v)
    k1 = kc(1);
    k2 = kc(2);
    k3 = kc(3);
    k4 = kc(4);
    k5 = kc(5);

    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);
    x5 = x(5);
    x6 = x(6);
    
    xi1 = - r^2 + x1^2 + x2^2;
    xi2 = 2*(v + x5)*(x1*cos(x3) + x2*sin(x3));
    xi3 = (v*sin(x3) + x5*sin(x3))*(2*v*sin(x3) + 2*x5*sin(x3)) + ((v*tan(x4))/L + (x5*tan(x4))/L)*(2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3))) + x6*(2*x1*cos(x3) + 2*x2*sin(x3)) + (v*cos(x3) + x5*cos(x3))*(2*v*cos(x3) + 2*x5*cos(x3));
    Lg1Lf2S = (2*x2*(v*cos(x3) + x5*cos(x3)) - 2*x1*(v*sin(x3) + x5*sin(x3)))*(v/(L*cos(x4)^2) + x5/(L*cos(x4)^2));
    Lg2Lf2S = 2*x1*cos(x3) + 2*x2*sin(x3);
    Lf3S = -(2*v^3*x1*cos(x3)*tan(x4)^2 - 6*L^2*x5*x6 - 6*L^2*v*x6 + 2*x1*x5^3*cos(x3)*tan(x4)^2 + 2*v^3*x2*sin(x3)*tan(x4)^2 + 2*x2*x5^3*sin(x3)*tan(x4)^2 + 6*v*x1*x5^2*cos(x3)*tan(x4)^2 + 6*v^2*x1*x5*cos(x3)*tan(x4)^2 + 6*v*x2*x5^2*sin(x3)*tan(x4)^2 + 6*v^2*x2*x5*sin(x3)*tan(x4)^2 - 6*L*v*x2*x6*cos(x3)*tan(x4) - 6*L*x2*x5*x6*cos(x3)*tan(x4) + 6*L*v*x1*x6*sin(x3)*tan(x4) + 6*L*x1*x5*x6*sin(x3)*tan(x4))/L^2;

    eta1 = atan2(x2,x1);
    eta2 = -(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3)))/(x1^2 + x2^2);
    eta3 = ((v*sin(x3) + x5*sin(x3))/(x1^2 + x2^2) + (2*x1*(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2)^2)*(v*cos(x3) + x5*cos(x3)) - (v*sin(x3) + x5*sin(x3))*((v*cos(x3) + x5*cos(x3))/(x1^2 + x2^2) - (2*x2*(x2*(v*cos(x3) + x5*cos(x3)) - x1*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2)^2) - (x6*(x2*cos(x3) - x1*sin(x3)))/(x1^2 + x2^2) + (((v*tan(x4))/L + (x5*tan(x4))/L)*(x1*(v*cos(x3) + x5*cos(x3)) + x2*(v*sin(x3) + x5*sin(x3))))/(x1^2 + x2^2);
    Lg1Lf2P = ((x1*(v*cos(x3) + x5*cos(x3)) + x2*(v*sin(x3) + x5*sin(x3)))*((v*(tan(x4)^2 + 1))/L + (x5*(tan(x4)^2 + 1))/L))/(x1^2 + x2^2);
    Lg2Lf2P = -(x2*cos(x3) - x1*sin(x3))/(x1^2 + x2^2);
    Lf3P = -((v + x5)*(tan(x4)*x1^2 - 2*L*sin(x3)*x1 + tan(x4)*x2^2 + 2*L*cos(x3)*x2)*(L*v^2*x1^2 + L*v^2*x2^2 + L*x1^2*x5^2 + L*x2^2*x5^2 + 2*L*v*x1^2*x5 + 2*L*v*x2^2*x5 + 2*L*v^2*x1^2*cos(2*x3) - 2*L*v^2*x2^2*cos(2*x3) + 2*L*x1^2*x5^2*cos(2*x3) - 2*L*x2^2*x5^2*cos(2*x3) - v^2*x2^3*cos(x3)*tan(x4) - x2^3*x5^2*cos(x3)*tan(x4) - 3*L*x1^3*x6*cos(x3) + v^2*x1^3*sin(x3)*tan(x4) + x1^3*x5^2*sin(x3)*tan(x4) - 3*L*x2^3*x6*sin(x3) - v^2*x1^2*x2*cos(x3)*tan(x4) - x1^2*x2*x5^2*cos(x3)*tan(x4) - 3*L*x1*x2^2*x6*cos(x3) + v^2*x1*x2^2*sin(x3)*tan(x4) + x1*x2^2*x5^2*sin(x3)*tan(x4) - 3*L*x1^2*x2*x6*sin(x3) + 4*L*v*x1^2*x5*cos(2*x3) - 4*L*v*x2^2*x5*cos(2*x3) + 4*L*v^2*x1*x2*sin(2*x3) + 4*L*x1*x2*x5^2*sin(2*x3) - 2*v*x2^3*x5*cos(x3)*tan(x4) + 2*v*x1^3*x5*sin(x3)*tan(x4) - 2*v*x1^2*x2*x5*cos(x3)*tan(x4) + 2*v*x1*x2^2*x5*sin(x3)*tan(x4) + 8*L*v*x1*x2*x5*sin(2*x3)))/(L^2*(x1^2 + x2^2)^3);

    % QP controller - Obstacle avoidance
    D = [Lg1Lf2P Lg2Lf2P;Lg1Lf2S Lg2Lf2S];
    M = inv(D);
    alpha = 0.95;
    alpha3 = 1;
    alpha2 = alpha;
    alpha1 = (alpha)/(2 - alpha);

    v_tang = -1*sign(eta1)*abs(eta1)^alpha1 - k4*sign(eta2-0.5)*abs(eta2-0.5)^alpha2 - k5*sign(eta3)*abs(eta3)^alpha3;
    v_tran = -k1*sign(xi1)*abs(xi1)^alpha1 - k2*sign(xi2)*abs(xi2)^alpha2 - k3*sign(xi3)*abs(xi3)^alpha3;
    v1_fbl_new = -Lf3P + v_tang;
    v2_fbl_new = -Lf3S + v_tran;

    xi = [xi1, xi2, xi3];
    eta = [eta1, eta2, eta3];

end