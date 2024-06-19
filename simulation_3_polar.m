%% PathFollowing of Car-like Robot
clc
clearvars
close all
addpath('CBF_control')
addpath('camera_communication')
addpath('robot_communication')
addpath('nominal_control')
options = optimoptions('quadprog','Display','off');

%% Controller parameters 
v        = 0.1; %%% Not the linear velocity input but the fixed speed
k1       = 60;    %
k2       = 60;
k3       = 8;    %
k4       = 100;
k5       = 26;
kc       = [k1, k2, k3, k4, k5];
L        = 0.33;
r        = 2;     %% radius of the circle to follow

%% Obstacle avoidance parameters
epsilon1    = 0.3;
eta0        = -pi/4;
eta0        = [pi/2 3*pi/4 ];
% [0 -pi];
% [0 -pi/4 -pi/8]
% [-pi/2 0 -pi]
% [-pi/4]
epsilon2    = epsilon1;
xi0         = [];
n_obstacles = length(eta0);
xs_obs = [];
epsilons = [];
for i = 1 :length(eta0)
    p = r*[cos(eta0(i)), sin(eta0(i))];
    xs_obs = [xs_obs; p];
    epsilons = [epsilons; [epsilon1, epsilon2]];
    xi0(i) = p(1)^2 + p(2)^2 - r^2;
end
phi_0 = pi/2; % threshold angle

%%
%%% Initial Conditions
X1_0 = r; %% x-position
X2_0 = 0;       %% y-position
X3_0 = pi/2;    %% car orientation
X4_0 = 0;       %% Wheel angle
X5_0 = 0;       %% Fictitious state and we can always initialize it with zero
X6_0 = 0;       %% Fictitious state and we can always initialize it with zero

v_input = 0;

% Initial condition Vector
x0 = [X1_0; X2_0; X3_0; X4_0; X5_0; X6_0];


% Simulation time
Tmax = 20;  % End point
dt = 1/20; % Time step
T = 0:dt:Tmax; % Time vector
time = T;

% Simulation setup
x_all = zeros(size(x0,1),length(T));  % Intialization of a matrix to save all the values of the states 
x_Old = x0; % Giving x0 as a the name of an old state to use it in the discretized equations
x_all(:,1) = x0; % Saving the initial state in the full state matrix
xi1_plot = zeros(1,length(T));
xi2_plot = zeros(1,length(T));
xi3_plot = zeros(1,length(T));
eta1_plot = zeros(1,length(T));
eta2_plot = zeros(1,length(T));
eta3_plot = zeros(1,length(T));
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));

%%% Giving values to the individual componets for the ease of coding
x1_Old = x_Old(1);
x2_Old = x_Old(2);
x3_Old = x_Old(3);
x4_Old = x_Old(4);
x5_Old = x_Old(5);
x6_Old = x_Old(6);
v_input_Old = v_input;


%%% for the first iteration the current states are assumed to be the old
%%% states
x1 = x1_Old;
x2 = x2_Old;
x3 = x3_Old;
x4 = x4_Old;
x5 = x5_Old;
x6 = x6_Old;


saturator_v = 1;
saturator_d = pi/3;
safe_mode = zeros(length(eta0), 1);

tau_xi = 20; % radius
tau_eta = 0.3; % angle

dist_x = tau_xi * 0;
dist_theta = tau_eta * 0.7; % fail at 0.7

for i=1:length(T)-1
    %% Transformed states
    
    % Measure state   
    x = [x1, x2, x3, x4, x5, x6];

    % Nominal control to follow the circle
    % circle_control
    [v_tang, v_tran, v1_fbl_new, v2_fbl_new, M, xi, eta, Lf3P, Lf3S] = nominal_control(x, kc, L, r, v);
    eta1 = eta(1); eta2 = eta(2); eta3 = eta(3);
    xi1 = xi(1); xi2 = xi(2); xi3 = xi(3);
    
    
    x_veh = [x1, x2];
    [dist, closest_obs] = find_closest_obs(x_veh, x3, xs_obs, epsilons, phi_0);
    k_switch = 1;
%     for obs_iter = 1:n_obstacles
    
    x0 = xs_obs(closest_obs, :);
    Bar_F(i, :) = dist;
    avoid_obstacle = 1;
    A = [];
    B = [];
    jump_to_safe = 0; jump_to_invariance = 0;
    tau = 2*pi*r/10;
    epsilon = 0.1;
    
    tau_xi = 20; % radius
    tau_eta = 0.3; % angle
    
    epsilon_xi = 0;
    epsilon_eta = 0;
    mu = 1;
    lambda = 0;
    
    for j = 1:length(eta0)
        % Compute h_{i,\eta}
        d_eta(j) = wrapToPi( eta1 - eta0(j) );
        d_eta(j) = abs( d_eta(j) )^2 - asin( epsilons(j,1)/sqrt(xi0(j)+r^2))^2;
        % Compute h_{i,\xi}
        d_xi(j) = abs( xi1 - xi0(j) )^2 - epsilons(j,1)^2;
           
        % introduce disturbance in the switching law
        if safe_mode(j) == 0 && mu*d_eta(j) < tau_eta 
            d_eta_temp = d_eta(j) + dist_theta;
        else
            d_eta_temp = d_eta(j);
        end

        if safe_mode(j) == 0 && mu*d_xi(j) < tau_xi
            d_xi_temp = d_xi(j) + dist_x;
        else
            d_xi_temp = d_xi(j);
        end
        
        if safe_mode(j) == 0 && mu*d_eta_temp < tau_eta  && mu*d_xi_temp < tau_xi
            jump_to_safe = 1;
        elseif safe_mode(j) > 0 && (d_eta_temp > mu*tau_eta || d_xi_temp > mu*tau_xi)
            jump_to_invariance = 1;
        end

        % % if safe_mode(j) == 0 && mu*d_eta(j) < tau_eta  && mu*d_xi(j) < tau_xi
        % %     jump_to_safe = 1;
        % % elseif safe_mode(j) > 0 && (d_eta(j) > mu*tau_eta || d_xi(j) > mu*tau_xi)
        % %     jump_to_invariance = 1;
        % % end
    end
    
    if jump_to_safe
        cond = d_eta < tau_eta  & d_xi < tau_xi ;
        cond = cond';
        safe_mode = safe_mode + (ones(n_obstacles, 1) - safe_mode) .* (cond);
    elseif jump_to_invariance
        cond = (d_eta < tau_eta) & (d_xi < tau_xi);
        cond = cond';
        safe_mode = safe_mode .* (cond);
    end
    for j = 1:length(eta0)
        current_obs = j;
        if safe_mode(j) > 0 && avoid_obstacle
            eta_obstacle = eta0(current_obs) - pi/2;
            xi_obstacle = (sum(xs_obs(j, :).^2) - r^2);
            epsilon = max(epsilons(current_obs, :));
            [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1] = lie_derivatives_B_1(eta, epsilon, eta_obstacle);
            [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2] = lie_derivatives_B_2(xi, epsilon, xi_obstacle);
            lie_B_1 = [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1]';
            lie_B_2 = [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2]';

            k_1 = [1, 3, 4, 4, 3] ;
            k_2 = [1, 3, 4, 4, 3] ;
            [A_ineq, B_ineq] = create_inequalities(k_1, k_2, lie_B_1, lie_B_2, M, Lf3P, Lf3S, x5, x4, saturator_v, saturator_d);
            
            A = [A; A_ineq];
            B = [B; B_ineq];
        end
    end
    if sum(safe_mode) > 0
        eps = max(max( epsilons(safe_mode == 1, :) ));
        [c1, c2, div] = pick_qp_params(eps, r);
        Q = diag([1/div 1/div 1 1]);
        uc = [v1_fbl_new v2_fbl_new]'/div;
        F_obj = [-uc(2) -uc(1) c1 c2];
        
        u = quadprog(Q, F_obj, A, B, [], [], [], [],[], options);
        v1_fbl_new = u(2);
        v2_fbl_new = u(1);
    end
%     end
    v1_fbl_new = v1_fbl_new - Lf3P;
    v2_fbl_new = v2_fbl_new - Lf3S;
        
    u = M * [v1_fbl_new; v2_fbl_new];
    u1 = u(1); % V heading
    u2 = u(2); % acceleration
    %%
    

    %%%%%%%%% Taking the derivative to get the real input
    %%% Controller dynamics %%%
    x6 = x6_Old + u2*dt;
    x5 = x5_Old + dt*x6;
    x5 = min( max(x5, 0), saturator_v);
    x6 = (x5 - x5_Old) / dt;
    
    x4 = x4_Old + u1*dt;
    x4 = min( max(x4, -saturator_d), saturator_d);
    v_input = x5 + v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    saturator1 = pi/3;
    v_input = min(v_input, saturator_v);
    v_input = max(v_input, 0);
    % The system (Bicycle) 
    int_step = 100;
    x1 = x1_Old; x2 = x2_Old; x3 = x3_Old;
    for j = 1:int_step % Euler integration with smaller step
        x1 = x1 + dt*( v_input * cos(x3) ) / int_step;
        x2 = x2 + dt*( v_input * sin(x3) ) / int_step;
        x3 = x3 + dt*( (v_input/L) * tan(x4) ) / int_step;
    end
    % % % %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %      
    %      x5 = x5_Old + x6*dt; %%% Fictitius state
    %      x6 = x6_Old + u2*dt; %%% Fictitius state

    % data Store
    u1_plot(i) = u(1);
    u2_plot(i) = u(2);
    
    %%% Making the state vector, that might be helpful for the debugging
    %%% purposes
    x = [x1;x2;x3;x4;x5;x6];
   
    x_Old = x; %%% At the end of the an iteration the current values become the old values
    x1_Old = x_Old(1);
    x2_Old = x_Old(2);
    x3_Old = x_Old(3);
    x4_Old = x_Old(4);
    x5_Old = x_Old(5);
    x6_Old = x_Old(6);
    
    v_input_Old = v_input;
    
    %%% Saving the current values of the state to the full state matrix,
    %%% that might be useful for plotting purposes
    x_all(:,i+1) = x; 
    %%% Saving variables for plotting purposes
    x1_plot(i) = x1;
    x2_plot(i) = x2;
    x3_plot(i) = x3;
    x4_plot(i) = x4;
    x5_plot(i) = x5;
    x6_plot(i) = x6;
    
    
    xi1_plot(i) = xi1;
    xi2_plot(i) = xi2;
    eta1_plot(i) = eta1;
    eta2_plot(i) = eta2;
    input1(i) = u(1);
    input2(i) = u(2);
    q_store(i, :) = sum(safe_mode)./(sum(safe_mode)+1e-10);
    q_store(i, :) = round(q_store(i, :));


    %%% collision checking
    % for j = 1:length(eta0)
    %%% stop if there's a collision
        

    
end


%%


lm = 0:0.01:4*pi;
ang = [0:0.01:2*pi];

% v = VideoWriter('newfile.avi');
% open(v)

figure();
plot(r * cos(lm), r * sin(lm), '--r', 'color', 'green', 'linewidth', 4);
hold on
for i_plot = 1:length(eta0)
    eps = max(epsilons(i_plot, :));
    plot(xs_obs(i_plot, 1) + eps*cos(ang), xs_obs(i_plot, 2) + eps*sin(ang), 'LineWidth',8)
end

j_plot = 1;
j_new = length(x1_plot);
q = q_store(1);
while j_plot < length(x1_plot)
    for i = j_plot:length(x1_plot)
        if q_store(i) ~= q
            j_new = i;
            q_new = q_store(i);
            break
        end
        j_new = i;
        q_new = q_store(i);
    end
    if q == 0
        plot(x1_plot(j_plot:j_new), x2_plot(j_plot:j_new), 'k', 'linewidth', 2);
    elseif q == 1
        plot(x1_plot(j_plot:j_new), x2_plot(j_plot:j_new), 'r', 'linewidth', 2);
    elseif q == 2
        plot(x1_plot(j_plot:j_new), x2_plot(j_plot:j_new), 'g', 'linewidth', 2);
    elseif q == 3
        plot(x1_plot(j_plot:j_new), x2_plot(j_plot:j_new), 'y', 'linewidth', 2);
    end
    j_plot = j_new;
    q = q_new;
end
xlabel('$x_{1}(m)$', 'FontSize', 16, 'Interpreter', 'latex')
ylabel('$y_{1}(m)$', 'FontSize', 16, 'Interpreter', 'latex')
grid on;






function [c1, c2, div] = pick_qp_params(radius_obs, radius)
    c1 = 90  + 400 * radius_obs ; 
    c2 = 40  +  50  * radius_obs ; 
    div = max(0.5, 1 - radius*2)*0+1;
    
end