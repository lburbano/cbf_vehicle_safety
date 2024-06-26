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
epsilon1    = 0.2;
eta0        = -pi/4;
eta0        = [-pi/4 -pi/4 -pi/4];
% [0 -pi];
% [0 -pi/4 -pi/8]
% [-pi/2 0 -pi]
% [-pi/4]
epsilon2    = epsilon1;
xi0         = 0;
n_obstacles = length(eta0);
xs_obs = [];
epsilons = [];
for i = 1 :length(eta0)
    xs_obs = [xs_obs; r*(1+i/10)*[sin(eta0(i)), cos(eta0(i))] ];
    epsilons = [epsilons; [epsilon1, epsilon2]];
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
dt = 1/25; % Time step
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
safe_mode = 0;
mu = 1.1;
lambda = 0;
epsilon_switch = 0.1;
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
    k_switch = 2;
    
%     tau = 2*pi*r/10 + 0.5;
    tau = k_switch * max(epsilons(:)) + 0.5;
    
    x0 = xs_obs(closest_obs, :);
    Bar_F(i, :) = dist;
    if safe_mode > 0 && min(Bar_F(i, :)) > tau % 2*pi*r/10 %norm([x1, x2] - x0) - epsilons(closest_obs) > k_switch*epsilons(closest_obs)
        % here there is a jump
        safe_mode = 0;
    elseif min(Bar_F(i, :)) <= tau - epsilon_switch % 0*2*pi*r/10-0.1 % avoid zeno behavior between nominal control and obstacle avoidance
        if safe_mode == 0
            [min_B, new_B] = min(Bar_F(i, :));
            safe_mode = new_B;
            current_obs = safe_mode;
        else
            current_B = Bar_F(i, current_obs);
            safe_mode = current_obs;
            Bar_F_store = Bar_F(i, :);
%             Bar_F_store(current_obs) = inf;
            [min_B, new_B] = min(Bar_F_store);
            if current_B >= (mu) * min_B % avoid zeno behavior in the switching among obstacles
                % here there is a jump
                safe_mode = new_B;
                current_obs = safe_mode;
            else
                % There is no jump!
            end
        end
    else
        % There is no jump!
    end
    avoid_obstacle = 1;
    if safe_mode > 0 && avoid_obstacle
        eta_obstacle = eta0(current_obs);
        xi_obstacle = sum(xs_obs(current_obs, :).^2) - r^2;
%         epsilon = epsilons(Bar_F(i, :) <= 2*pi*r/10, :);
%         epsilon = max(epsilon(:));
        epsilon = max(epsilons(current_obs, :));
        [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1] = lie_derivatives_B_1(eta, epsilon, eta_obstacle);
        [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2] = lie_derivatives_B_2(xi, epsilon, xi_obstacle);
        lie_B_1 = [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1]';
        lie_B_2 = [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2]';

        k_1 = [1, 3, 4, 4, 3] ;
        k_2 = [1, 3, 4, 4, 3] ;
        [A_ineq, B_ineq] = create_inequalities(k_1, k_2, lie_B_1, lie_B_2, M, Lf3P, Lf3S, x5, x4, saturator_v, saturator_d);
        [c1, c2, div] = pick_qp_params(epsilons(current_obs), r);
%         div = 1/min_B;
        Q = diag([1/div 1/div 1 1]);
        uc = [v1_fbl_new v2_fbl_new]'/div;
        F_obj = [-uc(2) -uc(1) c1 c2];

        u = quadprog(Q, F_obj, A_ineq, B_ineq, [], [], [], [],[], options);
        
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
    x5 = min( max(x5, -v), saturator_v);
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
    q_store(i) = safe_mode;
    
end


%%


lm = 0:0.01:4*pi;
ang = [0:0.01:2*pi];

% v = VideoWriter('newfile.avi');
% open(v)

figure();
plot(r * cos(lm), r * sin(lm), '--r', 'color', 'green', 'linewidth', 4);
hold on
for i = 1:length(eta0)
    eps = max(epsilons(i, :));
    plot(xs_obs(i, 1) + eps*cos(ang), xs_obs(i, 2) + eps*sin(ang), 'LineWidth',8)
end

j = 1;
j_new = length(x1_plot);
q = q_store(1);
while j < length(x1_plot)
    for i = j:length(x1_plot)
        if q_store(i) ~= q
            j_new = i;
            q_new = q_store(i);
            break
        end
        j_new = i;
        q_new = q_store(i);
    end
    if q == 0
        plot(x1_plot(j:j_new), x2_plot(j:j_new), 'k', 'linewidth', 2);
    elseif q == 1
        plot(x1_plot(j:j_new), x2_plot(j:j_new), 'r', 'linewidth', 2);
    elseif q == 2
        plot(x1_plot(j:j_new), x2_plot(j:j_new), 'g', 'linewidth', 2);
    elseif q == 3
        plot(x1_plot(j:j_new), x2_plot(j:j_new), 'y', 'linewidth', 2);
    end
    j = j_new;
    q = q_new;
end
xlabel('$x_{1}(m)$', 'FontSize', 16, 'Interpreter', 'latex')
ylabel('$y_{1}(m)$', 'FontSize', 16, 'Interpreter', 'latex')
grid on;






function [c1, c2, div] = pick_qp_params(radius_obs, radius)
    c1 = 60  + 300 * radius_obs + min(50 * radius, 200); 
    c2 = 40  +  50  * radius_obs + 50 * radius; 
    div = max(0.5, 1 - radius*2)*0+4;
    
end