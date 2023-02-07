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
v        = 0.00001; %%% Not the linear velocity input but the fixed speed
k1       = 70;    %
k2       = 60;
k3       = 5;    %
k4       = 40;
k5       = 26;
kc       = [k1, k2, k3, k4, k5];
L        = 0.33;
r        = 2;     %% radius of the circle to follow

%% Obstacle avoidance parameters
epsilon1    = 0.25;
eta0        = [pi/2];
% eta0        = [0];
epsilon2    = epsilon1;
xi0         = 0;
n_obstacles = length(eta0);
xs_obs = [];
epsilons = [];
for i = 1 :length(eta0)
    xs_obs = [xs_obs; r*[sin(eta0(i)), cos(eta0(i))] ];
    epsilons = [epsilons; [epsilon1, epsilon2]];
end
phi_0 = pi/2; % threshold angle

%%
%%% Initial Conditions
X1_0 = -r; %% x-position
X2_0 = 0;       %% y-position
X3_0 = -pi/2;    %% car orientation
X4_0 = 0;       %% Wheel angle
X5_0 = 0;       %% Fictitious state and we can always initialize it with zero
X6_0 = 0;       %% Fictitious state and we can always initialize it with zero

v_input = 0;

% Initial condition Vector
x0 = [X1_0; X2_0; X3_0; X4_0; X5_0; X6_0];


% Simulation time
Tmax = 60;  % End point
dt = 1/15; % Time step
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


safe_mode = 0;
for i=1:length(T)-1
    %% Transformed states
    
    % Measure state   
    x = [x1, x2, x3, x4, x5, x6];

    % Nominal control to follow the circle
    % circle_control
    [v_tang, v_tran, v1_fbl_new, v2_fbl_new, M, xi, eta, Lf3P, Lf3S] = nominal_control(x, kc, L, r, v);
    eta1 = eta(1); eta2 = eta(2); eta3 = eta(3);
    xi1 = xi(1); xi2 = xi(2); xi3 = xi(3);
    
    avoid_obstacle = 1;
    x_veh = [x1, x2];
    [dist, closest_obs] = find_closest_obs(x_veh, x3, xs_obs, epsilons, phi_0);
    x0 = xs_obs(closest_obs, :);
    dist_center = norm(x_veh - x0);

    k_switch = 3;
    dist_switch = k_switch * max(epsilons(closest_obs, :) ) + 0.7;
    if safe_mode == 1 && dist_center > dist_switch
        safe_mode = 0;
    end
    if safe_mode == 0 && dist_center <= dist_switch - 0.1
        safe_mode = 1;
    end
    for j = 1:length(eta0)
        x_obs_cbf = xs_obs(j, :);
        Bar_F(i, j) = epsilon2 - sqrt( ( x1 - x_obs_cbf (1) )^2 + ( x2 - x_obs_cbf(2) )^2 );
    end
    if safe_mode == 1 && avoid_obstacle
        fprintf("In safe mode, obs %i dist: %.1d, dist center: %.1d\n", closest_obs, dist, dist_center)
        eta_obstacle = eta0(closest_obs);
        
        [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1] = lie_derivatives_B_1(eta, epsilons(closest_obs, 1), eta_obstacle);
        [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2] = lie_derivatives_B_2(xi,  epsilons(closest_obs, 2), xi0);
        lie_B_1 = [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1]';
        lie_B_2 = [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2]';


        k_1 = [1, 2, 3, 3, 2] ;
        k_2 = [1, 2, 3, 3, 2] ;
        [A_ineq, B_ineq] = create_inequalities(k_1, k_2, lie_B_1, lie_B_2, M);
        
        [c1, c2, div] = pick_qp_params(max(epsilons(closest_obs, :)));
        Q = [1/div 0 0 0; 0 1/div 0 0; 0 0 1 0; 0 0 0 1];
        F_obj = [-v2_fbl_new -v1_fbl_new c1 c2];
%         F_obj = [-v1_fbl_new -v2_fbl_new c1 c2];

        u = quadprog(Q, F_obj, A_ineq, B_ineq, [], [], [], [],[], options);

        v1_fbl_new = u(2);
        v2_fbl_new = u(1);
%         v1_fbl_new = u(1);
%         v2_fbl_new = u(2);
        v1_fbl_new = v1_fbl_new - Lf3P;
        v2_fbl_new = v2_fbl_new - Lf3S;
    end

    u = M * [v1_fbl_new; v2_fbl_new];
    u1 = u(1);
    u2 = u(2);
    %%
    

    %%%%%%%%% Taking the derivative to get the real input
    %%% Controller dynamics %%%
    saturator_v = 2;
    saturator_d = pi/3;
    x6 = x6_Old + u2*dt;
    x5 = x5_Old + dt*x6;
    
%     x5 = min( max(x5, 0), saturator_v);
    x4 = x4_Old + u1*dt;
    x4 = min( max(x4, -saturator_d), saturator_d);
    v_input = x5 + v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    saturator1 = pi/3;
    u1 = min(u1, saturator1);
    u1 = max(u1, -saturator1);
    v_input = min(v_input, saturator_v);
    v_input = max(v_input, -0);
    u = [u1; v_input];
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
    
    
    xi1_plot(i) = xi1;
    xi2_plot(i) = xi2;
    eta1_plot(i) = eta1;
    eta2_plot(i) = eta2;
    input1(i) = u(1);
    input2(i) = u(2);
    
    
end


%%

length(find(Bar_F>0))

lambda = 0:0.01:4*pi;
ang = [0:0.01:2*pi];

% v = VideoWriter('newfile.avi');
% open(v)

figure();
plot(r * cos(lambda), r * sin(lambda), '--r', 'color', 'green', 'linewidth', 4);
hold on
plot(x1_plot, x2_plot, 'r', 'color', 'red', 'linewidth', 2);
xlabel('$x_{1}(m)$','FontSize',16,'Interpreter','latex')
ylabel('$y_{1}(m)$','FontSize',16,'Interpreter','latex')
grid on;

for i = 1:length(eta0)
    plot(xs_obs(i, 1) + epsilon2*cos(ang), xs_obs(i, 2) + epsilon2*sin(ang), 'LineWidth',8)
end
k = 1;
% M = [];



function [c1, c2, div] = pick_qp_params(radius)
    c1 = 110 + 250 * radius;
    c2 = 40  +  40 * radius;
    div = max(1 - radius/3, 0.6);
    
end