%% PathFollowing of Car-like Robot
clc
clearvars
close all
addpath('CBF_control')
addpath('camera_communication')
addpath('robot_communication')
addpath('nominal_control')
options = optimoptions('quadprog','Display','off');
%% Optitrack setting
dllPath = strcat(pwd, '\NatNet_SDK_4.0\NatNetSDK\lib\x64\NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
client_camera = NatNetML.NatNetClientML(0);
HostIP = '127.0.0.1';
client_camera.Initialize(HostIP, HostIP);
Drone_ID = 1;
%% Nominal controller Parameters
v        = 0.000001; %%% Not the linear velocity input but the fixed speed
k1       = 50;    %
k2       = 40;
k3       = 4;    %
k4       = 100;
k5       = 26;
kc       = [k1, k2, k3, k4, k5];
L        = 0.33;
r        = 2;     %% radius of the circle to follow

%% Obstacle avoidance parameters
epsilon1    = 0.25;
eta0        = [-pi/4-0.3 -5*pi/4-0.3];
epsilon2    = epsilon1;
xi0         = 0;
n_obstacles = length(eta0);
xs_obs = [];
epsilons = [];
for i = 1 :length(eta0)
    xs_obs = [xs_obs; r*[sin(eta0(i)), cos(eta0(i))] ];
    epsilons = [epsilons; [epsilon1, epsilon2]];
end
phi_I = pi/2; % threshold angle

%% Initialize communication
ip = "192.168.2.200";
message_size = 8;
port = 12345;
client_robot = clientCommunication(ip, port, message_size);

%%
% Simulation time
Tmax = 200;  % End point
dt   = 1/15; % Time step
T    = 0:dt:Tmax; % Time vector
time = T;

message_send = create_message(0, 0);
client_robot.send_data( message_send );
pause(5*dt)
robot_position = GetDronePosition(client_camera, Drone_ID);

% Initial condition
X4_0 = 0;       %% Wheel angle
X5_0 = 0;       %% Fictitious state and we can always initialize it with zero
X6_0 = 0;       %% Fictitious state and we can always initialize it with zero
v_input = 0;
x1 = robot_position(1);
x2 = robot_position(3);
x3 = robot_position(4);
x3 = x3 + pi;
x4 = X4_0;
x5 = X5_0;
x6 = X6_0;

x1_Old = x1;
x2_Old = x2;
x3_Old = x3;
x4_Old = X4_0;
x5_Old = X5_0;
x6_Old = X6_0;

v_input_Old = v_input;
x0 = [x1_Old, x2_Old, x3_Old, x4_Old, x5_Old, x6_Old]';
% Simulation setup
x_all = zeros(size(x0, 1),length(T));  % Intialization of a matrix to save all the values of the states
x_all(:, 1) = x0; % Saving the initial state in the full state matrix
u1_plot = zeros(1,length(T));
u2_plot = zeros(1,length(T));
safe_mode = 0;

%% Control loop
for i=1:length(T)-1
    % Measure state
    message_robot = client_robot.receive_data();
    robot_position = GetDronePosition(client_camera, Drone_ID);
    x1 = robot_position(1);
    x2 = robot_position(3);
    x3 = robot_position(4);
    x4 = message_robot/180*pi;
    
    x = [x1, x2, x3, x4, x5, x6];

    % Nominal control to follow the circle
    % circle_control
    [v_tang, v_tran, v1_fbl_new, v2_fbl_new, M, xi, eta, Lf3P, Lf3S] = nominal_control(x, kc, L, r, v);
    eta1 = eta(1); eta2 = eta(2); eta3 = eta(3);
    xi1 = xi(1); xi2 = xi(2); xi3 = xi(3);
    
    avoid_obstacle = 1;
    x_veh = [x1, x2];
%     [dist, closest_obs] = find_closest_obs(x_veh, x3, x_obs, epsilons, phi_I);
    [dist, closest_obs] = find_closest_obs(x_veh, xs_obs, eta0, epsilons);
    x0 = x_obs(closest_obs, :);
    if safe_mode == 1 && dist > 0.75*r 
        safe_mode = 0;
    end
    if safe_mode == 0 && avoid_obstacle && dist <= 0.75*r - 0.05*r
        safe_mode = 1;
    end
    if safe_mode == 1 
        eta_obstacle = eta0(closest_obs);
        
        [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1] = lie_derivatives_B_1(eta, epsilons(closest_obs, 1), eta_obstacle);
        [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2] = lie_derivatives_B_2(xi,  epsilons(closest_obs, 2), xi0);
        lie_B_1 = [B_1, LfB_1, L2fB_1, L3fB_1, LgLf2B_1]';
        lie_B_2 = [B_2, LfB_2, L2fB_2, L3fB_2, LgLf2B_2]';

        Bar_F(i) = epsilon2 - sqrt( (x1-x0(1))^2 + (x2-x0(2))^2 );

        k_1 = [1, 2, 3, 3, 2] ;
        k_2 = [1, 2, 3, 3, 2] ;
        [A_ineq, B_ineq] = create_inequalities(k_1, k_2, lie_B_1, lie_B_2, M);
        
        [c1, c2, div] = pick_qp_params(max(epsilons(closest_obs, :)));
        Q = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] / div;
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

    %% Control signal
    %%% Controller dynamics and saturation %%%
    x4 = x4_Old + u1*dt;
    x4 = min( max(x4, -pi/3), pi/3);
    x6 = x6_Old + u2*dt;
    x5 = x5_Old + dt*x6;
    v_input = x5 + v;
    saturator_velocity = 1;
    v_input = min(v_input, saturator_velocity);
    v_input = max(v_input, 0);
    u = [u1; v_input];

    %% Send control signal
    % send info to the system
    v_input_send = v_input ; 
    angle_send = x4 * 180/pi;
    message_send = create_message(v_input_send, angle_send);
    client_robot.send_data( message_send );
    pause(dt)
    %% Data store
    u1_plot(i) = u1;
    u2_plot(i) = u2;
    x = [x1;x2;x3;x4;x5;x6];
    x_Old = x; %%% At the end of the an iteration the current values become the old values
    x1_Old = x_Old(1);
    x2_Old = x_Old(2);
    x3_Old = x_Old(3);
    x4_Old = x_Old(4);
    x5_Old = x_Old(5);
    x6_Old = x_Old(6);

    v_input_Old = v_input;
    x_all(:,i+1) = x;
    %%% Saving variables for plotting purposes
    x1_plot(i) = x1;
    x2_plot(i) = x2;
    x3_plot(i) = x3;
    x4_plot(i) = x4;
    input1(i) = u(1);
    input2(i) = u(2);
    Bar_F(i) = epsilon1-sqrt((x1-x0(1))^2+(x2-x0(2))^2);
end
function strng = create_message(v, delta)
    strng = strcat(num2str(v), ";", num2str(delta) );
end
function [c1, c2, div] = pick_qp_params(radius)
    c1 = 120;
    c2 = 45;
    div = 1;
    if radius < 0.25 % parameters as a function of radius

    elseif radius < 0.35
        
    elseif radius < 0.45
        
    elseif radius < 0.55
        
    else
        
    end
end