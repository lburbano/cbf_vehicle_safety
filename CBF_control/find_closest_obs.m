function [dist, closest] = find_closest_obs(x_veh, angle_vehicle, xs_obs, radius, angle_th)
    phi_I = atan2(x_veh(2), x_veh(1));
    % Overapproximates ellipsoid to a circle
    % Easier to implement
    % Otherwise, implement the method distance to ellipsoid
    radius = max(radius(:, 1), [], 2);
    % Find obstancles in front vehicles
    angles_obs = atan2(xs_obs(:,2), xs_obs(:,1));
    diff_angle = wrapToPi(angles_obs - angle_vehicle);
%     not_in_front = abs(diff_angle) > angle_th;
    not_in_front = abs(diff_angle) > pi;

    % This is purely algorithmic
    % Makes the distance infinity if the robot is not heading the obstacle
    % It is not a mathematical thing but rather a way to implement
    xs_obs(not_in_front, :) = inf;
    n_obs = size(xs_obs);
    n_obs = n_obs(1);
    n_obs = max(n_obs, 1);
    % Find the closest obs in front
    pos_vehicle = ones(n_obs, 1) .* x_veh;
    dist = pos_vehicle - xs_obs;
    dist = vecnorm(dist, 2, 2);
    dist = dist - radius;
%     if sum(in_front) > 0
%         
%     else
%         dist = inf;
%     end
    [~, closest] = min(dist);
end