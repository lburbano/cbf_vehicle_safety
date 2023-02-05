function [dist, closest] = find_closest_obs(x_veh, angle_vehicle, xs_obs, radius, angle_th)
    % Overapproximates ellipsoid to a circle
    % Easier to implement
    % Otherwise, implement the method distance to ellipsoid
    radius = max(radius(:, 1), [], 2);
    % Find obstancles in front vehicles
    angles_obs = atan2(xs_obs(:,1), xs_obs(:,2));
    diff_angle = angles_obs - angle_vehicle;
    in_front = abs(diff_angle) < angle_th;

    xs_obs = xs_obs(:, :);
    n_obs = size(xs_obs);
    n_obs = n_obs(1);
    n_obs = max(n_obs, 1);
    % Find the closest obs in front
    pos_vehicle = ones(n_obs, 1) .* x_veh;
    dist = pos_vehicle - xs_obs;
    dist = vecnorm(dist, 2, 2);
    dist = dist - radius;
    [dist, closest] = min(dist);
end