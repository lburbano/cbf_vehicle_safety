function [dist, closest] = find_closest_obs_polar(x_veh, xs_obs, eta, radius)
    radius = max(radius(:, 1), [], 2);
    angle_obs = eta;
    angle_vehicle = atan2(x_veh(2), x_veh(1));
    score = abs( wrapToPi(angle_obs - angle_vehicle) );
    
    n_obs = length(eta);
    score2 = zeros(n_obs, 1);
    for i=1:n_obs
        sgn = sign(xs_obs(i, 1) * xs_obs(i, 2));
        ang_max_obs = atan2(xs_obs(i, 2) + sqrt(2) * radius(i), xs_obs(i, 1) - sgn * sqrt(2) * radius(i));
        score2(i) = abs(eta0 - ang_max_obs) * 1.1;
    end
    score = score - score2;
    [dist, closest] = min(score);
end