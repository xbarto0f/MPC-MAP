function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.0007, 0.0007, 0.0007]);
public_vars.kf.Q = [0.2502,0.0080; 0.0080,0.2469];

% public_vars.mu = [2,2,pi/2];
public_vars.mu = [mean(read_only_vars.gnss_history), 0];
% public_vars.sigma = zeros(3,3);
public_vars.sigma = ones(3)*20;
public_vars.sigma(1:2,1:2) = cov(read_only_vars.gnss_history);
end

