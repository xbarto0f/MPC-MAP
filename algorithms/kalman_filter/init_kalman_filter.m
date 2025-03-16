function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [0.001,0.001,0.001; 0.001,0.001,0.001];
public_vars.kf.R = [0.001,0.001,0.001; 0.001,0.001, 0.001; 0.001, 0.001, 0.001];
public_vars.kf.Q = [0.2502,0.0080; 0.0080,0.2469];

public_vars.mu = [2,2,pi/2];
public_vars.sigma = zeros(3,3);

end

