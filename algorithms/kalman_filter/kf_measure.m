function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

C = kf.C;
Q = kf.Q;

K = sigma*C'/(C*sigma*C'+Q);

new_mu = mu + (K*(z'-C*mu'))';
new_sigma = (eye(3,3)-K*C)*sigma;

% new_mu = mu;
% new_sigma = sigma;

end

