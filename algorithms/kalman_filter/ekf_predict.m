function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

x = mu(1);
y = mu(2);
theta = mu(3);

v = u(1);
omega = u(2);

dt = sampling_period;
R = kf.R;


new_mu = [x+cos(theta)*v*dt, y+sin(theta)*v*dt, theta+omega*dt];
G = [1, 0, -sin(theta)*v*dt; 0, 1, cos(theta)*v*dt; 0, 0, 1];
new_sigma = G*sigma*G' + R;

% new_mu = mu;
% new_sigma = sigma;

end

