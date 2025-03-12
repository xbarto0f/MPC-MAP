function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here
N = 300;
prtcls = zeros(N,3);
for i=1:N
    prtcls(i,1:2) = 10.*rand(1,2);
    prtcls(i,3) = -pi + (pi+pi).*rand(1,1);
end

public_vars.particles = prtcls;
% public_vars.particles = [2,8.5,0];
% public_vars.particles = [1,1,pi/2];

end

