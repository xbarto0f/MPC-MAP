function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

new_particles = particles;

public_vars.weights = weights;

N = size(particles,1);
index = randi([1,N-1]);
for i = 1:N
    beta = 2*max(weights)*rand(1,1);
    while weights(index) < beta
        beta = beta - weights(index);
        index = index+1;
        if index > N
            index = 1;
        end
    end
    new_particles(i) = particles(index);
end
% new_particles = particles;

end

