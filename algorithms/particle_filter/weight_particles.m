function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);
weights = ones(N,1) / N;

for i = 1:N
    % weights(i) = sqrt(1/(sum((particle_measurements(i,:)-lidar_distances).^2)));
    weights(i) = (1/sqrt(sum((particle_measurements(i,:)-lidar_distances).^2)));
end
end

