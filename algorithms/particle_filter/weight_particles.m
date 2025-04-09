function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

    N = size(particle_measurements, 1);
    weights = ones(N,1) / N;
    lidar_distances(isnan(lidar_distances)) = 20;
    lidar_distances(isinf(lidar_distances)) = 20;

    particle_measurements(isnan(particle_measurements)) = 20;
    particle_measurements(isinf(particle_measurements)) = 20;
    for i = 1:N

        weights(i) = (1/sqrt(sum((particle_measurements(i,:)-lidar_distances).^2)));
    end
    weights = weights / sum(weights);
end