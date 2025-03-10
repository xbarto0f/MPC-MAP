function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));
for i=1:8
    inters = ray_cast(pose(1:2),map.walls, pose(3)+lidar_config(i));
    dists = sqrt((inters(:,1) - pose(1)).^2 + (inters(:,2) - pose(2)).^2);
    measurement(i) = min(dists);
end
end

