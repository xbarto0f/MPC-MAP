function [new_particles] = resample_particles(particles, weights, map_limits)

N = size(particles, 1);
new_particles = particles; 

weights = weights / sum(weights);  

index = randi([1, N]);
beta = 0;
mw = max(weights);

c = 0;
for i = 1:N
    beta = beta + 2 * mw * rand;
    while beta > weights(index)
        beta = beta - weights(index);
        index = index + 1;
        if index > N
            index = 1;
        end
    end
    x = particles(index,1);
    y = particles(index,2);
    is_inside = (x >= map_limits(1)) && (x <= map_limits(3)) && (y >= map_limits(2)) && (y <= map_limits(4));

    if is_inside
        new_particles(i, :) = particles(index, :);
        c = c+ 1;
    else
        new_particles(i,1:2) = 10.*rand(1,2);
        new_particles(i,3) = -pi + (pi+pi).*rand(1,1);
    end
 
    
end

% make few particles random
n=fix(N/40);
    rnd_indexs = randi([1,N],n,1);

    for j=1:n
        rnd_index =rnd_indexs(j);
        new_particles(rnd_index,1:2) = 10.*rand(1,2);
        new_particles(rnd_index,3) = -pi + (pi+pi).*rand(1,1);
    end
end


