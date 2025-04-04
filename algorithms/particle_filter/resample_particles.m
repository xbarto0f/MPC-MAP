function [new_particles] = resample_particles(particles, weights, map_limits, pf_no_rnd)
% 
x_min = min([map_limits(1),map_limits(3),map_limits(5),map_limits(7)]);%-0.2;
x_max = max([map_limits(1),map_limits(3),map_limits(5),map_limits(7)]);%+0.2;
y_min = min([map_limits(2),map_limits(4),map_limits(6),map_limits(8)]);%-0.2;
y_max = max([map_limits(2),map_limits(4),map_limits(6),map_limits(8)]);%+0.2;

% N = size(particles, 1);
% new_particles = zeros(N, 3); %particles; 
% 
% % weights = weights / sum(weights);  
% 
% index = randi([1, N]);
% beta = 0;
% mw = max(weights);
% 
% c = 0;
% for i = 1:N
%     beta = beta + 2 * mw * rand;
%     while beta > weights(index)
%         beta = beta - weights(index);
%         index = index + 1;
%         if index > N
%             index = 1;
%         end
%     end
%     x = particles(index,1);
%     y = particles(index,2);
%     is_inside = (x >= x_min) && (x <= x_max) && (y >= y_min) && (y <= y_max);
% 
%     if is_inside
%         new_particles(i, :) = particles(index, :);
%         c = c+ 1;
%     else
%         new_particles(i,1) = x_min + (x_max - x_min) * rand();
%         new_particles(i,2) = y_min + (y_max - y_min) * rand();
%         new_particles(i,3) = -pi + (pi+pi).*rand(1,1);
%     end
% end
% 
%     % make few particles random
%     % if  (~pf_no_rnd)
%         n=fix(N/5);
%         rnd_indexs = randi([1,N],n,1);
% 
%         for j=1:n
%             rnd_index =rnd_indexs(j);
%             new_particles(i,1) = x_min + (x_max - x_min) * rand();
%             new_particles(i,2) = y_min + (y_max - y_min) * rand();
% 
%             new_particles(rnd_index,3) = -pi + (pi+pi).*rand(1,1);
%         end
%     % end
% end

    N_p = size(particles, 1);
    N_random =50;

    N = N_p -N_random;
    new_particles = zeros(N_p, 3); %particles; 
       
    %   index = 1;
    % r = rand()/N;
    % c = weights(1);
    % 
    % for i = 1:N
    %     u = r + (i-1)/N;
    %     while c < u
    %         c = c + weights(index);
    %         index = index + 1;
    %         if index > length(weights)
    %             index = length(weights);
    %         end 
    %     end
    % 
    %     new_particles(i, :) = particles(index,:);
    % end


    index = 1;
    u = rand()/N;
    c = weights(1);
    
    for i = 1:N
        while c < u
            c = c + weights(index);
            index = index + 1;
            if index > length(weights)
                index = length(weights);
            end 
        end
        u = u + 1/N;
        new_particles(i, :) = particles(index,:);
    end

    % make few particles random
    new_particles(N+1:end,1) = x_min + ((x_max - x_min) * rand());
    new_particles(N+1:end,2) = y_min + ((y_max - y_min) * rand());
    new_particles(N+1:end,3) = rand()*2*pi;
    end

