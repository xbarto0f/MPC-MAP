function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    % public_vars = init_particle_filter(read_only_vars, public_vars);
    
    public_vars.max_vel = 0.2;
    %% WEEK 2
    % public_vars.lidar_data = zeros(1000,8);
    % public_vars.gnss_data = zeros(1000,2);

    %% WEEK 3 - path for indoor_1 map
    % path = [2, 8.5;  
    %     3.5, 7.8;
    %     5, 7;
    %     5.2, 5.5;
    %     5, 4;
    %     5, 2;
    %     6, 1.8;
    %     7, 1.5;
    %     7.8, 1.7;
    %     8.5, 2;
    %     8.8, 5.5;
    %     9, 9];
    % 
    % x = path(:,1);
    % y = path(:,2);
    % t = linspace(0,1,length(x));
    % t_fine = linspace(0,1,100); 
    % x_smooth = spline(t, x, t_fine);
    % y_smooth = spline(t, y, t_fine);
    % public_vars.path  = [x_smooth(:), y_smooth(:)];
    %%

    public_vars.path_index = 1;
    
    %% WEEK 4
    
    public_vars.weights = [];

    %% WEEK 5 - path for outdoor_1 map
    
    path = [2, 2;  
        8, 8;
        12, 7.5;
        16,2];
    public_vars.path = path;

    x = path(:,1);
    y = path(:,2);
    t = linspace(0,1,length(x));
    t_fine = linspace(0,1,10); 
    x_smooth = spline(t, x, t_fine);
    y_smooth = spline(t, y, t_fine);
    public_vars.path  = [x_smooth(:), y_smooth(:)];
   

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);
if (read_only_vars.counter == 70)
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

if (read_only_vars.counter >= 71)
    % 10. Update Kalman filter
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    
    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)


    % 13. Plan next motion command
    public_vars = plan_motion(read_only_vars, public_vars);

end
% speed up
if (read_only_vars.counter == 100)
    public_vars.max_vel = 0.8;
end

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 
% if (read_only_vars.counter < 1001)
%     public_vars.lidar_data(read_only_vars.counter,:) = read_only_vars.lidar_distances;
%     public_vars.gnss_data(read_only_vars.counter,:) = read_only_vars.gnss_position;
% elseif (read_only_vars.counter == 1001)
%     figure(2)
%     histogram(public_vars.lidar_data(:,1))
%     figure(3)
%     histogram(public_vars.gnss_data(:,1))
% end

end

