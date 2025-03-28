function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    % public_vars = init_particle_filter(read_only_vars, public_vars);
    
    public_vars.max_vel = 0.2;              % max starting speed
    public_vars.path_index = 1;             % current path index
    public_vars.weights = [];               % init W for PF
    public_vars.planning_required = 1;      % flag for path planner

    public_vars.state = 'START';        % init state of state machine
    public_vars.help_counter = 1;


end
% 
% % check if is robot going from inside to outside
% if (is_outside && isnan(read_only_vars.gnss_position))
%     public_vars.is_outside = 1;
%     public_vars.going_outside = 1;
% end
% 
% % check if is robot going from outside to inside
% if (~is_outside && ~isnan(read_only_vars.gnss_position))
%     public_vars.is_outside = 0;
%     public_vars.going_inside = 1;
% end
% 
% % if is robot starting inside, start pf init
% if (~public_vars.is_outside && ~public_vars.going_inside)
% 
% end
% 
% % if is robot starting outside, start kalman init
% if (public_vars.is_outside && ~public_vars.going_outside)
% 
% end

% 
% switch public_vars.state
%     case 'START'
% 
%         if (~isnan(read_only_vars.gnss_position))
%             public_vars.state = 'KF_INIT';
%             public_vars.help_counter = 1;
%         end
% 
% 
%     case 'KF_INIT'
%         % wait few iterations before kalman init
%         public_vars.help_counter = public_vars.help_counter +1;
% 
%         if (public_vars.help_counter >= 70)
%             public_vars.kf_enabled = 1;
%             public_vars = init_kalman_filter(read_only_vars, public_vars);
%             [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
%             public_vars.estimated_pose = estimate_pose(public_vars); 
% 
%             [public_vars] = plan_path(read_only_vars, public_vars);
%             public_vars.state = 'KF';
%         end
% 
%     case 'KF'
%         % speed up
%         public_vars.help_counter = public_vars.help_counter +1;
%         if (public_vars.help_counter >= 100)
%             public_vars.max_vel = 0.8;
%         end
% 
%         [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
%         public_vars.estimated_pose = estimate_pose(public_vars); 
%         public_vars = plan_motion(read_only_vars, public_vars);
% 
%         % du dovnnitr
% 
%     case 'PF_INIT'
%         disp('Stav: PROCESS');
%         % Podmínka pro přechod do dalšího stavu
%         public_vars.state = 'END';
% 
%     case 'PF'
%         disp('Stav: PROCESS');
%         % Podmínka pro přechod do dalšího stavu
%         public_vars.state = 'END';
% 
%     case 'END'
%         disp('Stav: END');
% 
% 
%     otherwise
%         disp('HELP!');
% 
% end
% 


% 
% 
% public_vars.particles = update_particle_filter(read_only_vars, public_vars);


% wait few iterations before kalman init
if (read_only_vars.counter == 70)
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

if (read_only_vars.counter >= 71)
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

    [public_vars] = plan_path(read_only_vars, public_vars);
    % 13. Plan next motion command
    public_vars = plan_motion(read_only_vars, public_vars);
end

% speed up
if (read_only_vars.counter == 100)
    public_vars.max_vel = 0.8;
end


% 12. Path planning
% [public_vars] = plan_path(read_only_vars, public_vars);
% 
% public_vars = plan_motion(read_only_vars, public_vars); % TODO


end

