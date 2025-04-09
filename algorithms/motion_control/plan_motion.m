function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

    l_dist = read_only_vars.lidar_distances;
    r_pose = public_vars.estimated_pose;
    
    d = read_only_vars.agent_drive.interwheel_dist;
    theta = r_pose(3);

    epsilon = 0.2;
    kappa = 1/(10*epsilon); 

    P = r_pose(1:2) + epsilon * [cos(theta), sin(theta)];

    collision_thresh = 0.4; 
    min_dist = min(l_dist);
    if (min_dist < collision_thresh)
        % anticollision
        [~, min_idx] = min(l_dist);
        
        if min_idx == 1 || min_idx == 2 || min_idx == 8
            % Obstacle ahead
            vP = -0.5 * [cos(theta), sin(theta)];
            omega = pi/2;
        elseif  min_idx == 5
            % behind
            vP = 0.5 * [cos(theta), sin(theta)];
            omega = -pi/2; 
        elseif min_idx == 3 || min_idx == 4 
            % Obstacle on the left
            omega = -pi/4;
            vP = [0, 0];
        elseif min_idx == 6 || min_idx == 7
            % Obstacle on the tright
            omega = pi/4;
            vP = [0, 0];
        else
            vP = -0.5 * [cos(theta), sin(theta)];
            omega = pi/2; 
        end
    elseif (public_vars.ignore_motion_plan)
        % ignoring motion planning
        public_vars.motion_vector = [0.2,0.2];
        return;
    else
         % normal motion planning
         if isempty(public_vars.path)
            vP = 0.5 * [cos(theta), sin(theta)];
            omega = 0;
         else
            
            [G, vG, public_vars] = get_target(P, public_vars);
            vP = kappa * (G - P) + vG;
            vP = vP(:)'; 
            
            if isempty(vP) || size(vP, 2) < 2
                omega = 0;
            else
                omega = (-vP(1) * sin(theta) + vP(2) * cos(theta)) / epsilon;
            end
         end
    end

    
    if norm(vP) > public_vars.max_vel
        vP = vP / norm(vP) * public_vars.max_vel;
    end
    
    if isempty(vP) || size(vP, 2) < 2
        v = 0;
    else
        v = vP(1) * cos(theta) + vP(2) * sin(theta);
    end

    r = v + (omega * d) / 2; 
    l = v - (omega * d) / 2;
    
    public_vars.motion_vector = [r, l];

end