% function [public_vars] = plan_motion(read_only_vars, public_vars)
% %PLAN_MOTION Summary of this function goes here
% 
% l_dist = read_only_vars.lidar_distances;
% 
% 
% 
% r_pose = public_vars.estimated_pose;
% 
% d = read_only_vars.agent_drive.interwheel_dist;
% theta = r_pose(3);
% 
% epsilon = 0.2;
% kappa = 1/(10*epsilon); 
% 
% P = r_pose(1:2)+epsilon*[cos(theta), sin(theta)];
% 
% [G, vG, public_vars] = get_target(P, public_vars);
% 
% vP = kappa * (G - P) + vG;
% 
% if norm(vP) > public_vars.max_vel%read_only_vars.agent_drive.max_vel
%     vP = vP/norm(vP)*public_vars.max_vel; %read_only_vars.agent_drive.max_vel;
% end
% 
% v =  vP(1)*cos(theta) + vP(2)*sin(theta);
% omega = (-vP(1)*sin(theta)+vP(2)*cos(theta))/epsilon;
% 
% r = v + (omega * d)/2; 
% l = v - (omega * d)/2;
% 
% public_vars.motion_vector = [r, l];
% 
% end


function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

    l_dist = read_only_vars.lidar_distances;
    r_pose = public_vars.estimated_pose;
    
    d = read_only_vars.agent_drive.interwheel_dist;
    theta = r_pose(3);

    epsilon = 0.2;
    kappa = 1/(10*epsilon); 

    P = r_pose(1:2) + epsilon * [cos(theta), sin(theta)];

    % Anticolision
    collision_thresh = 0.4; 
    min_dist = min(l_dist);
    % reverse_mode = false; 

    if (min_dist < collision_thresh)
        % direction
        [~, min_idx] = min(l_dist);
        
        if min_idx == 1 || min_idx == 2 || min_idx == 8
            % Překážka vpředu -> couvání
            vP = -0.5 * [cos(theta), sin(theta)];
            omega = pi/2; % Mírná otočka dozadu
            % reverse_mode = true;
        elseif  min_idx == 5
            % Překážka vzadu -> dopředu
            vP = 0.5 * [cos(theta), sin(theta)];
            omega = -pi/2; % Mírná otočka dopředu
        elseif min_idx == 3 || min_idx == 4 
            % Překážka vlevo -> otočka doprava
            omega = -pi/4;
            vP = [0, 0];
        elseif min_idx == 6 || min_idx == 7
            % Překážka vpravo -> otočka doleva
            omega = pi/4;
            vP = [0, 0];
        else
            % Překážky ze všech stran -> couvání
            vP = -0.5 * [cos(theta), sin(theta)];
            omega = pi/2; 
            % reverse_mode = true;
        end
    else
         if isempty(public_vars.path)
            vP = 0.5 * [cos(theta), sin(theta)];
            omega = 0;
         else
            
            [G, vG, public_vars] = get_target(P, public_vars);
            vP = kappa * (G - P) + vG;

            % disp(vP)
            % disp(size(vP,2)) 
            % disp(theta)
            
            vP = vP(:)'; 
            
            if isempty(vP) || size(vP, 2) < 2
                omega = 0
            else
                omega = (-vP(1) * sin(theta) + vP(2) * cos(theta)) / epsilon;
            end
         end
    end

    
    if norm(vP) > public_vars.max_vel
        vP = vP / norm(vP) * public_vars.max_vel;
    end
    
    if isempty(vP) || size(vP, 2) < 2
        v = 0
    else
        v = vP(1) * cos(theta) + vP(2) * sin(theta);
    end
   
    % if reverse_mode
    %     v = -0.3 * public_vars.max_vel;
    % end

    r = v + (omega * d) / 2; 
    l = v - (omega * d) / 2;

    public_vars.motion_vector = [r, l];

end