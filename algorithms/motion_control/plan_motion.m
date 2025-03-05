function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

%% Week 2 Task 5
% l = 0.0;
% r = 0.0;

% if read_only_vars.counter < 140
%     l = 0.5;
%     r = 0.5;
% elseif read_only_vars.counter < 155
%     l = 0.5;
%     r = 0.3;
% elseif read_only_vars.counter < 210
%     l = 0.5;
%     r = 0.5;
% elseif read_only_vars.counter < 225
%     l = 0.5;
%     r = 0.3;
% elseif read_only_vars.counter < 400
%     l = 0.5;
%     r = 0.5;
% elseif read_only_vars.counter < 426
%     l = 0.3;
%     r = 0.5;
% elseif read_only_vars.counter < 600
%     l = 0.5;
%     r = 0.5;
% end

%% Week 3 Task 3
public_vars.estimated_pose = read_only_vars.mocap_pose;

r_pose = public_vars.estimated_pose;

d = read_only_vars.agent_drive.interwheel_dist;
theta = r_pose(3);

epsilon = 0.2;
kappa = 1/(10*epsilon); 

P = r_pose(1:2)+epsilon*[cos(theta), sin(theta)];

[G, vG, public_vars] = get_target(P, public_vars);

% PG_distance

vP = kappa * (G - P) + vG;

if norm(vP) > read_only_vars.agent_drive.max_vel
    vP = vP/norm(vP)*read_only_vars.agent_drive.max_vel;
end

v =  vP(1)*cos(theta) + vP(2)*sin(theta);
omega = (-vP(1)*sin(theta)+vP(2)*cos(theta))/epsilon;

r = v + (omega * d)/2; 
l = v - (omega * d)/2;


%% 
public_vars.motion_vector = [r, l];

end