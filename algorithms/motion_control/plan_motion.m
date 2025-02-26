function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

%% Week 2 Task 5
l = 0.0;
r = 0.0;

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

%% 
public_vars.motion_vector = [r, l];

end