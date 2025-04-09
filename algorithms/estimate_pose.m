function [public_vars] = estimate_pose(public_vars, read_only_vars)
%ESTIMATE_POSE Summary of this function goes here
    
    dt = read_only_vars.sampling_period;
    est_pos_history = read_only_vars.est_position_history;
    
    threshold_speed = 4.0;    
 

    if (public_vars.kf_enabled)
        public_vars.estimated_pose = public_vars.mu;
    elseif (public_vars.pf_enabled)
       
        public_vars.estimated_pose = median(public_vars.particles);

    
        if isempty(est_pos_history)
            last_stable_pose = public_vars.estimated_pose;
        else
            last_stable_pose = est_pos_history(end,:);
        end

        % If est. pose changes too quickly, replan path 
        speed = norm(public_vars.estimated_pose(1:2) - last_stable_pose(1:2)) / dt;
        if  speed > threshold_speed 
            public_vars.planning_required = 1;
        end
    else
        public_vars.estimated_pose = public_vars.estimated_pose;
    end
end