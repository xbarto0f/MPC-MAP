function [public_vars] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

% public_vars.planning_required = 1;
if public_vars.planning_required
    
    public_vars.path = astar(read_only_vars, public_vars);
    
    public_vars.path = smooth_path(public_vars.path);
    public_vars.planning_required = 0;
else
    
    public_vars.path = public_vars.path;
    
end

end

