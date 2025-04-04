function [public_vars] = init_particle_filter(read_only_vars, public_vars)
    %INIT_PARTICLE_FILTER Summary of this function goes here  
    map_limits = read_only_vars.map.gnss_denied;
    x_min = min([map_limits(1),map_limits(3),map_limits(5),map_limits(7)])-0.1;
    x_max = max([map_limits(1),map_limits(3),map_limits(5),map_limits(7)])+0.1;
    y_min = min([map_limits(2),map_limits(4),map_limits(6),map_limits(8)])-0.1;
    y_max = max([map_limits(2),map_limits(4),map_limits(6),map_limits(8)])+0.1;

    N = 600;
    
    prtcls = zeros(N,3);
    if (public_vars.init_pf_w_gnns)
        % public_vars.particles(N,:) = public_vars.estimated_pose;
        public_vars.particles = repmat(public_vars.estimated_pose, N, 1);
    else
        for i=1:N
            % prtcls(i,1:2) = 10.*rand(1,2);
            prtcls(i,1) = x_min + (x_max - x_min) * rand();
            prtcls(i,2) = y_min + (y_max - y_min) * rand();
            prtcls(i,3) = -pi + (pi+pi).*rand(1,1);
        end
        public_vars.particles = prtcls;
    end
end

