function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
    
    % 8. Perform initialization procedure
    if (read_only_vars.counter == 1)
              
        % public_vars = init_particle_filter(read_only_vars, public_vars);
        
        public_vars.max_vel = 0.8;              % max starting speed
        public_vars.path_index = 1;             % current path index
        public_vars.weights = [];               % init W for PF
        public_vars.planning_required = 1;      % flag for path planner
    
        public_vars.state = 'START';        % init state of state machine
        public_vars.help_counter = 1;
        public_vars.help_counter2 = 1;
        public_vars.help_counter_p =1;
        public_vars.init_pf_w_gnns = 0;
        
        public_vars.pf_inited = 0;
        public_vars.pf_no_rnd = 0;
        public_vars.pf_type = 1;

        public_vars.ignore_motion_plan = 0;     
    end
   
    
    switch public_vars.state
        case 'START'
    
            if (~isnan(read_only_vars.gnss_position))
                public_vars.state = 'KF_INIT';
                public_vars.help_counter = 1;
                disp("SWITCHING TO EKF")
            else
                public_vars.state = 'PF_INIT';
                public_vars.help_counter = 1;
                public_vars.pf_inited = 0;
                disp("SWITCHING TO PF")
            end
    
            
        case 'KF_INIT'
            % dont move for few iterations before kalman init
            public_vars.help_counter = public_vars.help_counter +1;
            public_vars.motion_vector = [0,0];

            if (public_vars.help_counter >= 100)
                public_vars.kf_enabled = 1;
                public_vars = init_kalman_filter(read_only_vars, public_vars);
                [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
                public_vars = estimate_pose(public_vars, read_only_vars); 
                
                public_vars.max_vel = 0.1; 
                public_vars.help_counter = 1;

                [public_vars] = plan_path(read_only_vars, public_vars);
                public_vars.state = 'KF';
            end
    
        case 'KF'
            % speed up
            public_vars.help_counter = public_vars.help_counter +1;
            public_vars.help_counter_p = public_vars.help_counter_p +1;
            if (public_vars.help_counter >= 150)
                public_vars.max_vel = 0.8;
            end
            
            % robot entering inside
            if (isnan(read_only_vars.gnss_position))
               public_vars.kf_enabled = 0;
               public_vars.help_counter = 1;
               
               public_vars.init_pf_w_gnns = 1;
               public_vars.pf_type = 1;
               public_vars.pf_inited = 0;
               public_vars.state = 'PF_INIT';
               disp("SWITCHING TO PF")
            end
            
            % path start too far from robot, replan
            if(~isempty(public_vars.path))
                dist = norm(public_vars.estimated_pose(1:2) - public_vars.path(public_vars.path_index,1:2));
                if(dist > 1)
                    disp("Path start too far")
                    public_vars.planning_required = 1;
                    public_vars = plan_path(read_only_vars, public_vars);
                end
            end
            
            % if no path, plan path
            if(isempty(public_vars.path) && public_vars.help_counter_p >60)
                public_vars.help_counter_p = 1;
                public_vars.planning_required = 1;
                public_vars = plan_path(read_only_vars, public_vars);
            end


            public_vars = estimate_pose(public_vars,read_only_vars); 
            [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
            public_vars = plan_motion(read_only_vars, public_vars);
            
    
        case 'PF_INIT'
            public_vars = estimate_pose(public_vars, read_only_vars);
            public_vars.pf_enabled = 1;         
            
            % init particles
            if(~public_vars.pf_inited)
                public_vars = init_particle_filter(read_only_vars, public_vars);
                public_vars.pf_inited =1;
                public_vars.pf_no_rnd = 1;
            end
            if(~public_vars.init_pf_w_gnns)
                % spin&wait for few iters before normal pf usage
                 public_vars.help_counter = public_vars.help_counter +1;
                 if (public_vars.help_counter < 180)
                    public_vars.motion_vector = [0,0.2];
                    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
                    public_vars = estimate_pose(public_vars, read_only_vars);
                 else
                     [public_vars] = plan_path(read_only_vars, public_vars);
                     public_vars.init_pf_w_gnns = 0;
                     public_vars.state = 'PF';
                 end
            else
                % public_vars = init_particle_filter(read_only_vars, public_vars);
                public_vars.state = 'PF';
            end
           
    
        case 'PF'
            public_vars.particles = update_particle_filter(read_only_vars, public_vars);
            public_vars = estimate_pose(public_vars, read_only_vars);

            public_vars.help_counter = public_vars.help_counter+1;
            
            % path start too far from robot, replan
            if(~isempty(public_vars.path))
                dist = norm(public_vars.estimated_pose(1:2) - public_vars.path(public_vars.path_index,1:2));
                if(dist > 1)
                    disp("Path start too far")
                    public_vars.planning_required = 1;
                    public_vars = plan_path(read_only_vars, public_vars);
                end
            end
            
            % if est robot stuck near goal, reinit pf
            if (norm(public_vars.estimated_pose(1:2) - read_only_vars.map.goal) < 1)
                public_vars.help_counter2 = public_vars.help_counter2+1;
            else
                public_vars.help_counter2 = 1;
            end
            if (public_vars.help_counter2 >50)
                public_vars.state = 'PF_INIT';
                public_vars.help_counter = 1;
                public_vars.pf_inited = 0;
                disp("RESTARTING PF")
            end

            % if no path, plan path
            if(isempty(public_vars.path) && public_vars.help_counter >60)
                public_vars.help_counter = 1;
                public_vars.planning_required = 1;
                public_vars = plan_path(read_only_vars, public_vars);
            end
            
            % robot entering outside
             if (~any(isnan(read_only_vars.gnss_position)))
                public_vars.ignore_motion_plan = 1;
                if(~any(isnan(read_only_vars.gnss_history(max(1, end-20):end, :)), 'all'))
                    public_vars.state = 'KF_INIT';
                    public_vars.help_counter = 1;
                    public_vars.pf_enabled = 0; 
                    public_vars.ignore_motion_plan = 0;

                    disp("SWITCHING TO EKF")
                end
            end

            public_vars = plan_motion(read_only_vars, public_vars);
    

        case 'END'
            disp('State: END');
    
    
        otherwise
            disp('HELP!');
    
    end

end

