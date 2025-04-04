function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

    d = read_only_vars.agent_drive.interwheel_dist; 
    
    x = old_pose(1);
    y = old_pose(2);
    theta = old_pose(3);
    r = motion_vector(1)+(randn(1)-0.5)/10;
    l = motion_vector(2)+(randn(1)-0.5)/10;
    dt = read_only_vars.sampling_period;    

    v = (r + l) / 2;
    omega = (r - l) / d;

    x_new = x + (v * cos(theta) * dt);
    y_new = y + (v * sin(theta) * dt);
    theta_new = theta + (omega*dt);
    
    new_pose = [x_new+(randn()/60); y_new+(randn()/60); theta_new+(randn()/100)];
end

