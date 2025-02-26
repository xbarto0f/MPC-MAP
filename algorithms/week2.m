%% Task 2
%GNSS
load("./gnss_data.mat")
disp("GNSS standard deviation:")
gnss_sigma = std(public_vars.gnss_data)

figure(2) 
subplot(1,2,1) 
histogram(public_vars.gnss_data(:,1))
title('GNSS Data Axis 1')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(1,2,2) 
histogram(public_vars.gnss_data(:,2))
title('GNSS Data Axis 2')
xlabel("Distance [m]")
ylabel("Count [-]")

% LIDAR
load("./lidar_data.mat")
disp("LIDAR standard deviation:")
lidar_sigma = std(public_vars.lidar_data)

figure(3)
subplot(2,4,1) 
histogram(public_vars.lidar_data(:,1))
title('LIDAR Data Axis 1')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,2) 
histogram(public_vars.lidar_data(:,2))
title('LIDAR Data Axis 1')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,3) 
histogram(public_vars.lidar_data(:,3))
title('LIDAR Data Axis 3')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,4) 
histogram(public_vars.lidar_data(:,4))
title('LIDAR Data Axis 4')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,5) 
histogram(public_vars.lidar_data(:,5))
title('LIDAR Data Axis 5')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,6) 
histogram(public_vars.lidar_data(:,6))
title('LIDAR Data Axis 6')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,7) 
histogram(public_vars.lidar_data(:,7))
title('LIDAR Data Axis 7')
xlabel("Distance [m]")
ylabel("Count [-]")

subplot(2,4,8) 
histogram(public_vars.lidar_data(:,8))
title('LIDAR Data Axis 8')
xlabel("Distance [m]")
ylabel("Count [-]")

%% Task 3
load("./gnss_data.mat")
disp("GNNS covariance matrix :")
gnss_cm = cov(public_vars.gnss_data)

load("./lidar_data.mat")
disp("LIDAR covariance matrix :")
lidar_cm = cov(public_vars.lidar_data)

%% Task 4
max_s = max(lidar_sigma(1), gnss_sigma(1));
x = linspace(-5*max_s, 5*max_s, 1000);

gnss_norm = norm_pdf(x, 0, gnss_sigma(1));
lidar_norm = norm_pdf(x, 0, lidar_sigma(1));

figure(4);
plot(x, lidar_norm, 'b', 'LineWidth', 3);
hold on;
plot(x, gnss_norm, 'g', 'LineWidth', 3);
xlabel('Measurement Error (x)');
ylabel('Probability Density');
legend('LIDAR', 'GNSS');
grid on;
