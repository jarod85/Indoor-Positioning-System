clear all;
close all;
clc;

filename = "fieldwalk3.mat";
%filename = "uni-stroll.mat";

[distance, max_locs] = adaptive_step_counter(filename);
[gyro_heading, kalman_orientation, mag_heading, baseline_heading] = kalman_heading_estimator(filename);
%orientation = heading_estimator(filename);

Fs = 10;            % Sampling frequency (Hz)                  
T = 1/Fs;             % Sampling period       
L = length(kalman_orientation);  % Length of signal = no. datapoints
t = (0:L-1)*T;           % Time vector

kx = zeros(1, length(distance));
ky = zeros(1, length(distance));
true_x = zeros(1, length(distance));
true_y = zeros(1, length(distance));
magx = zeros(1, length(distance));
magy = zeros(1, length(distance));
gyrox = zeros(1, length(distance));
gyroy = zeros(1, length(distance));
%% Intelligent Implementation
for i = 1:1:length(distance)
    if i == 1
        kx(1) = 0;
        ky(1) = 0;
        true_x(1) = 0;
        true_y(1) = 0;
        magx(1) = 0;
        magy(1) = 0;
        gyrox(1) = 0;
        gyroy(1) = 0;
    else
        index_at_step = max_locs(i);
        distance_diff = abs(distance(i)-distance(i-1));
        if (length(kalman_orientation) >= index_at_step)
            kx(i) = kx(i-1) + distance_diff*cosd(kalman_orientation(index_at_step));
            ky(i) = ky(i-1) + distance_diff*sind(kalman_orientation(index_at_step));
        else
        end
        if (length(mag_heading) >= index_at_step)
            magx(i) = magx(i-1) + distance_diff*cosd(mag_heading(index_at_step));
            magy(i) = magy(i-1) + distance_diff*sind(mag_heading(index_at_step));
        else
        end
        if (length(gyro_heading) >= index_at_step)
            gyrox(i) = gyrox(i-1) + distance_diff*cosd(gyro_heading(index_at_step));
            gyroy(i) = gyroy(i-1) + distance_diff*sind(gyro_heading(index_at_step));
        else
        end
        if (length(baseline_heading) >= index_at_step)
            true_x(i) = true_x(i-1) + distance_diff*cosd(baseline_heading(index_at_step));
            true_y(i) = true_y(i-1) + distance_diff*sind(baseline_heading(index_at_step));
        else
        end
    end
end

%% Simplified Implementation

% dist_k = distance(end)/L;
% dist_gyro = distance(end)/length(gyro_heading);
% 
% for i = 1:1:length(kalman_orientation)
%     if i == 1
%         kx(1) = 0;
%         ky(1) = 0;
%     else
%         kx(i) = kx(i-1) + dist_k*cosd(kalman_orientation(i));
%         ky(i) = ky(i-1) + dist_k*sind(kalman_orientation(i));
%     end
% end
% 
% for j = 1:1:length(gyro_heading)
%     if j == 1
%         true_x(1) = 0;
%         true_y(1) = 0;
%     else
%         true_x(j) = true_x(j-1) + dist_gyro*cosd(gyro_heading(j));
%         true_y(j) = true_y(j-1) + dist_gyro*sind(gyro_heading(j));
%     end
% end

%% GPS

load(filename);
P = timetable2table(Position);
lat = table2array(P(:, 2)); %x dimensional data
lon = table2array(P(:, 3)); %y dimensional data
alt = table2array(P(:, 4)); %z dimensional data

origin = [lat(1), lon(1), alt(1)];
% load("GPS.mat");
[xEast,yNorth,zUp] = latlon2local(lat,lon,alt,origin);
GPSx = -xEast;
GPSy = yNorth;

%% Plotting

figure(3);
plot(kx, ky, "b");
hold on;
plot(true_x, true_y, "r");
hold on;
plot(GPSx, GPSy, "k");
hold on;
plot(magx, magy, "c");
hold on;
plot(gyrox, gyroy, "g");
hold off;
xlabel("X - Axis (m)");
ylabel("Y - Axis (m)");
title("Walking Path");
legend("Kalman Filtered Path", "Orientation Heading - Baseline", "GPS", "Magnetometer Heading", "Gyroscope Heading");
