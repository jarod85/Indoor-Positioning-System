clear all;
close all;
clc;

filename = "rotate";

load(filename);

%% Magnometer

T = timetable2table(MagneticField);

time_data = table2array(T(:, 1));
azimuth = table2array(T(:, 2)); %x dimensional data
y_data = table2array(T(:, 3)); %y dimensional data
z_data = table2array(T(:, 4)); %z dimensional data
x = [azimuth(:),y_data(:),z_data(:)];

figure(1);
subplot(2, 1, 1);
scatter3(azimuth, y_data, z_data);
axis equal
title('Magnetometer Data - Before Calibration');
xlabel("x");
ylabel("y");
zlabel("z");

%% Calibration Step
% Taken from https://au.mathworks.com/help/fusion/ug/magnetometer-calibration.htmlx
[A,b,expMFS]  = magcal(x);
xCorrected = (x-b)*A;
subplot(2, 1, 2);
scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));
axis equal
title('Magnetometer Data - After Calibration');

%% Plotting the angle of the phone against time before and after calibration
figure(2);
Fs = 10;            % Sampling frequency (Hz)                  
T = 1/Fs;             % Sampling period       
L = length(time_data);  % Length of signal = no. datapoints
t_mag = (0:L-1)*T;        % Time vector
subplot(2, 1, 1);
mag_heading = (atan2(xCorrected(:,2),xCorrected(:,1)))*(180/pi);
threshold = 180;
mag_heading = rad2deg(unwrap(deg2rad(mag_heading)));

starting_pos = mag_heading(1);
mag_heading = -(mag_heading-starting_pos);
plot(t_mag, mag_heading,'o')
title("Magnetometer Heading vs Time - After Calibration")
xlabel("Time - Seconds");
ylabel("Angle - Degrees");

hold on; 
subplot(2, 1, 2);
old_angle_magnometer = atand(y_data/azimuth);
plot(t_mag, old_angle_magnometer);
title("Magnetometer Heading vs Time - Before Calibration");
xlabel("Time - Seconds");
ylabel("Angle - Degrees");
hold off;

%% Gyroscope

T = timetable2table(AngularVelocity);

time_data = table2array(T(:, 1));
azimuth = table2array(T(:, 2)); %x dimensional data
y_data = table2array(T(:, 3)); %y dimensional data
z_data = table2array(T(:, 4)); %z dimensional data

Fs = 10;            % Sampling frequency (Hz)                  
T = 1/Fs;             % Sampling period       
L = length(time_data);  % Length of signal = no. datapoints
t_gyro = (0:L-1)*T;        % Time vector
% Method of numerical integration (cumtrapz) to attain angle
x_angle = rad2deg(cumtrapz(t_gyro, azimuth));
y_angle = rad2deg(cumtrapz(t_gyro, y_data));
z_angle = rad2deg(cumtrapz(t_gyro, z_data));

%% Orientation

T = timetable2table(Orientation);

time_data = table2array(T(:, 1));
azimuth_old = table2array(T(:, 2)); %x dimensional data
pitch = table2array(T(:, 3)); %y dimensional data
roll = table2array(T(:, 4)); %z dimensional data 
azimuth = rad2deg(unwrap(deg2rad(azimuth_old)));
starting_pos_orientation = azimuth(1);
azimuth = -(azimuth-starting_pos_orientation);
Fs = 10;            % Sampling frequency (Hz)                  
T = 1/Fs;             % Sampling period       
L = length(time_data);  % Length of signal = no. datapoints
t_ori = (0:L-1)*T;        % Time vector
        
figure(3);
plot(t_ori, (azimuth_old), "r");
hold on;
plot(t_ori, (azimuth), "b");
title("Orientation Heading Before and After Calibration");
xlabel("Time - Seconds");
ylabel("Angle - Degrees");
legend("Before Calibration", "After Calibration");

%% Comparison Plots

figure(4);
plot(t_gyro, z_angle, "r");
hold on;
plot(t_mag, mag_heading, "b");
hold on;
plot(t_ori, azimuth, "g");
hold off;
title('Gyroscope vs Magnometer');
xlabel("Time - Seconds");
ylabel("Angle - Degrees");
legend("Gyroscope", "Magnometer", "Orientation - Baseline");

