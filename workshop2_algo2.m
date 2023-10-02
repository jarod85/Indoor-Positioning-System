clear all;
close all;
clc;

%% Files
%filename = "uni45step_run1.mat";
%filename = "uni47steps_run2.mat";
%filename = "uni47step_run3.mat";
%filename = "capstonewalk2_22steps.mat";
filename = "capstonewalk_12steps_8.2m.mat";
load(filename); 


%% Processing the acceleration data

%Acceleration is name of imported timetable
T = timetable2table(Acceleration);

time_data = table2array(T(:, 1));
ax_data = table2array(T(:, 2)); %x dimensional data
ay_data = table2array(T(:, 3)); %y dimensional data
az_data = table2array(T(:, 4)); %z dimensional data

a_norm = sqrt(ax_data.^2+ay_data.^2+az_data.^2); %norm of acceleration vector
a_norm = a_norm-mean(a_norm); %removes gravity 

% Lines below taken from MATLAB Documentation https://au.mathworks.com/help/matlab/ref/fft.html
X = a_norm;
Y = fft(X); %Computed DFT

Fs = 10;            % Sampling frequency (Hz)                  
T = 1/Fs;             % Sampling period       
L = length(time_data);  % Length of signal = no. datapoints
t = (0:L-1)*T;        % Time vector

% Double sided amplitude spectrum

%ffshift 
Yshift2 = abs(fftshift(Y));
fshift2 = linspace(-Fs/2, Fs/2, L);

%Single sided amplitude spectrum

Yshift1 = Yshift2(L/2+1:L);
fshift1 = fshift2(L/2+1:L);

% Get location of where highest peak occurs
highest_val = 0;
location = 0;
for i = 1:length(Yshift1)
    if (Yshift1(i) > highest_val)
        highest_val = Yshift1(i);   
        location = i;
    end
end
highest_val_freq = fshift1(location);

%% Filtering 
n = 2; % order of butterworth filter used
%Picking lower and upper bound of bandpass filter according to data
difference = 0.25; %making algorithm intellegent by always creating a filter with respect to walking frequency
lowbound = highest_val_freq-difference; %Hz
upperbound = highest_val_freq+difference; %Hz

%Conversion
w1 = lowbound*(2/Fs); 
w2 = upperbound*(2/Fs); 
[b,a] = butter(n, [w1,w2], "bandpass");
y = filter(b, a, a_norm);
        

% Count the number of steps taken

%Lines below taken from https://au.mathworks.com/help/matlabmobile/ug/counting-steps-by-capturing-acceleration-data.html

% Determined through trial and error
beta = 4;
minPeakHeight = std(y)-std(y)/beta; %Minimum height to be counted as a "step"

[max_pks, max_locs] = findpeaks(y, 'MINPEAKHEIGHT',minPeakHeight); % max_pks pertains to toe strikes
flipped_y = -y;
[min_pks, min_locs] = findpeaks(flipped_y, 'MINPEAKHEIGHT', minPeakHeight); %min_pks pertains to heel strikes
numSteps = numel(max_pks); % gets length of array pks which returns the number of steps
disp("The number of steps taken is: " + numSteps)

%Finding step duration:
step_duration = [];
if (length(min_locs) < length(max_locs))
    size = length(min_locs);
elseif (length(max_locs) < length(min_locs))
    size = length(max_locs);
else
    size = length(max_locs);
end
for i = 1:size
    step_duration(i) = abs(t(min_locs(i))-t(max_locs(i)));
end

%% Algorithm 2

avg_step_len = 0.46;
shoe_length = 0.3;
distance = zeros(1, length(max_pks));
displacement = zeros(1, length(max_pks));
avg_accel = mean(max_pks); %Baseline acceleration
avg_step_duration = mean(step_duration); %Baseline step length
%constants
alpha = 1.3;
beta = 0.8;
for i = 1:length(max_pks)
    acc_weight = alpha*max_pks(i)/avg_accel;
    if (i > length(step_duration))
        step_weight = avg_step_duration;
    else
        step_weight = beta*step_duration(i)/avg_step_duration;
    end
    % average out 2 factors to get their overall impact on step length
    displacement(i) = ((acc_weight + step_weight)/2)*avg_step_len + shoe_length;
    if (i == 1)
        distance(i) = distance(i) + displacement(i);
    else
        distance(i) = distance(i-1)+displacement(i);
    end
end

%% Plots

figure(1);
subplot(3, 1, 1);
plot(t, a_norm);
xlabel("Relative Time (s)");
ylabel({"Acceleration Magnitude", "Accounting For Gravity - m/s^2"});
title("Acceleration vs Relative Time - Corrupted Data");

subplot(3, 1, 2);
plot(t, y);
title("Filtered Data");
xlabel("Relative Time (s)"); 
ylabel({"Acceleration Magnitude", "Accounting For Gravity - m/s^2"});

subplot(3, 1, 3);
plot(t, y);
hold on;
plot(t(max_locs), max_pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
hold on;
plot(t(min_locs), -min_pks, "g", "Marker", "v", "LineStyle", "none");
title('Counting Steps');
xlabel('Relative Time (s)');
ylabel({"Acceleration Magnitude", "Accounting For Gravity - m/s^2"});
legend("Filtered Data", "Toe Strikes", "Heel Strikes");
hold off;

figure(2);
%start at (0,0)
plot([0, t(max_locs(1)), t(max_locs)], [0, 0, distance], "r");
hold on;
xlabel("Time - seconds");
ylabel("Distance - metres");
title("Distance vs Time - Algorithm 2");

%% True Distance
% capstone office is 15.33m 
%mid section of capstone office is 8.2m
% home is 4.1m+3.55m
%uni walk 37.4800m
True_Distance = 37.4800;
time = t(max_locs);
m = (True_Distance/time(end));
True_Distance = m.*t(max_locs);
%Starting at (0, 0)
plot([0, t(max_locs)], [0, True_Distance], "b");
legend("Estimated Distance", "True Distance: " + True_Distance(end) + "m");
hold off;


