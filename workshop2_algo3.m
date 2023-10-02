clear all;
close all;
clc;

%% Files
filename = "uni47steps_run2.mat";
%filename = "capstonewalk2_22steps.mat";
%filename = "capstonewalk_12steps_8.2m.mat";
load(filename); 

%% Processing Acceleration Data

%Acceleration is name of imported timetable
T = timetable2table(Acceleration);

time_data = table2array(T(:, 1));
ax_data = table2array(T(:, 2)); %x dimensional data
ay_data = table2array(T(:, 3)); %y dimensional data
az_data = table2array(T(:, 4)); %z dimensional data

a_norm = az_data; %norm of acceleration vector
a_norm = a_norm-mean(a_norm); %removes gravity 

%% Fourier Transform

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

beta = 4;
minPeakHeight = std(y)-std(y)/beta; %Minimum height to be counted as a "step"

[pks,locs] = findpeaks(y, 'MINPEAKHEIGHT' ,minPeakHeight); %Return the location of peaks "loc" and height of each peak "pks"

%[pks, locs] = findpeaks(y, "MinPeakProminence", 0.4);

numSteps = numel(pks); % gets length of array pks which returns the number of steps
disp("The number of steps taken is: " + numSteps)

%Graphs

%% Plotting acceleration data

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
plot(t(locs), pks, 'r', 'Marker', 'v', 'LineStyle', 'none');
title('Counting Steps');
xlabel('Relative Time (s)');
ylabel({"Acceleration Magnitude", "Accounting For Gravity - m/s^2"});
hold off;

%% Plotting algorithm 3

figure(2);
%Acceleration
acc = pks;
subplot(3, 1, 1);
plot(t(locs), acc);
xlabel("Time - seconds");
ylabel("Acceleration - m/s^2");
title("Acceleration vs Time");

%First integration: Acceleration - Veloctiy
% pks is where max acceleration occurs for each gait cycle
vel = cumtrapz(t(locs), acc);
subplot(3, 1, 2);
plot(t(locs), vel);
xlabel("Time - seconds");
ylabel("Velocity - m/s");
title("Velocity vs Time");

%2nd integration: Velocity - Displacement
displacement = cumtrapz(t(locs), vel);
subplot(3, 1, 3);
plot(t(locs), displacement, "r");
hold on;
% capstone office is 15.33m 
%mid section of capstone office is 8.2m
% home is 4.1m+3.55m
%True_Distance = input("Enter the true distance walked: "); %m %15.33
true_dist = 15.33;
time = t(locs);
m = (true_dist/time(end));
true_dist = m.*t(locs);
plot(t(locs), true_dist, "b");
legend("Estimated Distance", "True Distance: " + true_dist(end) + "m");
xlabel("Time - seconds");
ylabel("Displacement - Metres");
title("Displacement vs Time - Algorithm 3");
hold off;

