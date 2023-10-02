clear all;
close all;
clc;

%% Files
filename = "uni45step_run1.mat";
filename = "uni47steps_run2.mat";
%filename = "capstonewalk2_22steps.mat";
%filename = "capstonewalk_12steps_8.2m.mat";
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

beta = 4;
minPeakHeight = std(y)-std(y)/beta; %Minimum height to be counted as a "step"

[max_pks, max_locs] = findpeaks(y, 'MINPEAKHEIGHT',minPeakHeight); % max_pks pertains to toe strikes
flipped_y = -y;
[min_pks, min_locs] = findpeaks(flipped_y, 'MINPEAKHEIGHT', minPeakHeight); %min_pks pertains to heel strikes
numSteps = numel(max_pks); % gets length of array pks which returns the number of steps
disp("The number of steps taken is: " + numSteps)

%% Plotting Acceleration Data
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

%% Algorithm 1

figure(2);
true_dist = 8.2;
step_length = 0.76; %m
Estimated_Distance = numSteps*step_length;
X = ["True Distance"; "Estimated Distance"];
Y = [true_dist; Estimated_Distance]
b = bar(Y, 0.4);
xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')
set(gca, "xticklabel", X);
ylabel("Distance - metres");
title("True Distance vs Estimated Distance - Algorithm 1");

