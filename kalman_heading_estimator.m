function [z_angle kalman_orientation mag_heading baseline_heading] = heading_estimator(filename)
    load(filename);
    
    %% Magnometer
    
    T = timetable2table(MagneticField);
    
    time_data = table2array(T(:, 1));
    x_data = table2array(T(:, 2)); %x dimensional data
    y_data = table2array(T(:, 3)); %y dimensional data
    z_data = table2array(T(:, 4)); %z dimensional data
    x = [x_data(:),y_data(:),z_data(:)];
    
    %% Calibration Step
    % Taken from https://au.mathworks.com/help/fusion/ug/magnetometer-calibration.htmlx
    [A,b,expMFS]  = magcal(x);
    xCorrected = (x-b)*A;
    Fs = 10;            % Sampling frequency (Hz)                  
    T = 1/Fs;             % Sampling period       
    L = length(time_data);  % Length of signal = no. datapoints
    t_mag = (0:L-1)*T;        % Time vector
    mag_heading = (atan2(xCorrected(:,2),xCorrected(:,1)))*(180/pi);
    mag_heading = rad2deg(unwrap(deg2rad(mag_heading)));
    
    starting_pos = mag_heading(1);
    mag_heading = -(mag_heading-starting_pos);

    
    %% Gyroscope
    
    T = timetable2table(AngularVelocity);
    
    time_data = table2array(T(:, 1));
    x_data = table2array(T(:, 2)); %x dimensional data
    y_data = table2array(T(:, 3)); %y dimensional data
    z_data = table2array(T(:, 4)); %z dimensional data
    
    Fs = 10;            % Sampling frequency (Hz)                  
    T = 1/Fs;             % Sampling period       
    L = length(time_data);  % Length of signal = no. datapoints
    t_gyro = (0:L-1)*T;        % Time vector
    % Method of numerical integration (cumtrapz) to attain angle
    x_angle = rad2deg(cumtrapz(t_gyro, x_data));
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
    baseline_heading = azimuth;

    %% kalman_filter
    % initialising
    % state transition matrix
    A = 1;
    % input control matrix
    B = 1/Fs;
    % predidction
    u = z_angle;
    %measurement
    y = mag_heading;
    kalman_orientation = zeros(1, length(L));

    R = 0.1697; % Measurement model noise = magnetometer
    Q = 6.819; % Prediction model noise = gyro
    P_past = R+Q; % Error covariance matrix

    C = 1; % Measurement Matrix
    x_past = 0; % initial angle
    
    if (length(u) <= length(y))
        L = length(u);
        t_kalman = t_gyro;
    else
        L = length(y);
        t_kalman = t_mag;
    end

    for i = 1:1:L
        %Prediction
        x_current = A*x_past + B*u(i);
        P_current = A*P_past*A' + Q
        
        %Update
        K = (P_past*C')/(C*P_past*C'+R);
        x_past = x_current + K*(y(i)-C*x_current);
        P_past = (1-K*C)*P_current;
        kalman_orientation(i) = x_past;
    end

    figure(2);
    plot(t_gyro, z_angle, "r");
    hold on;
    plot(t_mag, mag_heading, "b");
    hold on;
    plot(t_ori, azimuth, "g");
    hold on;
    plot(t_kalman, kalman_orientation, "k")
    title('Heading Estimator Comparison');
    xlabel("Time - Seconds");   
    ylabel("Angle - Degrees");
    legend("Gyroscope", "Magnetometer", "Orientation - Baseline", "Kalman Orientation");

end

