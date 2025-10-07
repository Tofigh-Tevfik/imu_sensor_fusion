% imu_fft_analysis.m - Visualize IMU data with FFT
clear;
clc;

% Load data from file
filename = 'imu_data_2025-09-06.txt';
if ~exist(filename, 'file')
    error(['File ' filename ' not found. Ensure it''s in the current directory.']);
end

% Read the text file, skipping the header
data = readtable(filename, 'Delimiter', ',', 'HeaderLines', 1, 'ReadVariableNames', false);
data = table2array(data); % Convert to array for easier processing

% Extract columns (assuming format: TS, Gx, Gy, Gz, Ax, Ay, Az)
timestamp = data(:, 1); % Timestamp in ms
gyroX = data(:, 2);    % Gyro X
gyroY = data(:, 3);    % Gyro Y
gyroZ = data(:, 4);    % Gyro Z
accelX = data(:, 5);   % Accel X
accelY = data(:, 6);   % Accel Y
accelZ = data(:, 7);   % Accel Z

% Estimate sampling frequency (based on timestamp differences)
dt = mean(diff(timestamp)) / 1000; % Convert ms to seconds
fs = 1 / dt; % Sampling frequency in Hz
N = length(timestamp); % Number of samples
t = (0:N-1) * dt; % Time vector in seconds

% Time-domain plots
figure('Name', 'IMU Time Domain Data');
subplot(2, 1, 1);
plot(t, gyroX, 'b', t, gyroY, 'r', t, gyroZ, 'g', 'LineWidth', 1.5);
title('Gyroscope Time Domain');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
legend('X', 'Y', 'Z');
grid on;

subplot(2, 1, 2);
plot(t, accelX, 'b', t, accelY, 'r', t, accelZ, 'g', 'LineWidth', 1.5);
title('Accelerometer Time Domain');
xlabel('Time (s)');
ylabel('Acceleration (g)');
legend('X', 'Y', 'Z');
grid on;

% FFT Analysis
nfft = 2^nextpow2(N); % Next power of 2 for efficient FFT
f = fs * (0:(nfft-1)) / nfft; % Frequency vector

% Compute FFT for each axis
gyroX_fft = fft(gyroX, nfft) / N; % Normalize by number of samples
gyroY_fft = fft(gyroY, nfft) / N;
gyroZ_fft = fft(gyroZ, nfft) / N;
accelX_fft = fft(accelX, nfft) / N;
accelY_fft = fft(accelY, nfft) / N;
accelZ_fft = fft(accelZ, nfft) / N;

% Take magnitude (single-sided spectrum)
mag_gyroX = 2 * abs(gyroX_fft(1:nfft/2+1));
mag_gyroY = 2 * abs(gyroY_fft(1:nfft/2+1));
mag_gyroZ = 2 * abs(gyroZ_fft(1:nfft/2+1));
mag_accelX = 2 * abs(accelX_fft(1:nfft/2+1));
mag_accelY = 2 * abs(accelY_fft(1:nfft/2+1));
mag_accelZ = 2 * abs(accelZ_fft(1:nfft/2+1));

% Frequency-domain plots
figure('Name', 'IMU Frequency Domain (FFT)');
subplot(2, 1, 1);
plot(f(1:nfft/2+1), mag_gyroX, 'b', f(1:nfft/2+1), mag_gyroY, 'r', f(1:nfft/2+1), mag_gyroZ, 'g', 'LineWidth', 1.5);
title('Gyroscope FFT Magnitude');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
xlim([0, 100]); % Limit to 100 Hz for clarity
legend('X', 'Y', 'Z');
grid on;

subplot(2, 1, 2);
plot(f(1:nfft/2+1), mag_accelX, 'b', f(1:nfft/2+1), mag_accelY, 'r', f(1:nfft/2+1), mag_accelZ, 'g', 'LineWidth', 1.5);
title('Accelerometer FFT Magnitude');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
xlim([0, 100]); % Limit to 100 Hz for clarity
legend('X', 'Y', 'Z');
grid on;

% Display estimated sampling frequency
fprintf('Estimated Sampling Frequency: %.2f Hz\n', fs);