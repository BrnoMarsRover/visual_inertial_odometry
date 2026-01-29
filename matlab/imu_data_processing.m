bag = ros2bagreader("flight_madwick_imu_0-01_vio.bag\");

imu_topic = "/f450_1/aircraft/imu/filtered";
imu_messages = readMessages(select(bag,"Topic",imu_topic));

N = length(imu_messages);

t = zeros(N,1);
ax = zeros(N,1);
ay = zeros(N,1);
az = zeros(N,1);

for i = 1:N
    msg = imu_messages{i};

    % timestamp (sekundy)
    t(i) = double(msg.header.stamp.sec) + ...
           double(msg.header.stamp.nanosec)*1e-9;

    % acceleration
    ax(i) = msg.linear_acceleration.x;
    ay(i) = msg.linear_acceleration.y;
    az(i) = msg.linear_acceleration.z;
end

% normalizuj čas
t = t - t(1);

figure;
subplot(3,1,1)
plot(t, ax); grid on;
ylabel('a_x [m/s^2]')

subplot(3,1,2)
plot(t, ay); grid on;
ylabel('a_y [m/s^2]')

subplot(3,1,3)
plot(t, az); grid on;
ylabel('a_z [m/s^2]')
xlabel('time [s]')

sgtitle('RAW IMU acceleration')

fs = 1 / mean(diff(t));
fc = 1;              % cutoff 20 Hz (rozumný kompromis)
[b,a] = butter(2, fc/(fs/2), 'low');

ax_f = filtfilt(b,a,ax);
ay_f = filtfilt(b,a,ay);
az_f = filtfilt(b,a,az);

figure;
subplot(3,1,1)
plot(t, ax, 'r'); hold on;
plot(t, ax_f, 'k','LineWidth',1.2);
grid on;
ylabel('a_x')
legend('raw','LPF')

subplot(3,1,2)
plot(t, ay, 'r'); hold on;
plot(t, ay_f, 'k','LineWidth',1.2);
grid on;
ylabel('a_y')

subplot(3,1,3)
plot(t, az, 'r'); hold on;
plot(t, az_f, 'k','LineWidth',1.2);
grid on;
ylabel('a_z')
xlabel('time [s]')

sgtitle('IMU acceleration – RAW vs Low-pass filtered')
% === FFT (single-sided spectrum) ===
N_fft = 2^nextpow2(N);
f = fs*(0:(N_fft/2))/N_fft;

AX_fft  = fft(ax,  N_fft);
AY_fft  = fft(ay,  N_fft);
AZ_fft  = fft(az,  N_fft);

AXf_fft = fft(ax_f, N_fft);
AYf_fft = fft(ay_f, N_fft);
AZf_fft = fft(az_f, N_fft);

% amplitudové spektrum (jednostranné)
AX  = abs(AX_fft(1:N_fft/2+1));
AY  = abs(AY_fft(1:N_fft/2+1));
AZ  = abs(AZ_fft(1:N_fft/2+1));

AXf = abs(AXf_fft(1:N_fft/2+1));
AYf = abs(AYf_fft(1:N_fft/2+1));
AZf = abs(AZf_fft(1:N_fft/2+1));

figure;
subplot(3,1,1)
plot(f, AX, 'r'); hold on;
plot(f, AXf, 'k','LineWidth',1.2);
xlim([0 100]); grid on;
ylabel('|A_x(f)|')
legend('raw','LPF')

subplot(3,1,2)
plot(f, AY, 'r'); hold on;
plot(f, AYf, 'k','LineWidth',1.2);
xlim([0 100]); grid on;
ylabel('|A_y(f)|')

subplot(3,1,3)
plot(f, AZ, 'r'); hold on;
plot(f, AZf, 'k','LineWidth',1.2);
xlim([0 100]); grid on;
ylabel('|A_z(f)|')
xlabel('Frequency [Hz]')

sgtitle('IMU FFT – single-sided spectrum (RAW vs LPF)')


