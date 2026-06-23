% IMU analýza: raw data, FFT, low-pass filtr, pásmová zádrž
% Data: rosbag2_2026_06_10-15_57_51

clear; clc; close all;

%% ── Parametry ────────────────────────────────────────────────────────────
BAG_PATH = '/mnt/ros2bags/rosbag2_2026_06_10-15_57_51';

% 'oak' = OAK-D Pro W (~400 Hz), 'fc' = flight controller (~86 Hz)
IMU_SOURCE = 'oak';

LP_CUTOFF_HZ = 50;   % low-pass mezní frekvence [Hz]
BS_CENTER_HZ = 120;   % pásmová zádrž: střed [Hz]
BS_BW_HZ     = 40;   % pásmová zádrž: šířka pásma [Hz]  (center ± bw/2)
FILTER_ORDER = 4;
%% ─────────────────────────────────────────────────────────────────────────

TOPICS = struct( ...
    'oak', '/f450_1/sensors/oak_d_pro_w/imu/data', ...
    'fc',  '/f450_1/aircraft/imu');

topic = TOPICS.(IMU_SOURCE);

fprintf('Čtu bag: %s\n', BAG_PATH);
bag  = ros2bagreader(BAG_PATH);
msgs = readMessages(select(bag, 'Topic', topic));

N  = numel(msgs);
t  = zeros(N,1);
ax = zeros(N,1); ay = zeros(N,1); az = zeros(N,1);
gx = zeros(N,1); gy = zeros(N,1); gz = zeros(N,1);

for i = 1:N
    m    = msgs{i};
    t(i) = double(m.header.stamp.sec) + double(m.header.stamp.nanosec) * 1e-9;
    ax(i) = m.linear_acceleration.x;
    ay(i) = m.linear_acceleration.y;
    az(i) = m.linear_acceleration.z;
    gx(i) = m.angular_velocity.x;
    gy(i) = m.angular_velocity.y;
    gz(i) = m.angular_velocity.z;
end

t  = t - t(1);
Fs = 1 / median(diff(t));
fprintf('Vzorků: %d | Fs ≈ %.1f Hz\n', N, Fs);

signals = {ax, ay, az, gx, gy, gz};
labels  = {'ax [m/s²]','ay [m/s²]','az [m/s²]', ...
           'gx [rad/s]','gy [rad/s]','gz [rad/s]'};
colors  = lines(6);

%% ── 1. Raw data ──────────────────────────────────────────────────────────
figure('Name','IMU raw data','NumberTitle','off','Position',[50 50 1400 700]);
for i = 1:6
    subplot(6,1,i);
    plot(t, signals{i}, 'Color', colors(i,:), 'LineWidth', 0.5);
    ylabel(labels{i}); grid on;
    if i == 1, title(sprintf('Raw IMU (%s)', IMU_SOURCE)); end
    if i == 6, xlabel('čas [s]'); end
end

%% ── 2. FFT ───────────────────────────────────────────────────────────────
freq = (0:N-1) * Fs / N;
f    = freq(1:floor(N/2));

figure('Name','FFT','NumberTitle','off','Position',[50 800 1400 700]);
for i = 1:6
    Y = abs(fft(signals{i} - mean(signals{i}))) / N;
    subplot(6,1,i);
    plot(f, 2*Y(1:floor(N/2)), 'Color', colors(i,:), 'LineWidth', 0.7);
    ylabel(labels{i}); grid on; xlim([0 Fs/2]);
    if i == 1, title('FFT amplitudové spektrum'); end
    if i == 6, xlabel('frekvence [Hz]'); end
end

%% ── Návrh filtrů ─────────────────────────────────────────────────────────
[b_lp, a_lp] = butter(FILTER_ORDER, LP_CUTOFF_HZ / (Fs/2), 'low');

Wn_bs = [(BS_CENTER_HZ - BS_BW_HZ/2), (BS_CENTER_HZ + BS_BW_HZ/2)] / (Fs/2);
Wn_bs = max(min(Wn_bs, 0.9999), 1e-4);
[b_bs, a_bs] = butter(FILTER_ORDER, Wn_bs, 'stop');

%% ── 3. Low-pass ──────────────────────────────────────────────────────────
figure('Name',sprintf('Low-pass (%.0f Hz)', LP_CUTOFF_HZ), ...
       'NumberTitle','off','Position',[500 50 1400 700]);
for i = 1:6
    filt = filtfilt(b_lp, a_lp, signals{i});
    subplot(6,1,i); hold on;
    plot(t, signals{i}, 'Color', [colors(i,:) 0.2], 'LineWidth', 0.4);
    plot(t, filt,       'Color', colors(i,:),        'LineWidth', 1.2);
    ylabel(labels{i}); grid on;
    if i == 1
        title(sprintf('Low-pass filtr – mezní frekvence = %.0f Hz', LP_CUTOFF_HZ));
        legend('raw','filtrováno','Location','best');
    end
    if i == 6, xlabel('čas [s]'); end
end

%% ── 4. Pásmová zádrž ─────────────────────────────────────────────────────
figure('Name',sprintf('Pásmová zádrž (%.0f ± %.0f Hz)', BS_CENTER_HZ, BS_BW_HZ/2), ...
       'NumberTitle','off','Position',[950 50 1400 700]);
for i = 1:6
    filt = filtfilt(b_bs, a_bs, signals{i});
    subplot(6,1,i); hold on;
    plot(t, signals{i}, 'Color', [colors(i,:) 0.2], 'LineWidth', 0.4);
    plot(t, filt,       'Color', colors(i,:),        'LineWidth', 1.2);
    ylabel(labels{i}); grid on;
    if i == 1
        title(sprintf('Pásmová zádrž – %.0f Hz ± %.0f Hz', BS_CENTER_HZ, BS_BW_HZ/2));
        legend('raw','filtrováno','Location','best');
    end
    if i == 6, xlabel('čas [s]'); end
end
