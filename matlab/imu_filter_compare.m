%% imu_filter_compare.m
% Skript pro porovnání IMU filtrů: RAW -> Lowpass -> Madwick
% Autor: Martin Kriz
%
% Načte data ze 3 topiců a porovná je s vlastním nastavením filtru

clear all
close all

%% KONFIGURACE
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags');

% Rosbag s filtrovanými daty
bag_path = fullfile(rosbag_base_path, "winter_flight1/flight_lowpass1hz_madwick0_01");

% Topic names
imu_raw_topic      = "/f450_1/aircraft/imu";
imu_lowpass_topic  = "/f450_1/aircraft/imu/lowpass";
imu_filtered_topic = "/f450_1/aircraft/imu/filtered";

%% NAČTENÍ DAT
fprintf('Načítám rosbag: %s\n', bag_path);
bag = ros2bagreader(bag_path);

% RAW data
fprintf('Načítám RAW IMU data...\n');
raw_msgs = readMessages(select(bag, "Topic", imu_raw_topic));
[t_raw, accel_raw, gyro_raw] = extract_imu_data(raw_msgs);

% Lowpass data
fprintf('Načítám Lowpass IMU data...\n');
try
    lp_msgs = readMessages(select(bag, "Topic", imu_lowpass_topic));
    [t_lp, accel_lp, gyro_lp] = extract_imu_data(lp_msgs);
    has_lowpass = true;
catch
    warning('Lowpass data nenalezena');
    has_lowpass = false;
end

% Filtered (Madwick) data
fprintf('Načítám Filtered (Madwick) IMU data...\n');
try
    filt_msgs = readMessages(select(bag, "Topic", imu_filtered_topic));
    [t_filt, accel_filt, gyro_filt] = extract_imu_data(filt_msgs);
    has_filtered = true;
catch
    warning('Filtered data nenalezena');
    has_filtered = false;
end

% Normalizace času
t_raw = t_raw - t_raw(1);
if has_lowpass, t_lp = t_lp - t_lp(1); end
if has_filtered, t_filt = t_filt - t_filt(1); end

% Sampling frekvence
fs = 1 / mean(diff(t_raw));
fprintf('Sampling frekvence: %.1f Hz\n', fs);

%% ====================================================================
%% OKNO 1: Akcelerace - RAW / Lowpass / Filtered (Madwick)
%% ====================================================================
figure('Name', '1: IMU Akcelerace', 'NumberTitle', 'off');
axis_labels = {'X', 'Y', 'Z'};

for j = 1:3
    subplot(3,1,j);

    h_raw = plot(t_raw, accel_raw(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;

    if has_lowpass
        h_lp = plot(t_lp, accel_lp(:,j), 'b-', 'LineWidth', 1.2);
    end

    if has_filtered
        h_filt = plot(t_filt, accel_filt(:,j), 'r-', 'LineWidth', 1.2);
    end

    hold off;
    grid on;
    ylabel(sprintf('a_%s [m/s^2]', axis_labels{j}));

    if j == 1
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_lowpass
            leg_h(end+1) = h_lp;
            leg_str{end+1} = 'Lowpass';
        end
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Čas [s]');
    end
end
sgtitle('IMU Akcelerace: RAW → Lowpass → Madwick');

%% ====================================================================
%% OKNO 2: Gyroskop - RAW / Lowpass / Filtered (Madwick)
%% ====================================================================
figure('Name', '2: IMU Gyroskop', 'NumberTitle', 'off');

for j = 1:3
    subplot(3,1,j);

    h_raw = plot(t_raw, gyro_raw(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;

    if has_lowpass
        h_lp = plot(t_lp, gyro_lp(:,j), 'b-', 'LineWidth', 1.2);
    end

    if has_filtered
        h_filt = plot(t_filt, gyro_filt(:,j), 'r-', 'LineWidth', 1.2);
    end

    hold off;
    grid on;
    ylabel(sprintf('\\omega_%s [rad/s]', axis_labels{j}));

    if j == 1
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_lowpass
            leg_h(end+1) = h_lp;
            leg_str{end+1} = 'Lowpass';
        end
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Čas [s]');
    end
end
sgtitle('IMU Gyroskop: RAW → Lowpass → Madwick');

%% ====================================================================
%% OKNO 3: FFT Akcelerace
%% ====================================================================
figure('Name', '3: FFT Akcelerace', 'NumberTitle', 'off');

N_fft = 2^nextpow2(length(t_raw));
f = fs*(0:(N_fft/2))/N_fft;

for j = 1:3
    subplot(3,1,j);

    % RAW FFT
    fft_raw = abs(fft(accel_raw(:,j), N_fft));
    fft_raw = fft_raw(1:N_fft/2+1);
    h_raw = plot(f, fft_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;

    % Lowpass FFT
    if has_lowpass
        accel_lp_interp = interp1(t_lp, accel_lp(:,j), t_raw, 'linear', 'extrap');
        fft_lp = abs(fft(accel_lp_interp, N_fft));
        fft_lp = fft_lp(1:N_fft/2+1);
        h_lp = plot(f, fft_lp, 'b-', 'LineWidth', 1.2);
    end

    % Filtered FFT
    if has_filtered
        accel_filt_interp = interp1(t_filt, accel_filt(:,j), t_raw, 'linear', 'extrap');
        fft_filt = abs(fft(accel_filt_interp, N_fft));
        fft_filt = fft_filt(1:N_fft/2+1);
        h_filt = plot(f, fft_filt, 'r-', 'LineWidth', 1.2);
    end

    hold off;
    grid on;
    xlim([0 50]);
    ylabel(sprintf('|A_%s(f)|', axis_labels{j}));

    if j == 1
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_lowpass
            leg_h(end+1) = h_lp;
            leg_str{end+1} = 'Lowpass';
        end
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Frekvence [Hz]');
    end
end
sgtitle('FFT Spektrum - Akcelerace');

%% ====================================================================
%% OKNO 4: FFT Gyroskop
%% ====================================================================
figure('Name', '4: FFT Gyroskop', 'NumberTitle', 'off');

for j = 1:3
    subplot(3,1,j);

    % RAW FFT
    fft_raw = abs(fft(gyro_raw(:,j), N_fft));
    fft_raw = fft_raw(1:N_fft/2+1);
    h_raw = plot(f, fft_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;

    % Lowpass FFT
    if has_lowpass
        gyro_lp_interp = interp1(t_lp, gyro_lp(:,j), t_raw, 'linear', 'extrap');
        fft_lp = abs(fft(gyro_lp_interp, N_fft));
        fft_lp = fft_lp(1:N_fft/2+1);
        h_lp = plot(f, fft_lp, 'b-', 'LineWidth', 1.2);
    end

    % Filtered FFT
    if has_filtered
        gyro_filt_interp = interp1(t_filt, gyro_filt(:,j), t_raw, 'linear', 'extrap');
        fft_filt = abs(fft(gyro_filt_interp, N_fft));
        fft_filt = fft_filt(1:N_fft/2+1);
        h_filt = plot(f, fft_filt, 'r-', 'LineWidth', 1.2);
    end

    hold off;
    grid on;
    xlim([0 50]);
    ylabel(sprintf('|\\Omega_%s(f)|', axis_labels{j}));

    if j == 1
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_lowpass
            leg_h(end+1) = h_lp;
            leg_str{end+1} = 'Lowpass';
        end
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Frekvence [Hz]');
    end
end
sgtitle('FFT Spektrum - Gyroskop');

%% ====================================================================
%% OKNO 5: NÁVRH VLASTNÍHO FILTRU - Porovnání s RAW a Filtered
%% ====================================================================
%
%  Zde si můžeš nastavit vlastní parametry filtru a porovnat výsledek
%  s naměřenými daty. Vstupem do filtru jsou RAW data z imu topicu.
%
%  PARAMETRY K ÚPRAVĚ:
%  -------------------

custom_fc = 1.0;             % Cutoff frekvence [Hz]
custom_filter_order = 2;     % Řád Butterworth filtru

%  -------------------

fprintf('\n========================================\n');
fprintf('NÁVRH VLASTNÍHO FILTRU\n');
fprintf('Butterworth lowpass: fc = %.2f Hz, order = %d\n', custom_fc, custom_filter_order);
fprintf('========================================\n');

% Návrh filtru
[b, a] = butter(custom_filter_order, custom_fc/(fs/2), 'low');

% Aplikace filtru na RAW data
accel_custom = zeros(size(accel_raw));
gyro_custom = zeros(size(gyro_raw));
for j = 1:3
    accel_custom(:,j) = filtfilt(b, a, accel_raw(:,j));
    gyro_custom(:,j) = filtfilt(b, a, gyro_raw(:,j));
end

% Vykreslení
figure('Name', '5: Návrh vlastního filtru', 'NumberTitle', 'off');

% Akcelerace
for j = 1:3
    subplot(3,2, (j-1)*2 + 1);

    h_raw = plot(t_raw, accel_raw(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;
    if has_filtered
        h_filt = plot(t_filt, accel_filt(:,j), 'r-', 'LineWidth', 1.0);
    end
    h_custom = plot(t_raw, accel_custom(:,j), 'g-', 'LineWidth', 1.5);
    hold off;

    grid on;
    ylabel(sprintf('a_%s [m/s^2]', axis_labels{j}));

    if j == 1
        title('Akcelerace');
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        leg_h(end+1) = h_custom;
        leg_str{end+1} = sprintf('Custom (fc=%.1fHz)', custom_fc);
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Čas [s]');
    end
end

% Gyroskop
for j = 1:3
    subplot(3,2, (j-1)*2 + 2);

    h_raw = plot(t_raw, gyro_raw(:,j), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    hold on;
    if has_filtered
        h_filt = plot(t_filt, gyro_filt(:,j), 'r-', 'LineWidth', 1.0);
    end
    h_custom = plot(t_raw, gyro_custom(:,j), 'g-', 'LineWidth', 1.5);
    hold off;

    grid on;
    ylabel(sprintf('\\omega_%s [rad/s]', axis_labels{j}));

    if j == 1
        title('Gyroskop');
        leg_h = [h_raw];
        leg_str = {'RAW'};
        if has_filtered
            leg_h(end+1) = h_filt;
            leg_str{end+1} = 'Filtered (Madwick)';
        end
        leg_h(end+1) = h_custom;
        leg_str{end+1} = sprintf('Custom (fc=%.1fHz)', custom_fc);
        legend(leg_h, leg_str, 'Location', 'best');
    end

    if j == 3
        xlabel('Čas [s]');
    end
end

sgtitle(sprintf('Návrh filtru: Butterworth fc=%.1f Hz, order=%d', custom_fc, custom_filter_order));

fprintf('\nHotovo!\n');

%% ====================================================================
%% HELPER FUNKCE
%% ====================================================================
function [t, accel, gyro] = extract_imu_data(msgs)
    N = length(msgs);
    t = zeros(N, 1);
    accel = zeros(N, 3);
    gyro = zeros(N, 3);

    for i = 1:N
        msg = msgs{i};
        t(i) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        accel(i,:) = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z];
        gyro(i,:) = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z];
    end
end
