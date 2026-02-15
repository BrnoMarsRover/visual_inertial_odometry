%% tracking_odometry_compare.m
% Porovnání tracking/odometry z dvou letů: IMU disabled vs IMU enabled (SLAM disabled)
% Časové zarovnání pomocí cross-korelace GPS odometrie (stejný záznam v obou bagách)
% Zobrazuje průběh x, y, z v čase - každá osa ve vlastním subplotu

clear all
close all

%% KONFIGURACE
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags', 'winter_flight1');

bag_imu_off_path = fullfile(rosbag_base_path, "imu_disabled_slam_disabled_00");
bag_imu_on_path  = fullfile(rosbag_base_path, "imu_enabled_slam_disabled_00");

odom_topic = "/f450_1/vio_isaac/visual_slam/tracking/odometry";
gps_topic  = "/f450_1/aircraft/gps_odometry";

%% NAČTENÍ DAT
fprintf('Načítám bag: IMU disabled...\n');
bag_off = ros2bagreader(bag_imu_off_path);
msgs_off = readMessages(select(bag_off, "Topic", odom_topic));
[t_off_raw, pos_off] = odomCellToPosT(msgs_off);

gps_msgs_off = readMessages(select(bag_off, "Topic", gps_topic));
[t_gps_off_raw, pos_gps_off] = odomCellToPosT(gps_msgs_off);

fprintf('Načítám bag: IMU enabled...\n');
bag_on = ros2bagreader(bag_imu_on_path);
msgs_on = readMessages(select(bag_on, "Topic", odom_topic));
[t_on_raw, pos_on] = odomCellToPosT(msgs_on);

gps_msgs_on = readMessages(select(bag_on, "Topic", gps_topic));
[t_gps_on_raw, pos_gps_on] = odomCellToPosT(gps_msgs_on);

%% ČASOVÉ ZAROVNÁNÍ POMOCÍ GPS CROSS-KORELACE
% GPS data jsou ze stejného záznamu, takže pozice jsou identické,
% jen posunuté v čase. Cross-korelací najdeme offset.

% Normalizace GPS časů na 0
t_gps_off = t_gps_off_raw - t_gps_off_raw(1);
t_gps_on  = t_gps_on_raw  - t_gps_on_raw(1);

% Interpolace GPS na společný uniformní čas (pro cross-korelaci)
dt = 0.02;  % 50 Hz
t_max = min(t_gps_off(end), t_gps_on(end));
t_uniform = (0:dt:t_max)';

gps_off_interp = interp1(t_gps_off, pos_gps_off(:,1), t_uniform, 'linear', NaN);
gps_on_interp  = interp1(t_gps_on,  pos_gps_on(:,1),  t_uniform, 'linear', NaN);

% Nahrazení NaN nulami pro xcorr
gps_off_interp(isnan(gps_off_interp)) = 0;
gps_on_interp(isnan(gps_on_interp))   = 0;

[xc, lags] = xcorr(gps_off_interp - mean(gps_off_interp), ...
                    gps_on_interp  - mean(gps_on_interp));
[~, idx] = max(xc);
lag_samples = lags(idx);
time_offset = lag_samples * dt;  % offset v sekundách (off vůči on)

fprintf('\n=== ČASOVÉ ZAROVNÁNÍ ===\n');
fprintf('GPS cross-korelace offset: %.3f s\n', time_offset);
fprintf('(IMU_off začíná o %.3f s %s než IMU_on)\n', ...
    abs(time_offset), ternary(time_offset > 0, 'později', 'dříve'));

%% APLIKACE OFFSETU A NORMALIZACE ČASU
% Referenční čas = čas GPS v bagu IMU_off (t=0 = start GPS v IMU_off bagu)
% VIO časy normalizujeme relativně ke startu GPS v daném bagu
t_off = t_off_raw - t_gps_off_raw(1);            % VIO_off relativně ke GPS_off startu
t_on  = t_on_raw  - t_gps_on_raw(1) + time_offset; % VIO_on posunuté o offset

%% VÝPOČET ODCHYLEK OD POČÁTKU
fprintf('\n=== ODCHYLKA OD POČÁTKU (koncový bod vs start) ===\n');

for i = 1:2
    if i == 1
        label = 'IMU disabled';
        pos = pos_off;
    else
        label = 'IMU enabled';
        pos = pos_on;
    end

    drift = pos(end,:) - pos(1,:);
    drift_abs = norm(drift);

    fprintf('\n--- %s ---\n', label);
    fprintf('Absolutní odchylka:  %.4f m\n', drift_abs);
    fprintf('Odchylka X:          %.4f m\n', drift(1));
    fprintf('Odchylka Y:          %.4f m\n', drift(2));
    fprintf('Odchylka Z:          %.4f m\n', drift(3));
end

%% VYKRESLENÍ
figure('Name', 'Tracking Odometry: IMU off vs IMU on', 'NumberTitle', 'off');

axis_labels = {'X', 'Y', 'Z'};

for ax = 1:3
    subplot(3, 1, ax);
    plot(t_off, pos_off(:, ax), 'b-', 'LineWidth', 1.2); hold on;
    plot(t_on,  pos_on(:, ax),  'r-', 'LineWidth', 1.2); hold off;
    grid on;
    xlabel('Čas [s]');
    ylabel(sprintf('%s [m]', axis_labels{ax}));
    title(sprintf('Tracking odometry - osa %s', axis_labels{ax}));
    legend('IMU disabled', 'IMU enabled', 'Location', 'best');
end

sgtitle('Porovnání tracking/odometry (SLAM disabled)');

fprintf('\nHotovo!\n');

%% POMOCNÁ FUNKCE
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
