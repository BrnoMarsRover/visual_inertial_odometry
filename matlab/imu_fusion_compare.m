%% imu_fusion_compare.m
% Skript pro porovnání VIO odometrie s IMU fusion DISABLE vs ENABLE
% Autor: Martin Kriz
%
% Struktura:
%   1. Okno: IMU Disable - X,Y,Z vs cas + IMU data se sliderem
%   2. Okno: IMU Enable - X,Y,Z vs cas + IMU data se sliderem
%   3. Okno: Srovnání obou odometrií + IMU data (z jednoho bagu)

clear all
close all

%% KONFIGURACE
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags');

% Cesty k bagum
bag_disable_path = fullfile(rosbag_base_path, "imu_test_110226/imu_disable");
bag_enable_path = fullfile(rosbag_base_path, "imu_test_110226/imu_enable");

% Topic names
vio_topic = "/f450_1/vio_isaac/visual_slam/vis/slam_odometry";
imu_topic = "/f450_1/aircraft/imu";

%% NACITANI DAT
fprintf('Nacitam IMU DISABLE bag: %s\n', bag_disable_path);
bag_disable = ros2bagreader(bag_disable_path);

fprintf('Nacitam IMU ENABLE bag: %s\n', bag_enable_path);
bag_enable = ros2bagreader(bag_enable_path);

% VIO data - disable
fprintf('Nacitam VIO data (disable)...\n');
vio_disable_msgs = readMessages(select(bag_disable, "Topic", vio_topic));
[t_vio_disable_raw, pos_vio_disable] = odomCellToPosT(vio_disable_msgs);
pos_vio_disable = pos_vio_disable - pos_vio_disable(1,:);

% IMU data - z disable bagu
fprintf('Nacitam IMU data (disable bag)...\n');
imu_msgs_disable = readMessages(select(bag_disable, "Topic", imu_topic));
[t_imu_disable_raw, accel_disable, gyro_disable] = extract_imu_data(imu_msgs_disable);

% Synchronizace casu pro disable bag - spolecny referencni cas
t0_disable = min(t_vio_disable_raw(1), t_imu_disable_raw(1));
t_vio_disable = t_vio_disable_raw - t0_disable;
t_imu_disable = t_imu_disable_raw - t0_disable;

% VIO data - enable
fprintf('Nacitam VIO data (enable)...\n');
vio_enable_msgs = readMessages(select(bag_enable, "Topic", vio_topic));
[t_vio_enable_raw, pos_vio_enable] = odomCellToPosT(vio_enable_msgs);
pos_vio_enable = pos_vio_enable - pos_vio_enable(1,:);

% IMU data - z enable bagu
fprintf('Nacitam IMU data (enable bag)...\n');
imu_msgs_enable = readMessages(select(bag_enable, "Topic", imu_topic));
[t_imu_enable_raw, accel_enable, gyro_enable] = extract_imu_data(imu_msgs_enable);

% Synchronizace casu pro enable bag - spolecny referencni cas
t0_enable = min(t_vio_enable_raw(1), t_imu_enable_raw(1));
t_vio_enable = t_vio_enable_raw - t0_enable;
t_imu_enable = t_imu_enable_raw - t0_enable;

fprintf('Data nactena!\n');
fprintf('  VIO disable: %d vzorku, %.1f s\n', length(t_vio_disable), t_vio_disable(end));
fprintf('  VIO enable:  %d vzorku, %.1f s\n', length(t_vio_enable), t_vio_enable(end));
fprintf('  IMU disable: %d vzorku, %.1f s\n', length(t_imu_disable), t_imu_disable(end));
fprintf('  IMU enable:  %d vzorku, %.1f s\n', length(t_imu_enable), t_imu_enable(end));

%% ====================================================================
%% OKNO 1: IMU DISABLE - X,Y,Z vs cas + IMU data
%% ====================================================================
fig1 = figure('Name', '1: IMU Fusion DISABLE', 'NumberTitle', 'off', ...
              'Position', [50 50 1200 900]);

% Ulozeni dat do figure pro pristup z callbacku
fig1.UserData.t_vio = t_vio_disable;
fig1.UserData.pos_vio = pos_vio_disable;
fig1.UserData.t_imu = t_imu_disable;
fig1.UserData.accel = accel_disable;
fig1.UserData.gyro = gyro_disable;

% X vs cas
subplot(5,1,1);
plot(t_vio_disable, pos_vio_disable(:,1), 'b-', 'LineWidth', 1.5);
hold on;
h1_line_x = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('X [m]');
title('IMU Fusion DISABLE - Pozice X');
fig1.UserData.h_line_x = h1_line_x;

% Y vs cas
subplot(5,1,2);
plot(t_vio_disable, pos_vio_disable(:,2), 'b-', 'LineWidth', 1.5);
hold on;
h1_line_y = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Y [m]');
title('Pozice Y');
fig1.UserData.h_line_y = h1_line_y;

% Z vs cas
subplot(5,1,3);
plot(t_vio_disable, pos_vio_disable(:,3), 'b-', 'LineWidth', 1.5);
hold on;
h1_line_z = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Z [m]');
title('Pozice Z');
fig1.UserData.h_line_z = h1_line_z;

% IMU Akcelerace
subplot(5,1,4);
plot(t_imu_disable, accel_disable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_disable, accel_disable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_disable, accel_disable(:,3), 'b-', 'LineWidth', 0.8);
h1_line_accel = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Akcel [m/s^2]');
title('IMU Akcelerace');
legend({'a_x', 'a_y', 'a_z'}, 'Location', 'eastoutside');
fig1.UserData.h_line_accel = h1_line_accel;

% IMU Gyroskop
subplot(5,1,5);
plot(t_imu_disable, gyro_disable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_disable, gyro_disable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_disable, gyro_disable(:,3), 'b-', 'LineWidth', 0.8);
h1_line_gyro = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
xlabel('Cas [s]');
ylabel('Gyro [rad/s]');
title('IMU Gyroskop');
legend({'\omega_x', '\omega_y', '\omega_z'}, 'Location', 'eastoutside');
fig1.UserData.h_line_gyro = h1_line_gyro;

% Slider pro cas
slider1 = uicontrol('Parent', fig1, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.01 0.8 0.02], ...
                    'Min', 0, 'Max', t_vio_disable(end), 'Value', 0, ...
                    'Callback', @(src, evt) update_time_marker_single(fig1, src.Value));

uicontrol('Parent', fig1, 'Style', 'text', ...
          'Units', 'normalized', ...
          'Position', [0.01 0.01 0.08 0.02], ...
          'String', 'Cas [s]:');

fig1.UserData.time_label = uicontrol('Parent', fig1, 'Style', 'text', ...
                                      'Units', 'normalized', ...
                                      'Position', [0.91 0.01 0.08 0.02], ...
                                      'String', '0.0 s');

%% ====================================================================
%% OKNO 2: IMU ENABLE - X,Y,Z vs cas + IMU data
%% ====================================================================
fig2 = figure('Name', '2: IMU Fusion ENABLE', 'NumberTitle', 'off', ...
              'Position', [100 100 1200 900]);

% Ulozeni dat do figure
fig2.UserData.t_vio = t_vio_enable;
fig2.UserData.pos_vio = pos_vio_enable;
fig2.UserData.t_imu = t_imu_enable;
fig2.UserData.accel = accel_enable;
fig2.UserData.gyro = gyro_enable;

% X vs cas
subplot(5,1,1);
plot(t_vio_enable, pos_vio_enable(:,1), 'Color', [0 0.6 0], 'LineWidth', 1.5);
hold on;
h2_line_x = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('X [m]');
title('IMU Fusion ENABLE - Pozice X');
fig2.UserData.h_line_x = h2_line_x;

% Y vs cas
subplot(5,1,2);
plot(t_vio_enable, pos_vio_enable(:,2), 'Color', [0 0.6 0], 'LineWidth', 1.5);
hold on;
h2_line_y = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Y [m]');
title('Pozice Y');
fig2.UserData.h_line_y = h2_line_y;

% Z vs cas
subplot(5,1,3);
plot(t_vio_enable, pos_vio_enable(:,3), 'Color', [0 0.6 0], 'LineWidth', 1.5);
hold on;
h2_line_z = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Z [m]');
title('Pozice Z');
fig2.UserData.h_line_z = h2_line_z;

% IMU Akcelerace
subplot(5,1,4);
plot(t_imu_enable, accel_enable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_enable, accel_enable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_enable, accel_enable(:,3), 'b-', 'LineWidth', 0.8);
h2_line_accel = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Akcel [m/s^2]');
title('IMU Akcelerace');
legend({'a_x', 'a_y', 'a_z'}, 'Location', 'eastoutside');
fig2.UserData.h_line_accel = h2_line_accel;

% IMU Gyroskop
subplot(5,1,5);
plot(t_imu_enable, gyro_enable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_enable, gyro_enable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_enable, gyro_enable(:,3), 'b-', 'LineWidth', 0.8);
h2_line_gyro = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
xlabel('Cas [s]');
ylabel('Gyro [rad/s]');
title('IMU Gyroskop');
legend({'\omega_x', '\omega_y', '\omega_z'}, 'Location', 'eastoutside');
fig2.UserData.h_line_gyro = h2_line_gyro;

% Slider pro cas
slider2 = uicontrol('Parent', fig2, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.01 0.8 0.02], ...
                    'Min', 0, 'Max', t_vio_enable(end), 'Value', 0, ...
                    'Callback', @(src, evt) update_time_marker_single(fig2, src.Value));

uicontrol('Parent', fig2, 'Style', 'text', ...
          'Units', 'normalized', ...
          'Position', [0.01 0.01 0.08 0.02], ...
          'String', 'Cas [s]:');

fig2.UserData.time_label = uicontrol('Parent', fig2, 'Style', 'text', ...
                                      'Units', 'normalized', ...
                                      'Position', [0.91 0.01 0.08 0.02], ...
                                      'String', '0.0 s');

%% ====================================================================
%% OKNO 3: SROVNANI - Obe odometrie + IMU data
%% ====================================================================
fig3 = figure('Name', '3: Srovnani IMU Disable vs Enable', 'NumberTitle', 'off', ...
              'Position', [150 150 1200 900]);

% Spolecny casovy rozsah
t_max = min(t_vio_disable(end), t_vio_enable(end));

% Ulozeni dat do figure (IMU z disable bagu - jsou stejne)
fig3.UserData.t_vio_disable = t_vio_disable;
fig3.UserData.pos_vio_disable = pos_vio_disable;
fig3.UserData.t_vio_enable = t_vio_enable;
fig3.UserData.pos_vio_enable = pos_vio_enable;
fig3.UserData.t_imu = t_imu_disable;
fig3.UserData.accel = accel_disable;
fig3.UserData.gyro = gyro_disable;
fig3.UserData.t_max = t_max;

% X vs cas - srovnani
subplot(5,1,1);
plot(t_vio_disable, pos_vio_disable(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_vio_enable, pos_vio_enable(:,1), 'Color', [0 0.6 0], 'LineWidth', 1.5);
h3_line_x = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('X [m]');
title('Srovnani - Pozice X');
legend({'Disable', 'Enable'}, 'Location', 'eastoutside');
fig3.UserData.h_line_x = h3_line_x;

% Y vs cas - srovnani
subplot(5,1,2);
plot(t_vio_disable, pos_vio_disable(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_vio_enable, pos_vio_enable(:,2), 'Color', [0 0.6 0], 'LineWidth', 1.5);
h3_line_y = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Y [m]');
title('Pozice Y');
legend({'Disable', 'Enable'}, 'Location', 'eastoutside');
fig3.UserData.h_line_y = h3_line_y;

% Z vs cas - srovnani
subplot(5,1,3);
plot(t_vio_disable, pos_vio_disable(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_vio_enable, pos_vio_enable(:,3), 'Color', [0 0.6 0], 'LineWidth', 1.5);
h3_line_z = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Z [m]');
title('Pozice Z');
legend({'Disable', 'Enable'}, 'Location', 'eastoutside');
fig3.UserData.h_line_z = h3_line_z;

% IMU Akcelerace (z disable bagu)
subplot(5,1,4);
plot(t_imu_disable, accel_disable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_disable, accel_disable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_disable, accel_disable(:,3), 'b-', 'LineWidth', 0.8);
h3_line_accel = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
ylabel('Akcel [m/s^2]');
title('IMU Akcelerace');
legend({'a_x', 'a_y', 'a_z'}, 'Location', 'eastoutside');
fig3.UserData.h_line_accel = h3_line_accel;

% IMU Gyroskop (z disable bagu)
subplot(5,1,5);
plot(t_imu_disable, gyro_disable(:,1), 'r-', 'LineWidth', 0.8); hold on;
plot(t_imu_disable, gyro_disable(:,2), 'g-', 'LineWidth', 0.8);
plot(t_imu_disable, gyro_disable(:,3), 'b-', 'LineWidth', 0.8);
h3_line_gyro = xline(0, 'r-', 'LineWidth', 2);
hold off;
grid on;
xlabel('Cas [s]');
ylabel('Gyro [rad/s]');
title('IMU Gyroskop');
legend({'\omega_x', '\omega_y', '\omega_z'}, 'Location', 'eastoutside');
fig3.UserData.h_line_gyro = h3_line_gyro;

% Slider pro cas
slider3 = uicontrol('Parent', fig3, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.01 0.8 0.02], ...
                    'Min', 0, 'Max', t_max, 'Value', 0, ...
                    'Callback', @(src, evt) update_time_marker_compare(fig3, src.Value));

uicontrol('Parent', fig3, 'Style', 'text', ...
          'Units', 'normalized', ...
          'Position', [0.01 0.01 0.08 0.02], ...
          'String', 'Cas [s]:');

fig3.UserData.time_label = uicontrol('Parent', fig3, 'Style', 'text', ...
                                      'Units', 'normalized', ...
                                      'Position', [0.91 0.01 0.08 0.02], ...
                                      'String', '0.0 s');

%% VYPIS METRIK
fprintf('\n========================================\n');
fprintf('METRIKY\n');
fprintf('========================================\n');

% Celkova ujeta vzdalenost
dist_disable = sum(vecnorm(diff(pos_vio_disable), 2, 2));
dist_enable = sum(vecnorm(diff(pos_vio_enable), 2, 2));
fprintf('Celkova vzdalenost (disable): %.2f m\n', dist_disable);
fprintf('Celkova vzdalenost (enable):  %.2f m\n', dist_enable);

% Koncova pozice
fprintf('Koncova pozice (disable): [%.2f, %.2f, %.2f] m\n', ...
        pos_vio_disable(end,1), pos_vio_disable(end,2), pos_vio_disable(end,3));
fprintf('Koncova pozice (enable):  [%.2f, %.2f, %.2f] m\n', ...
        pos_vio_enable(end,1), pos_vio_enable(end,2), pos_vio_enable(end,3));

fprintf('========================================\n');
fprintf('Hotovo! Pouzij slidery pro prochazeni dat v case.\n');

%% ====================================================================
%% CALLBACK FUNKCE
%% ====================================================================
function update_time_marker_single(fig, t_current)
    % Aktualizace markeru pro okno 1 nebo 2
    data = fig.UserData;

    % Aktualizuj casove cary
    set(data.h_line_x, 'Value', t_current);
    set(data.h_line_y, 'Value', t_current);
    set(data.h_line_z, 'Value', t_current);
    set(data.h_line_accel, 'Value', t_current);
    set(data.h_line_gyro, 'Value', t_current);

    % Aktualizuj label
    set(data.time_label, 'String', sprintf('%.1f s', t_current));
end

function update_time_marker_compare(fig, t_current)
    % Aktualizace markeru pro okno 3 (srovnani)
    data = fig.UserData;

    % Aktualizuj casove cary
    set(data.h_line_x, 'Value', t_current);
    set(data.h_line_y, 'Value', t_current);
    set(data.h_line_z, 'Value', t_current);
    set(data.h_line_accel, 'Value', t_current);
    set(data.h_line_gyro, 'Value', t_current);

    % Aktualizuj label
    set(data.time_label, 'String', sprintf('%.1f s', t_current));
end

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
