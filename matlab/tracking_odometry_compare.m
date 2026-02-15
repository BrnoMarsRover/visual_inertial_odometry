%% tracking_odometry_compare.m
% Comparison of tracking/odometry from two flights: IMU disabled vs IMU enabled (SLAM disabled)
% Time alignment using cross-correlation of GPS odometry (same recording in both bags)
% Displays x, y, z over time - each axis in its own subplot

clear all
close all

%% CONFIGURATION
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags', 'winter_flight1');

bag_imu_off_path = fullfile(rosbag_base_path, "imu_disabled_slam_disabled_00");
bag_imu_on_path  = fullfile(rosbag_base_path, "imu_enabled_slam_disabled_00");

odom_topic = "/f450_1/vio_isaac/visual_slam/tracking/odometry";
gps_topic  = "/f450_1/aircraft/gps_odometry";

%% LOAD DATA
fprintf('Loading bag: IMU disabled...\n');
bag_off = ros2bagreader(bag_imu_off_path);
msgs_off = readMessages(select(bag_off, "Topic", odom_topic));
[t_off_raw, pos_off] = odomCellToPosT(msgs_off);

gps_msgs_off = readMessages(select(bag_off, "Topic", gps_topic));
[t_gps_off_raw, pos_gps_off] = odomCellToPosT(gps_msgs_off);

fprintf('Loading bag: IMU enabled...\n');
bag_on = ros2bagreader(bag_imu_on_path);
msgs_on = readMessages(select(bag_on, "Topic", odom_topic));
[t_on_raw, pos_on] = odomCellToPosT(msgs_on);

gps_msgs_on = readMessages(select(bag_on, "Topic", gps_topic));
[t_gps_on_raw, pos_gps_on] = odomCellToPosT(gps_msgs_on);

%% TIME ALIGNMENT USING GPS CROSS-CORRELATION
% GPS data come from the same recording, so positions are identical,
% only shifted in time. Cross-correlation finds the offset.

% Normalize GPS times to 0
t_gps_off = t_gps_off_raw - t_gps_off_raw(1);
t_gps_on  = t_gps_on_raw  - t_gps_on_raw(1);

% Interpolate GPS onto a common uniform time base (for cross-correlation)
dt = 0.02;  % 50 Hz
t_max = min(t_gps_off(end), t_gps_on(end));
t_uniform = (0:dt:t_max)';

gps_off_interp = interp1(t_gps_off, pos_gps_off(:,1), t_uniform, 'linear', NaN);
gps_on_interp  = interp1(t_gps_on,  pos_gps_on(:,1),  t_uniform, 'linear', NaN);

% Replace NaN with zeros for xcorr
gps_off_interp(isnan(gps_off_interp)) = 0;
gps_on_interp(isnan(gps_on_interp))   = 0;

[xc, lags] = xcorr(gps_off_interp - mean(gps_off_interp), ...
                    gps_on_interp  - mean(gps_on_interp));
[~, idx] = max(xc);
lag_samples = lags(idx);
time_offset = lag_samples * dt;  % offset in seconds (off vs on)

fprintf('\n=== TIME ALIGNMENT ===\n');
fprintf('GPS cross-correlation offset: %.3f s\n', time_offset);
fprintf('(IMU_off starts %.3f s %s than IMU_on)\n', ...
    abs(time_offset), ternary(time_offset > 0, 'later', 'earlier'));

%% APPLY OFFSET AND NORMALIZE TIME
% Reference time = GPS time in IMU_off bag (t=0 = GPS start in IMU_off bag)
% VIO times normalized relative to GPS start in the respective bag
t_off = t_off_raw - t_gps_off_raw(1);            % VIO_off relative to GPS_off start
t_on  = t_on_raw  - t_gps_on_raw(1) + time_offset; % VIO_on shifted by offset

%% DRIFT FROM ORIGIN
fprintf('\n=== DRIFT FROM ORIGIN (end position vs start) ===\n');

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
    fprintf('Absolute drift:  %.4f m\n', drift_abs);
    fprintf('Drift X:         %.4f m\n', drift(1));
    fprintf('Drift Y:         %.4f m\n', drift(2));
    fprintf('Drift Z:         %.4f m\n', drift(3));
end

%% PLOT - IMU OFF vs IMU ON
figure('Name', 'Tracking Odometry: IMU off vs IMU on', 'NumberTitle', 'off');

axis_labels = {'X', 'Y', 'Z'};

for ax = 1:3
    subplot(3, 1, ax);
    plot(t_off, pos_off(:, ax), 'b-', 'LineWidth', 1.2); hold on;
    plot(t_on,  pos_on(:, ax),  'r-', 'LineWidth', 1.2); hold off;
    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('%s [m]', axis_labels{ax}));
    title(sprintf('Tracking odometry - %s axis', axis_labels{ax}));
    legend('IMU disabled', 'IMU enabled', 'Location', 'best');
end

sgtitle('Tracking odometry comparison (SLAM disabled)');

fprintf('\nDone!\n');

%% =====================================================================
%  IMU NOISE PARAMETER COMPARISON (bags 01-04)
%  Same flight, different IMU noise parameters
%  =====================================================================

%% CONFIGURATION - IMU NOISE COMPARISON
bag_noise_names = { ...
    'imu_enabled_slam_disabled_01', ...
    'imu_enabled_slam_disabled_02', ...
    'imu_enabled_slam_disabled_03', ...
    'imu_enabled_slam_disabled_04'  ...
};

noise_labels = {'01', '02', '03', '04'};

n_noise = numel(bag_noise_names);

%% LOAD REFERENCE GPS (from first bag) FOR ALIGNMENT
fprintf('\n=== IMU NOISE PARAMETER COMPARISON ===\n');
fprintf('Loading reference GPS from %s...\n', bag_noise_names{1});
bag_ref = ros2bagreader(fullfile(rosbag_base_path, bag_noise_names{1}));
gps_msgs_ref = readMessages(select(bag_ref, "Topic", gps_topic));
[t_gps_ref_raw, pos_gps_ref] = odomCellToPosT(gps_msgs_ref);
t_gps_ref = t_gps_ref_raw - t_gps_ref_raw(1);

%% LOAD AND ALIGN ALL BAGS
t_noise   = cell(n_noise, 1);
pos_noise = cell(n_noise, 1);

for i = 1:n_noise
    fprintf('Loading bag: %s...\n', bag_noise_names{i});
    bag = ros2bagreader(fullfile(rosbag_base_path, bag_noise_names{i}));

    % VIO odometry
    msgs = readMessages(select(bag, "Topic", odom_topic));
    [t_raw, pos] = odomCellToPosT(msgs);

    % GPS for alignment
    gps_msgs = readMessages(select(bag, "Topic", gps_topic));
    [t_gps_raw, pos_gps] = odomCellToPosT(gps_msgs);
    t_gps = t_gps_raw - t_gps_raw(1);

    % GPS cross-correlation for time offset vs reference
    dt_n = 0.02;
    t_max_n = min(t_gps_ref(end), t_gps(end));
    t_uniform_n = (0:dt_n:t_max_n)';

    gps_ref_interp = interp1(t_gps_ref, pos_gps_ref(:,1), t_uniform_n, 'linear', NaN);
    gps_interp     = interp1(t_gps,     pos_gps(:,1),     t_uniform_n, 'linear', NaN);

    gps_ref_interp(isnan(gps_ref_interp)) = 0;
    gps_interp(isnan(gps_interp))         = 0;

    [xc_n, lags_n] = xcorr(gps_ref_interp - mean(gps_ref_interp), ...
                            gps_interp     - mean(gps_interp));
    [~, idx_n] = max(xc_n);
    time_offset_n = lags_n(idx_n) * dt_n;

    fprintf('  GPS offset: %.3f s\n', time_offset_n);

    t_noise{i}   = t_raw - t_gps_raw(1) + time_offset_n;
    pos_noise{i} = pos;
end

%% DRIFT - IMU NOISE
fprintf('\n=== DRIFT FROM ORIGIN - IMU NOISE PARAMETERS ===\n');
fprintf('%-8s  %10s  %10s  %10s  %10s\n', 'Bag', 'Abs [m]', 'X [m]', 'Y [m]', 'Z [m]');
fprintf('%s\n', repmat('-', 1, 52));

for i = 1:n_noise
    drift = pos_noise{i}(end,:) - pos_noise{i}(1,:);
    drift_abs = norm(drift);
    fprintf('%-8s  %10.4f  %10.4f  %10.4f  %10.4f\n', ...
        noise_labels{i}, drift_abs, drift(1), drift(2), drift(3));
end

%% ALIGN IMU_OFF TO NOISE REFERENCE (for plotting in the same figure)
dt_ref = 0.02;
t_max_ref = min(t_gps_ref(end), (t_gps_off_raw(end) - t_gps_off_raw(1)));
t_uniform_ref = (0:dt_ref:t_max_ref)';

gps_ref_interp2 = interp1(t_gps_ref, pos_gps_ref(:,1), t_uniform_ref, 'linear', NaN);
gps_off_interp2 = interp1(t_gps_off, pos_gps_off(:,1), t_uniform_ref, 'linear', NaN);

gps_ref_interp2(isnan(gps_ref_interp2)) = 0;
gps_off_interp2(isnan(gps_off_interp2)) = 0;

[xc_off, lags_off] = xcorr(gps_ref_interp2 - mean(gps_ref_interp2), ...
                            gps_off_interp2 - mean(gps_off_interp2));
[~, idx_off] = max(xc_off);
time_offset_off = lags_off(idx_off) * dt_ref;

t_off_aligned = t_off_raw - t_gps_off_raw(1) + time_offset_off;

%% PLOT - IMU NOISE COMPARISON
noise_styles = {'-', '-.', ':', '-'};
noise_colors = {[0 0.45 0.74], [0.85 0.33 0.1], [0.47 0.67 0.19], [0.64 0.08 0.18]};
noise_widths = [1.5, 1.5, 2.0, 1.5];

fig2 = figure('Name', 'Tracking Odometry - IMU noise comparison', 'NumberTitle', 'off');
set(fig2, 'Position', [100 100 1200 800]);

for ax = 1:3
    subplot(3, 1, ax);
    % IMU disabled (dashed, black)
    plot(t_off_aligned, pos_off(:, ax), 'k--', 'LineWidth', 1.8);
    hold on;
    % Noise bags 01-04
    for i = 1:n_noise
        plot(t_noise{i}, pos_noise{i}(:, ax), noise_styles{i}, ...
            'Color', noise_colors{i}, 'LineWidth', noise_widths(i));
    end
    hold off;
    grid on;
    set(gca, 'FontSize', 11);
    xlabel('Time [s]');
    ylabel(sprintf('%s [m]', axis_labels{ax}));
    title(sprintf('Tracking odometry - %s axis', axis_labels{ax}));
    legend(['00 (IMU off)', noise_labels], 'Location', 'best', 'FontSize', 10);
end

sgtitle('IMU noise parameter comparison (SLAM disabled)', 'FontSize', 14);

fprintf('\nAll done!\n');

%% HELPER FUNCTION
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
