%% optitrack_odometry_compare.m
% Comparison of VIO tracking odometry vs OptiTrack ground truth (/mocap/husky_1)
% OptiTrack is the same continuous recording streamed into all bags.
% Cross-correlation of OptiTrack data is used to align bags in time
% (analogous to GPS cross-correlation in tracking_odometry_compare.m).
%
% Section 1: SLAM enabled (IMU off) vs IMU enabled (SLAM off) - bags 00 vs 01
% Section 2: IMU noise parameter comparison - bags 01-04
%
% IMU noise parameters from docs/imu_fusion.md:
%   01 - RealSense default: gyro_nd=2.44e-4, gyro_rw=1.94e-5, accel_nd=1.86e-3, accel_rw=3e-3
%   02 - 2x worse:          gyro_nd=5e-4,    gyro_rw=4e-5,    accel_nd=4e-3,    accel_rw=6e-3
%   03 - 4x worse:          gyro_nd=1e-3,    gyro_rw=8e-5,    accel_nd=8e-3,    accel_rw=1.2e-2
%   04 - 8x worse:          gyro_nd=2e-3,    gyro_rw=1.6e-4,  accel_nd=1.6e-2,  accel_rw=2.4e-2

clear all
close all

%% CONFIGURATION
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags', 'optitrack');

odom_topic      = "/f450_1/vio_isaac/visual_slam/tracking/odometry";
optitrack_topic = "/mocap/husky_1";

bag_names = { ...
    'imu_disabled_slam_enabled_00', ...
    'imu_enabled_slam_disabled_01', ...
    'imu_enabled_slam_disabled_02', ...
    'imu_enabled_slam_disabled_03', ...
    'imu_enabled_slam_disabled_04'  ...
};
n_bags = numel(bag_names);

REF = 2;   % bag 01 is the time-alignment reference
dt  = 0.02;  % 50 Hz interpolation grid for cross-correlation

axis_labels = {'X', 'Y', 'Z'};

%% LOAD ALL BAGS
fprintf('=== LOADING BAGS ===\n');

t_vio_raw = cell(n_bags, 1);
pos_vio   = cell(n_bags, 1);
t_opt_raw = cell(n_bags, 1);
pos_opt   = cell(n_bags, 1);

for i = 1:n_bags
    fprintf('Loading %s...\n', bag_names{i});
    bag = ros2bagreader(fullfile(rosbag_base_path, bag_names{i}));

    [t_vio_raw{i}, pos_vio{i}] = odomCellToPosT(readMessages(select(bag, "Topic", odom_topic)));
    [t_opt_raw{i}, pos_opt{i}] = odomCellToPosT(readMessages(select(bag, "Topic", optitrack_topic)));

    fprintf('  VIO: %d msgs | OptiTrack: %d msgs\n', numel(t_vio_raw{i}), numel(t_opt_raw{i}));
end

%% TIME ALIGNMENT USING OPTITRACK CROSS-CORRELATION
% OptiTrack is the same continuous recording in all bags, started at different times.
% Cross-correlation finds the time offset of each bag relative to the reference (bag 01).

fprintf('\n=== OPTITRACK CROSS-CORRELATION (reference: %s) ===\n', bag_names{REF});

% Normalize OptiTrack time within each bag (t=0 at bag's own OptiTrack start)
t_opt_loc = cell(n_bags, 1);
for i = 1:n_bags
    t_opt_loc{i} = t_opt_raw{i} - t_opt_raw{i}(1);
end

t_ref_loc = t_opt_loc{REF};

% Cross-correlate each bag's OptiTrack vs reference
time_offsets = zeros(n_bags, 1);

for i = 1:n_bags
    if i == REF
        fprintf('  %s: reference (0.000 s)\n', bag_names{i});
        continue;
    end

    t_max     = min(t_ref_loc(end), t_opt_loc{i}(end));
    t_uniform = (0:dt:t_max)';

    ref_interp = interp1(t_ref_loc,    pos_opt{REF}(:,1), t_uniform, 'linear', NaN);
    bag_interp = interp1(t_opt_loc{i}, pos_opt{i}(:,1),   t_uniform, 'linear', NaN);

    ref_interp(isnan(ref_interp)) = 0;
    bag_interp(isnan(bag_interp)) = 0;

    [xc, lags] = xcorr(ref_interp - mean(ref_interp), bag_interp - mean(bag_interp));
    [~, idx]   = max(xc);
    time_offsets(i) = lags(idx) * dt;

    fprintf('  %s: offset = %.3f s\n', bag_names{i}, time_offsets(i));
end

%% APPLY OFFSETS - align all bags to the REF OptiTrack time axis
% t=0 corresponds to bag REF's OptiTrack start
t_vio_al = cell(n_bags, 1);
t_opt_al = cell(n_bags, 1);

for i = 1:n_bags
    t_vio_al{i} = (t_vio_raw{i} - t_opt_raw{i}(1)) + time_offsets(i);
    t_opt_al{i} = (t_opt_raw{i} - t_opt_raw{i}(1)) + time_offsets(i);
end

%% NORMALIZE POSITIONS
% VIO: zeroed at its own first sample.
% OptiTrack: zeroed at the moment VIO tracking starts (interpolated).

pos_vio_n = cell(n_bags, 1);
pos_opt_n = cell(n_bags, 1);

for i = 1:n_bags
    t_vio_start = t_vio_al{i}(1);

    if t_vio_start >= t_opt_al{i}(1) && t_vio_start <= t_opt_al{i}(end)
        opt_at_vio0 = interp1(t_opt_al{i}, pos_opt{i}, t_vio_start, 'linear');
    else
        opt_at_vio0 = pos_opt{i}(1,:);
    end

    pos_vio_n{i} = pos_vio{i} - pos_vio{i}(1,:);
    pos_opt_n{i} = pos_opt{i} - opt_at_vio0;
end

%% RMSE: VIO vs OptiTrack (per bag)
fprintf('\n=== RMSE: VIO vs OptiTrack ===\n');
fprintf('%-32s  %8s  %8s  %8s  %8s\n', 'Bag', 'X [m]', 'Y [m]', 'Z [m]', '3D [m]');
fprintf('%s\n', repmat('-', 1, 68));

rmse_all = zeros(n_bags, 4);
for i = 1:n_bags
    [rx, ry, rz, r3d] = computeRmse(t_vio_al{i}, pos_vio_n{i}, t_opt_al{i}, pos_opt_n{i});
    rmse_all(i,:) = [rx, ry, rz, r3d];
    fprintf('%-32s  %8.4f  %8.4f  %8.4f  %8.4f\n', bag_names{i}, rx, ry, rz, r3d);
end

%% DRIFT FROM ORIGIN (VIO)
fprintf('\n=== DRIFT FROM ORIGIN (VIO) ===\n');
fprintf('%-32s  %10s  %10s  %10s  %10s\n', 'Bag', 'Abs [m]', 'X [m]', 'Y [m]', 'Z [m]');
fprintf('%s\n', repmat('-', 1, 72));

for i = 1:n_bags
    d = pos_vio_n{i}(end,:) - pos_vio_n{i}(1,:);
    fprintf('%-32s  %10.4f  %10.4f  %10.4f  %10.4f\n', bag_names{i}, norm(d), d(1), d(2), d(3));
end

%% =====================================================================
%  SECTION 1: SLAM on (IMU off) vs IMU on (SLAM off) - bags 00 vs 01
%  =====================================================================

fprintf('\n=== SECTION 1: SLAM on vs IMU on ===\n');
fprintf('  Offset bag_00 vs bag_01: %.3f s\n', time_offsets(1));

fig1 = figure('Name', 'VIO vs OptiTrack: SLAM on vs IMU on', 'NumberTitle', 'off');
set(fig1, 'Position', [100 100 1200 800]);

for ax = 1:3
    subplot(3, 1, ax);
    plot(t_vio_al{1}, pos_vio_n{1}(:, ax), 'b-',  'LineWidth', 1.5); hold on;
    plot(t_opt_al{1}, pos_opt_n{1}(:, ax), 'b--', 'LineWidth', 1.2);
    plot(t_vio_al{2}, pos_vio_n{2}(:, ax), 'r-',  'LineWidth', 1.5);
    plot(t_opt_al{2}, pos_opt_n{2}(:, ax), 'r--', 'LineWidth', 1.2);
    hold off;
    grid on;
    set(gca, 'FontSize', 11);
    xlabel('Time [s]');
    ylabel(sprintf('%s [m]', axis_labels{ax}));
    title(sprintf('Tracking odometry - %s axis', axis_labels{ax}));
    legend('00 VIO (SLAM on)', '00 OptiTrack', '01 VIO (IMU on)', '01 OptiTrack', 'Location', 'best');
end

sgtitle(sprintf('VIO vs OptiTrack: SLAM on (IMU off) vs IMU on (SLAM off)\nRMSE 3D: %.4f m  /  %.4f m', ...
    rmse_all(1,4), rmse_all(2,4)), 'FontSize', 13);

%% =====================================================================
%  SECTION 2: IMU NOISE PARAMETER COMPARISON (bags 01-04, without bag 00)
%  =====================================================================

fprintf('\n=== SECTION 2: IMU NOISE PARAMETER COMPARISON ===\n');
fprintf('%-6s  %10s  %10s  %10s  %10s\n', 'Bag', 'Abs [m]', 'X [m]', 'Y [m]', 'Z [m]');
fprintf('%s\n', repmat('-', 1, 52));
for i = 2:n_bags
    d = pos_vio_n{i}(end,:) - pos_vio_n{i}(1,:);
    fprintf('  %02d    %10.4f  %10.4f  %10.4f  %10.4f\n', i-1, norm(d), d(1), d(2), d(3));
end

% Bags 01-04: indices 2-5
noise_idx    = 2:5;
noise_labels = {'01 (default)', '02 (2x)', '03 (4x)', '04 (8x)'};
noise_colors = { ...
    [0.00  0.45  0.74], ...   % blue
    [0.85  0.33  0.10], ...   % orange-red
    [0.18  0.72  0.18], ...   % green
    [0.62  0.15  0.69]  ...   % purple
};

fig2 = figure('Name', 'Tracking Odometry - IMU noise comparison vs OptiTrack', 'NumberTitle', 'off');
set(fig2, 'Position', [100 100 1200 800]);

for ax = 1:3
    subplot(3, 1, ax);

    % OptiTrack from reference bag (bag 01) as ground truth - thick black
    plot(t_opt_al{REF}, pos_opt_n{REF}(:, ax), 'k-', 'LineWidth', 2.5);
    hold on;

    % VIO for bags 01-04
    for k = 1:numel(noise_idx)
        i = noise_idx(k);
        plot(t_vio_al{i}, pos_vio_n{i}(:, ax), '-', ...
            'Color', noise_colors{k}, 'LineWidth', 2.0);
    end
    hold off;

    grid on;
    set(gca, 'FontSize', 11);
    xlabel('Time [s]');
    ylabel(sprintf('%s [m]', axis_labels{ax}));
    title(sprintf('Tracking odometry - %s axis', axis_labels{ax}));
    legend(['OptiTrack (ref)', noise_labels], 'Location', 'best', 'FontSize', 10);
end

sgtitle('IMU noise parameter comparison vs OptiTrack (SLAM disabled)', 'FontSize', 14);

%% PLOT - RMSE bar chart (all 5 bags)
all_labels = {'00 (SLAM on)', '01 (default)', '02 (2x)', '03 (4x)', '04 (8x)'};

fig3 = figure('Name', 'RMSE comparison - all bags', 'NumberTitle', 'off');
set(fig3, 'Position', [100 100 900 500]);

b = bar(rmse_all(:, 1:3), 'grouped');
b(1).FaceColor = [0.20 0.60 0.90];
b(2).FaceColor = [0.90 0.40 0.20];
b(3).FaceColor = [0.30 0.75 0.40];
set(gca, 'XTick', 1:n_bags, 'XTickLabel', all_labels, 'FontSize', 10);
xtickangle(20);
ylabel('RMSE [m]');
legend('X', 'Y', 'Z', 'Location', 'northeast');
title('VIO vs OptiTrack RMSE per axis - IMU noise parameter comparison');
grid on;

fprintf('\nAll done!\n');

%% =====================================================================
%  LOCAL FUNCTIONS
%  =====================================================================

function [rmse_x, rmse_y, rmse_z, rmse_3d] = computeRmse(t_vio, pos_vio, t_opt, pos_opt)
% Compute RMSE of VIO vs OptiTrack on the overlapping time interval.
% OptiTrack is interpolated onto the VIO time grid.
    t_start = max(t_vio(1), t_opt(1));
    t_end   = min(t_vio(end), t_opt(end));

    if t_end <= t_start
        warning('No overlapping time interval for RMSE computation.');
        rmse_x = NaN; rmse_y = NaN; rmse_z = NaN; rmse_3d = NaN;
        return;
    end

    mask       = t_vio >= t_start & t_vio <= t_end;
    opt_interp = interp1(t_opt, pos_opt, t_vio(mask), 'linear');
    err        = pos_vio(mask,:) - opt_interp;

    rmse_x  = sqrt(mean(err(:,1).^2));
    rmse_y  = sqrt(mean(err(:,2).^2));
    rmse_z  = sqrt(mean(err(:,3).^2));
    rmse_3d = sqrt(mean(sum(err.^2, 2)));
end
