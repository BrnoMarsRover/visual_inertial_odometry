%% vio_overview_video.m
% Generuje synchronizované video:
%   - snímek kamery (RealSense infra1)
%   - 3D trajektorie GPS + VIO s pohybující se aktuální pozicí
%   - graf chyby GPS-VIO s pohybujícím se kurzorem
%
% Výstup: matlab/drift_videos/overview.avi

clear all
close all

%% ==================== KONFIGURACE ====================
vio_bag_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags', ...
    '6.3.10_53', 'gps_vio_2026_03_06');
cam_bag_path = '/mnt/ros2bags/rosbag2_2026_03_06-10_53_23';

gps_topic = "/f450_1/aircraft/raw_gps";
vio_topic = "/f450_1/vio_isaac/visual_slam/tracking/odometry";
cam_topic = "/f450_1/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw";

vio_yaw_offset_deg = 0;   % manuální doladění alignmentu

out_dir  = fullfile(fileparts(mfilename('fullpath')), 'drift_videos');
out_file = fullfile(out_dir, 'overview.avi');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

video_fps    = 15;    % FPS výstupního videa (kamera jede ~22fps, downsample)
fig_size     = [1600 600];   % šířka x výška figury [px]
% ======================================================

%% NAČTENÍ VIO + GPS DAT
fprintf('Načítám VIO bag...\n');
bag = ros2bagreader(vio_bag_path);

% GPS -> ENU
gps_msgs = readMessages(select(bag, "Topic", gps_topic));
lat = cellfun(@(m) m.latitude,  gps_msgs);
lon = cellfun(@(m) m.longitude, gps_msgs);
alt = cellfun(@(m) m.altitude,  gps_msgs);
t_gps = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1e-9, gps_msgs);

lat0 = lat(1); lon0 = lon(1); alt0 = alt(1);
a  = 6378137.0; f = 1/298.257223563; e2 = 2*f - f^2;
N_fn = @(la) a ./ sqrt(1 - e2*sind(la).^2);
ecef = @(la,lo,al) [(N_fn(la)+al).*cosd(la).*cosd(lo), ...
                    (N_fn(la)+al).*cosd(la).*sind(lo), ...
                    (N_fn(la).*(1-e2)+al).*sind(la)];
dp = ecef(lat,lon,alt) - ecef(lat0,lon0,alt0);
R_enu = [-sind(lon0), cosd(lon0), 0;
         -sind(lat0)*cosd(lon0), -sind(lat0)*sind(lon0), cosd(lat0);
          cosd(lat0)*cosd(lon0),  cosd(lat0)*sind(lon0), sind(lat0)];
pos_gps = (R_enu * dp')';
valid = ~(lat == 0 & lon == 0);
pos_gps = pos_gps(valid,:);
t_gps   = t_gps(valid);

% VIO
vio_msgs = readMessages(select(bag, "Topic", vio_topic));
t_vio = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1e-9, vio_msgs);
pos_vio_raw = cellfun(@(m) [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z], ...
    vio_msgs, 'UniformOutput', false);
pos_vio_raw = vertcat(pos_vio_raw{:});

% Alignment VIO -> GPS
pos_vio = pos_vio_raw - pos_vio_raw(1,:);
N_fit = min(20, min(length(pos_gps), size(pos_vio,1)));
gps_dir = pos_gps(N_fit,1:2) - pos_gps(1,1:2);
vio_dir = pos_vio(N_fit,1:2) - pos_vio(1,1:2);
if norm(gps_dir) > 0.1 && norm(vio_dir) > 0.1
    yaw_auto = atan2(gps_dir(2),gps_dir(1)) - atan2(vio_dir(2),vio_dir(1));
    fprintf('Auto yaw alignment: %.1f°\n', rad2deg(yaw_auto));
else
    yaw_auto = 0;
end
yaw_total = yaw_auto + deg2rad(vio_yaw_offset_deg);
R_yaw = [cos(yaw_total) -sin(yaw_total) 0; sin(yaw_total) cos(yaw_total) 0; 0 0 1];
pos_vio = (R_yaw * pos_vio')';

% Normalizace časů
t0 = t_vio(1);
t_gps_norm = t_gps - t0;
t_vio_norm = t_vio - t0;

% Chyba GPS-VIO na společné ose
t_start = max(t_gps_norm(1), t_vio_norm(1));
t_end   = min(t_gps_norm(end), t_vio_norm(end));
mask    = t_gps_norm >= t_start & t_gps_norm <= t_end;
t_err   = t_gps_norm(mask);
pos_gps_c = pos_gps(mask,:);
pos_vio_c = interp1(t_vio_norm, pos_vio, t_err);
err_3d = vecnorm(pos_gps_c - pos_vio_c, 2, 2);
err_xy = vecnorm(pos_gps_c(:,1:2) - pos_vio_c(:,1:2), 2, 2);

%% NAČTENÍ KAMEROVÝCH SNÍMKŮ
fprintf('Načítám camera frames z bagu (může trvat pár minut)...\n');
cam_bag  = ros2bagreader(cam_bag_path);
cam_msgs = readMessages(select(cam_bag, "Topic", cam_topic));
fprintf('Načteno %d camera frames.\n', length(cam_msgs));

% Timestampy kamer
t_cam = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1e-9, cam_msgs);
t_cam_norm = t_cam - t0;

% Decoding prvního framu pro rozměry
msg1   = cam_msgs{1};
img1   = reshape(msg1.data, msg1.step, msg1.height)';
img1   = img1(:, 1:msg1.width);

%% SETUP VIDEA
fprintf('Generuji video: %s\n', out_file);
vw = VideoWriter(out_file, 'Motion JPEG AVI');
vw.FrameRate = video_fps;
vw.Quality   = 85;
open(vw);

hfig = figure('Position', [50 50 fig_size(1) fig_size(2)], ...
              'Color', 'k', 'Visible', 'on');

% Předvykresli statické části trajektorie
ax_cam = subplot(1,3,1, 'Parent', hfig);
ax_3d  = subplot(1,3,2, 'Parent', hfig);
ax_err = subplot(1,3,3, 'Parent', hfig);

% 3D trajektorie - statická část
axes(ax_3d);
h_gps_traj = plot3(pos_gps(:,1), pos_gps(:,2), pos_gps(:,3), ...
    'Color', [0.3 0.6 1.0], 'LineWidth', 1.2);
hold on;
h_vio_traj = plot3(pos_vio(:,1), pos_vio(:,2), pos_vio(:,3), ...
    'Color', [1.0 0.4 0.4], 'LineWidth', 1.2);
h_gps_cur  = plot3(pos_gps(1,1), pos_gps(1,2), pos_gps(1,3), ...
    'bo', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.6 1.0]);
h_vio_cur  = plot3(pos_vio(1,1), pos_vio(1,2), pos_vio(1,3), ...
    'ro', 'MarkerSize', 10, 'MarkerFaceColor', [1.0 0.3 0.3]);
hold off;
grid on; axis equal;
xlabel('X [m]', 'Color', 'w'); ylabel('Y [m]', 'Color', 'w'); zlabel('Z [m]', 'Color', 'w');
legend({'GPS', 'VIO', 'GPS pos', 'VIO pos'}, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], ...
    'Location', 'best', 'FontSize', 8);
title('3D trajektorie', 'Color', 'w');
set(ax_3d, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
view(45, 30);

% Chyba - statická část
axes(ax_err);
h_err3d = plot(t_err, err_3d, 'Color', [0.9 0.9 0.9], 'LineWidth', 1.5);
hold on;
h_errxy = plot(t_err, err_xy, 'Color', [0.5 0.7 1.0], 'LineWidth', 1.0, 'LineStyle', '--');
h_cur_line = xline(0, 'r-', 'LineWidth', 1.5);
h_cur_dot  = plot(t_err(1), err_3d(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold off;
grid on;
xlabel('Čas [s]', 'Color', 'w'); ylabel('Chyba [m]', 'Color', 'w');
title('GPS - VIO chyba', 'Color', 'w');
legend({'3D', 'XY'}, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], ...
    'Location', 'best', 'FontSize', 8);
set(ax_err, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
xlim([t_err(1) t_err(end)]);

set(hfig, 'Color', 'k');

%% RENDEROVÁNÍ FRAME PO FRAMU
n_frames = length(cam_msgs);
fprintf('Renderuji %d frameů...\n', n_frames);

for fi = 1:n_frames
    msg    = cam_msgs{fi};
    t_curr = t_cam_norm(fi);

    % Dekóduj obraz
    img = reshape(msg.data, msg.step, msg.height)';
    img = img(:, 1:msg.width);

    % --- Kamera ---
    axes(ax_cam); %#ok<LAXES>
    imshow(img, [], 'Parent', ax_cam);
    title(ax_cam, sprintf('t = %.2f s', t_curr), 'Color', 'w', 'FontSize', 10);
    set(ax_cam, 'Color', 'k');

    % --- 3D pozice ---
    % VIO aktuální pozice
    [~, i_vio] = min(abs(t_vio_norm - t_curr));
    set(h_vio_cur, 'XData', pos_vio(i_vio,1), 'YData', pos_vio(i_vio,2), 'ZData', pos_vio(i_vio,3));
    % GPS nejbližší bod
    [~, i_gps] = min(abs(t_gps_norm - t_curr));
    set(h_gps_cur, 'XData', pos_gps(i_gps,1), 'YData', pos_gps(i_gps,2), 'ZData', pos_gps(i_gps,3));

    % --- Kurzor v grafu chyby ---
    set(h_cur_line, 'Value', t_curr);
    if t_curr >= t_err(1) && t_curr <= t_err(end)
        err_now = interp1(t_err, err_3d, t_curr);
        set(h_cur_dot, 'XData', t_curr, 'YData', err_now);
    end

    % Titulek
    sgtitle(hfig, sprintf('VIO Overview  |  t = %.1f s / %.1f s', t_curr, t_err(end)), ...
        'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');

    drawnow limitrate;
    frame = getframe(hfig);
    frame_img = imresize(frame.cdata, [fig_size(2) fig_size(1)]);
    writeVideo(vw, frame_img);

    if mod(fi, 100) == 0
        fprintf('  %d / %d (%.0f%%)\n', fi, n_frames, 100*fi/n_frames);
    end
end

close(vw);
close(hfig);
fprintf('Video uloženo: %s\n', out_file);
