%% gps_vio_compare_6310.m
% Porovnání GPS (NavSatFix) a VIO trajektorie z bagu 6.3.10
% Rosbag: rosbags/6.3.10_53/gps_vio_2026_03_06
%
% Topics:
%   /f450_1/aircraft/raw_gps                           - sensor_msgs/NavSatFix
%   /f450_1/vio_isaac/visual_slam/tracking/odometry    - nav_msgs/Odometry

clear all
close all

%% KONFIGURACE
bag_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags', ...
    '6.3.10_53', 'gps_vio_2026_03_06');

gps_topic = "/f450_1/aircraft/raw_gps";
vio_topic = "/f450_1/vio_isaac/visual_slam/tracking/odometry";

% Manuální yaw offset VIO vůči GPS [stupně]
% Uprav dokud trajektorie nesedí (VIO se otočí o tuto hodnotu)
vio_yaw_offset_deg = 20;

% Násobič kovariance pro vizualizaci (1 = 1σ, 2 = 2σ, 3 = 3σ)
sigma_scale = 1;

% Video render - nastav na true pro generování (pomalé, čte velký bag)
% Varianta 1: kamera vlevo | X/Y/Z + chyba vpravo
% Varianta 2: kamera vlevo | chyba + camera delay vpravo
make_drift_videos    = false;
make_drift_videos_v2 = true;

%% NAČTENÍ BAGU
fprintf('Načítám bag: %s\n', bag_path);
bag = ros2bagreader(bag_path);

%% GPS - NavSatFix -> lokální ENU souřadnice
fprintf('Čtu GPS data...\n');
gps_msgs = readMessages(select(bag, "Topic", gps_topic));

% Extrakce lat, lon, alt, kovariance a časů
lat = cellfun(@(m) m.latitude,  gps_msgs);
lon = cellfun(@(m) m.longitude, gps_msgs);
alt = cellfun(@(m) m.altitude,  gps_msgs);
t_gps = cellfun(@(m) double(m.header.stamp.sec) + ...
                     double(m.header.stamp.nanosec)*1e-9, gps_msgs);

% Kovariance pozice - NavSatFix ukládá 3x3 matici po řádcích (ENU)
% Diagonála: [0]=var_E, [4]=var_N, [8]=var_U  (indexy 1,5,9 v MATLABu)
cov_raw = cellfun(@(m) reshape(m.position_covariance, 1, 9), gps_msgs, 'UniformOutput', false);
cov_raw = vertcat(cov_raw{:});   % [N x 9]
sigma_gps = sqrt(abs(cov_raw(:, [1 5 9])));  % 1-sigma [E, N, U]

% Převod geodetických souřadnic na lokální ENU (origin = první bod)
lat0 = lat(1);
lon0 = lon(1);
alt0 = alt(1);

% WGS84 parametry
a  = 6378137.0;           % velká poloosa [m]
f  = 1/298.257223563;     % zploštění
e2 = 2*f - f^2;           % první excentricita^2

% LLA -> ECEF
N = @(la) a ./ sqrt(1 - e2*sind(la).^2);

ecef = @(la, lo, al) [ ...
    (N(la) + al) .* cosd(la) .* cosd(lo), ...
    (N(la) + al) .* cosd(la) .* sind(lo), ...
    (N(la).*(1-e2) + al) .* sind(la) ];

p_ecef  = ecef(lat,  lon,  alt);
p0_ecef = ecef(lat0, lon0, alt0);

% ECEF rozdíl -> ENU rotací
dp = p_ecef - p0_ecef;

R_enu = [ -sind(lon0),               cosd(lon0),              0;
          -sind(lat0)*cosd(lon0),    -sind(lat0)*sind(lon0),   cosd(lat0);
           cosd(lat0)*cosd(lon0),     cosd(lat0)*sind(lon0),   sind(lat0) ];

pos_gps = (R_enu * dp')';   % [N x 3]  E, N, U

% Filtrování špatných GPS bodů (nulový fix apod.)
valid = ~(lat == 0 & lon == 0);
pos_gps   = pos_gps(valid, :);
t_gps     = t_gps(valid);
sigma_gps = sigma_gps(valid, :);

% Oba začínají z [0,0,0] — GPS origin posuneme na první validní bod
pos_gps = pos_gps - pos_gps(1,:);

fprintf('GPS sigma - mean:  E=%.4f  N=%.4f  U=%.4f m\n', mean(sigma_gps));
fprintf('GPS sigma - max:   E=%.4f  N=%.4f  U=%.4f m\n', max(sigma_gps));

%% VIO - nav_msgs/Odometry
fprintf('Čtu VIO data...\n');
vio_msgs = readMessages(select(bag, "Topic", vio_topic));
[t_vio, pos_vio] = odomCellToPosT(vio_msgs);

% Kovariance pozice VIO - pose.covariance je 6x6 (36 prvků), řádkový vektor
% Diagonála: [1]=var_x, [8]=var_y, [15]=var_z
vio_cov_raw = cellfun(@(m) reshape(m.pose.covariance, 1, 36), vio_msgs, 'UniformOutput', false);
vio_cov_raw = vertcat(vio_cov_raw{:});   % [N x 36]
sigma_vio   = sqrt(abs(vio_cov_raw(:, [1 8 15])));  % 1-sigma [x, y, z]

fprintf('VIO sigma - mean:  x=%.4f  y=%.4f  z=%.4f m\n', mean(sigma_vio));
fprintf('VIO sigma - max:   x=%.4f  y=%.4f  z=%.4f m\n', max(sigma_vio));

%% ALIGNMENT VIO -> GPS
% 1) Posunutí počátku VIO na počátek GPS
pos_vio_aligned = pos_vio - pos_vio(1,:);   % VIO začíná v [0,0,0]

% 2) Yaw rotace VIO (VIO lokální frame -> GPS ENU frame)
%    Automaticky odhadneme yaw z prvních N bodů GPS a VIO,
%    pak přičteme manuální offset pro doladění.
N_fit = min(20, min(length(pos_gps), length(pos_vio_aligned)));

% Směrový vektor z prvních N bodů
gps_dir = pos_gps(N_fit, 1:2) - pos_gps(1, 1:2);
vio_dir = pos_vio_aligned(N_fit, 1:2) - pos_vio_aligned(1, 1:2);

if norm(gps_dir) > 0.1 && norm(vio_dir) > 0.1
    yaw_gps = atan2(gps_dir(2), gps_dir(1));
    yaw_vio = atan2(vio_dir(2), vio_dir(1));
    yaw_auto = yaw_gps - yaw_vio;
    fprintf('Auto yaw alignment: %.1f°\n', rad2deg(yaw_auto));
else
    yaw_auto = 0;
    warning('Nedostatek pohybu pro auto yaw - použij vio_yaw_offset_deg ručně');
end

yaw_total = yaw_auto + deg2rad(vio_yaw_offset_deg);
R_yaw = [cos(yaw_total) -sin(yaw_total) 0;
         sin(yaw_total)  cos(yaw_total) 0;
         0               0              1];

pos_vio_aligned = (R_yaw * pos_vio_aligned')';
% Sigma se rotuje stejnou yaw maticí (pouze XY, Z zůstává)
sigma_vio_aligned = (R_yaw * sigma_vio')';

%% NORMALIZACE ČASŮ NA 0
t_gps_norm = t_gps - t_gps(1);
t_vio_norm = t_vio - t_vio(1);

%% VÝPOČET CHYBY GPS-VIO
% Interpolace VIO na časovou osu GPS (GPS má nižší frekvenci -> méně bodů)
t_common_start = max(t_gps_norm(1),   t_vio_norm(1));
t_common_end   = min(t_gps_norm(end), t_vio_norm(end));
mask_gps = t_gps_norm >= t_common_start & t_gps_norm <= t_common_end;
t_common = t_gps_norm(mask_gps);

pos_gps_common = pos_gps(mask_gps, :);
pos_vio_common = interp1(t_vio_norm, pos_vio_aligned, t_common, 'linear');

err_3d = vecnorm(pos_gps_common - pos_vio_common, 2, 2);
err_xy = vecnorm(pos_gps_common(:,1:2) - pos_vio_common(:,1:2), 2, 2);
err_z  = abs(pos_gps_common(:,3) - pos_vio_common(:,3));

sigma_gps_common = interp1(t_gps_norm, sigma_gps, t_common, 'linear');
sigma_3d_common  = vecnorm(sigma_gps_common, 2, 2);

fprintf('\n=== CHYBA GPS-VIO ===\n');
fprintf('3D chyba  - mean: %.2f m,  max: %.2f m,  RMSE: %.2f m\n', ...
    mean(err_3d), max(err_3d), sqrt(mean(err_3d.^2)));
fprintf('XY chyba  - mean: %.2f m,  max: %.2f m\n', mean(err_xy), max(err_xy));
fprintf('Z  chyba  - mean: %.2f m,  max: %.2f m\n', mean(err_z),  max(err_z));
fprintf('=====================\n');

%% -------- FIGURE 1: X, Y, Z, chyba vs čas --------
figure('Name', 'GPS vs VIO - pozice v čase', 'NumberTitle', 'off', ...
       'Position', [100 50 1000 900]);

ax_labels = {'X (East) [m]', 'Y (North) [m]', 'Z (Up) [m]'};

for ax = 1:3
    subplot(4,1,ax);
    % Kovarianční pás GPS ±Nσ
    t_fill = [t_gps_norm; flipud(t_gps_norm)];
    y_fill = [pos_gps(:,ax) + sigma_scale*sigma_gps(:,ax); ...
              flipud(pos_gps(:,ax) - sigma_scale*sigma_gps(:,ax))];
    hfill_gps = fill(t_fill, y_fill, [0.6 0.8 1.0], ...
                     'EdgeColor', 'none', 'FaceAlpha', 0.4);
    hold on;
    h1 = plot(t_gps_norm, pos_gps(:,ax), 'b-', 'LineWidth', 1.5);
    h2 = plot(t_vio_norm, pos_vio_aligned(:,ax), 'r-', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Čas [s]');
    ylabel(ax_labels{ax});
    title(ax_labels{ax});
    % VIO kovariance Isaac ROS jsou ~20x menší než GPS -> na tomto měřítku neviditelné
    legend([h1 h2 hfill_gps], ...
           {'GPS (ENU)', sprintf('VIO (cov ≈%.2fm)', mean(sigma_vio(:,ax))), ...
            sprintf('GPS ±%dσ', sigma_scale)}, ...
           'Location', 'best');
end

% 4. subplot: chyba GPS-VIO v čase
subplot(4,1,4);
t_fill = [t_common; flipud(t_common)];
y_fill = [sigma_scale*sigma_3d_common; zeros(size(sigma_3d_common))];
hfill = fill(t_fill, y_fill, [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
h3d  = plot(t_common, err_3d, 'k-',  'LineWidth', 1.5);
hxy  = plot(t_common, err_xy, 'b--', 'LineWidth', 1.2);
hz   = plot(t_common, err_z,  'r--', 'LineWidth', 1.2);
hmean = yline(mean(err_3d), 'k:', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Čas [s]');
ylabel('Chyba [m]');
title('Geometrická chyba GPS - VIO');
legend([h3d hxy hz hfill hmean], ...
       {'3D', 'XY', 'Z', sprintf('GPS %dσ nejistota', sigma_scale), sprintf('mean = %.2f m', mean(err_3d))}, ...
       'Location', 'best');

sgtitle('Porovnání GPS a VIO trajektorie', 'FontSize', 13, 'FontWeight', 'bold');

%% -------- FIGURE 2: 3D trajektorie --------
figure('Name', 'GPS vs VIO - 3D trajektorie', 'NumberTitle', 'off', ...
       'Position', [200 100 900 650]);

h1 = plot3(pos_gps(:,1), pos_gps(:,2), pos_gps(:,3), ...
           'b-o', 'LineWidth', 1.5, 'MarkerSize', 2, 'MarkerIndices', 1:10:length(pos_gps));
hold on;
h2 = plot3(pos_vio_aligned(:,1), pos_vio_aligned(:,2), pos_vio_aligned(:,3), ...
           'r-', 'LineWidth', 1.5);

% Počáteční body
plot3(pos_gps(1,1),  pos_gps(1,2),  pos_gps(1,3),  'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot3(pos_vio_aligned(1,1),  pos_vio_aligned(1,2),  pos_vio_aligned(1,3),  'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Koncové body
plot3(pos_gps(end,1), pos_gps(end,2), pos_gps(end,3), 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot3(pos_vio_aligned(end,1), pos_vio_aligned(end,2), pos_vio_aligned(end,3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

hold off;
grid on;
axis equal;
xlabel('X (East) [m]');
ylabel('Y (North) [m]');
zlabel('Z (Up) [m]');
legend([h1, h2], {'GPS (ENU)', 'VIO'}, 'Location', 'best');
title('3D trajektorie: GPS vs VIO');
view(45, 30);

%% -------- FIGURE 3: Drift --------
figure('Name', 'GPS vs VIO - drift', 'NumberTitle', 'off', ...
       'Position', [300 100 900 350]);

cum_dist = [0; cumsum(vecnorm(diff(pos_gps_common), 2, 2))];
plot(cum_dist, err_3d, 'k-', 'LineWidth', 1.5); hold on;
plot(cum_dist, err_xy, 'b--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Ujetá vzdálenost GPS [m]');
ylabel('Chyba [m]');
title(sprintf('Drift: chyba vs ujetá vzdálenost  (konečná 3D chyba: %.2f m)', err_3d(end)));
legend({'3D', 'XY'}, 'Location', 'best');

fprintf('Hotovo!\n');

%% -------- FIGURE 4 + VSLAM data (z cache .mat) --------
mat_dir    = fileparts(mfilename('fullpath'));
py_script  = fullfile(mat_dir, 'extract_slam_status.py');
timing_mat = fullfile(mat_dir, 'cam_timing.mat');
status_mat = fullfile(mat_dir, 'slam_status.mat');
obs_mat    = fullfile(mat_dir, 'slam_observations.mat');

% Spusť Python extrakci pokud chybí jakýkoli .mat
if ~isfile(timing_mat) || ~isfile(status_mat) || ~isfile(obs_mat)
    fprintf('Chybí cache .mat, spouštím Python extrakci (jen poprvé)...\n');
    ret = system(sprintf('bash -c "source /opt/ros/humble/setup.bash && python3 %s"', py_script));
    if ret ~= 0
        error('Extrakce selhala. Spusť ručně: python3 %s', py_script);
    end
end

% Camera timing
c          = load(timing_mat);
t_cam_norm = c.t_cam_abs(:) - t_vio(1);
dt_cam     = c.dt_cam(:);
fps_cam    = c.fps_cam(:);

s             = load(status_mat);
t_status_norm = s.t_status_abs - t_vio(1);
track_time_ms = s.track_time_ms(:);
vo_state_arr  = s.vo_state(:);
vo_fails      = vo_state_arr ~= 1;
fprintf('VSLAM status: %d zpráv, %d failures, track mean=%.2f ms, max=%.2f ms\n', ...
        numel(track_time_ms), sum(vo_fails), mean(track_time_ms), max(track_time_ms));

o               = load(obs_mat);
t_obs_norm      = o.t_obs_abs - t_vio(1);
feature_count   = o.feature_count(:);
feat_low_thresh = 80;   % pod tímto = problematické tracking (empiricky z issue #200)
fprintf('Feature count: %d framů, mean=%.0f, min=%.0f, max=%.0f, <80: %.1f%%\n', ...
        numel(feature_count), mean(feature_count), min(feature_count), ...
        max(feature_count), 100*mean(feature_count < feat_low_thresh));

figure('Name', 'Camera frame timing', 'NumberTitle', 'off', ...
       'Position', [100 100 1100 500]);

subplot(2,1,1);
plot(t_cam_norm(2:end), dt_cam, 'b-', 'LineWidth', 1.0);
hold on;
yline(mean(dt_cam), 'k--', 'LineWidth', 1.5);
yline(median(dt_cam), 'g--', 'LineWidth', 1.5);
% Zvýrazni výpadky (dt > 2x medián)
thresh = 2 * median(dt_cam);
drops  = dt_cam > thresh;
plot(t_cam_norm(find(drops)+1), dt_cam(drops), 'rv', ...
     'MarkerSize', 7, 'MarkerFaceColor', 'r');
hold off;
grid on;
xlabel('Čas [s]');
ylabel('Interval mezi snímky [ms]');
title('Frame interval kamery (infra1)');
legend({'dt', sprintf('mean = %.1f ms', mean(dt_cam)), ...
        sprintf('median = %.1f ms', median(dt_cam)), 'výpadky (>2× median)'}, ...
       'Location', 'best');

subplot(2,1,2);
plot(t_cam_norm(2:end), fps_cam, 'r-', 'LineWidth', 1.0);
hold on;
yline(mean(fps_cam), 'k--', 'LineWidth', 1.5);
hold off;
grid on;
ylim([0, min(max(fps_cam)*1.2, 60)]);
xlabel('Čas [s]');
ylabel('FPS');
title(sprintf('Okamžitý FPS  (mean = %.1f,  výpadků = %d)', ...
      mean(fps_cam), sum(drops)));

sgtitle('Timing snímků RealSense infra1', 'FontSize', 12, 'FontWeight', 'bold');

%% -------- FIGURE 5: Feature count (observations_cloud) --------
figure('Name', 'VSLAM feature count', 'NumberTitle', 'off', ...
       'Position', [150 100 1100 500]);

subplot(2,1,1);
plot(t_obs_norm, feature_count, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.0);
hold on;
yline(mean(feature_count), 'k--', 'LineWidth', 1.3, ...
      'DisplayName', sprintf('mean = %.0f', mean(feature_count)));
yline(feat_low_thresh, 'r--', 'LineWidth', 1.2, ...
      'DisplayName', sprintf('práh %d (nespolehlivé)', feat_low_thresh));
% Zvýrazni oblasti pod prahem
low_mask = feature_count < feat_low_thresh;
plot(t_obs_norm(low_mask), feature_count(low_mask), 'r.', 'MarkerSize', 3);
hold off;
grid on;
xlabel('Čas [s]'); ylabel('Počet features');
title(sprintf('Feature count per frame  (mean=%.0f, min=%d, max=%d)', ...
      mean(feature_count), min(feature_count), max(feature_count)));
legend('Location', 'best');

subplot(2,1,2);
% Korelace: err_3d (GPS čas) vs feature count (obs čas) — interpolace na společnou osu
t_common_obs_start = max(t_common(1),   t_obs_norm(1));
t_common_obs_end   = min(t_common(end), t_obs_norm(end));
mask_common_obs    = t_common >= t_common_obs_start & t_common <= t_common_obs_end;
t_corr             = t_common(mask_common_obs);
err_corr           = err_3d(mask_common_obs);
feat_corr          = interp1(t_obs_norm, feature_count, t_corr, 'linear');

yyaxis left;
plot(t_corr, err_corr, 'k-', 'LineWidth', 1.3);
ylabel('GPS-VIO chyba [m]');
yyaxis right;
plot(t_corr, feat_corr, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.0);
yline(feat_low_thresh, 'r--', 'LineWidth', 1.0);
ylabel('Počet features');
grid on;
xlabel('Čas [s]');
title('Korelace: chyba GPS-VIO vs počet features');
legend({'GPS-VIO chyba', 'feature count', sprintf('práh %d', feat_low_thresh)}, ...
       'Location', 'best');

sgtitle('VSLAM observations_cloud — kvalita trackingu', 'FontSize', 12, 'FontWeight', 'bold');

%% -------- VIDEA: krátké klipy okolo největšího driftu --------
if make_drift_videos
    cam_bag_path = '/mnt/ros2bags/rosbag2_2026_03_06-10_53_23';
    cam_topic    = '/f450_1/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw';
    video_window = 5.0;     % ±sekund okolo každého času
    video_fps    = 20;      % FPS výstupního videa
    out_dir      = fullfile(fileparts(mfilename('fullpath')), 'drift_videos');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    % Manuálně zadané časy peaků [s]
    peak_times = [97, 106, 139, 152];

    fprintf('Načítám camera bag...\n');
    cam_bag      = ros2bagreader(cam_bag_path);
    t_abs_origin = t_vio(1);

    fig_w = 1600; fig_h = 820;
    hfig = figure('Visible', 'on', 'Position', [0 0 fig_w fig_h], 'Color', 'w');

    % Jedno video pro všechny peaky
    vid_path = fullfile(out_dir, 'drift_all_peaks.avi');
    vw = VideoWriter(vid_path, 'Motion JPEG AVI');
    vw.FrameRate = video_fps;
    vw.Quality   = 90;
    open(vw);

    for k = 1:length(peak_times)
        t_peak   = peak_times(k);
        t_abs_s  = t_abs_origin + t_peak - video_window;
        t_abs_e  = t_abs_origin + t_peak + video_window;
        err_peak = interp1(t_common, err_3d, t_peak);

        fprintf('Peak %d/%d: t=%.1fs, err=%.2fm - načítám frames...\n', ...
                k, length(peak_times), t_peak, err_peak);

        sel          = select(cam_bag, "Topic", cam_topic, "Time", [t_abs_s, t_abs_e]);
        cam_msgs_win = readMessages(sel);

        if isempty(cam_msgs_win)
            fprintf('  -> žádné framy v okně, přeskakuji\n');
            continue;
        end

        % VideoWriter se otevírá jednou před smyčkou, tady jen logujeme
        fprintf('  -> zapisuji do společného videa...\n');

        % ---- Předkresli statické části (jen jednou pro každý peak) ----
        clf(hfig);

        % Layout: kamera vlevo (sloupec 1), 4 grafy vpravo (sloupce 2-3)
        % subplot(4,3, ...) - 4 řádky x 3 sloupce
        ax_cam = subplot(4, 3, [1 4 7 10]);   % celý levý sloupec
        ax_x   = subplot(4, 3, [2  3]);
        ax_y   = subplot(4, 3, [5  6]);
        ax_z   = subplot(4, 3, [8  9]);
        ax_err = subplot(4, 3, [11 12]);

        % GPS sigma pás (statický)
        axes(ax_x);
        fill([t_gps_norm; flipud(t_gps_norm)], ...
             [pos_gps(:,1)+sigma_scale*sigma_gps(:,1); flipud(pos_gps(:,1)-sigma_scale*sigma_gps(:,1))], ...
             [0.6 0.8 1.0], 'EdgeColor','none','FaceAlpha',0.35); hold on;
        plot(ax_x, t_gps_norm, pos_gps(:,1), 'b-', 'LineWidth', 1.4);
        plot(ax_x, t_vio_norm, pos_vio_aligned(:,1), 'r-', 'LineWidth', 1.4);
        hcur_x = xline(ax_x, t_peak, 'k-', 'LineWidth', 1.5);
        hold off; grid on;
        ylabel(ax_x, 'X [m]'); set(ax_x,'XTickLabel',[]);
        legend(ax_x, {sprintf('GPS ±%dσ',sigma_scale),'GPS','VIO'}, ...
               'Location','best','FontSize',7);

        axes(ax_y);
        fill([t_gps_norm; flipud(t_gps_norm)], ...
             [pos_gps(:,2)+sigma_scale*sigma_gps(:,2); flipud(pos_gps(:,2)-sigma_scale*sigma_gps(:,2))], ...
             [0.6 0.8 1.0], 'EdgeColor','none','FaceAlpha',0.35); hold on;
        plot(ax_y, t_gps_norm, pos_gps(:,2), 'b-', 'LineWidth', 1.4);
        plot(ax_y, t_vio_norm, pos_vio_aligned(:,2), 'r-', 'LineWidth', 1.4);
        hcur_y = xline(ax_y, t_peak, 'k-', 'LineWidth', 1.5);
        hold off; grid on;
        ylabel(ax_y, 'Y [m]'); set(ax_y,'XTickLabel',[]);

        axes(ax_z);
        fill([t_gps_norm; flipud(t_gps_norm)], ...
             [pos_gps(:,3)+sigma_scale*sigma_gps(:,3); flipud(pos_gps(:,3)-sigma_scale*sigma_gps(:,3))], ...
             [0.6 0.8 1.0], 'EdgeColor','none','FaceAlpha',0.35); hold on;
        plot(ax_z, t_gps_norm, pos_gps(:,3), 'b-', 'LineWidth', 1.4);
        plot(ax_z, t_vio_norm, pos_vio_aligned(:,3), 'r-', 'LineWidth', 1.4);
        hcur_z = xline(ax_z, t_peak, 'k-', 'LineWidth', 1.5);
        hold off; grid on;
        ylabel(ax_z, 'Z [m]'); set(ax_z,'XTickLabel',[]);

        axes(ax_err);
        fill([t_common; flipud(t_common)], ...
             [sigma_scale*sigma_3d_common; zeros(size(sigma_3d_common))], ...
             [0.85 0.85 0.85], 'EdgeColor','none','FaceAlpha',0.6); hold on;
        plot(ax_err, t_common, err_3d, 'k-',  'LineWidth', 1.4);
        plot(ax_err, t_common, err_xy, 'b--', 'LineWidth', 1.0);
        hcur_err  = xline(ax_err, t_peak, 'k-', 'LineWidth', 1.5);
        hcur_dot  = plot(ax_err, t_peak, err_peak, 'ro', ...
                         'MarkerSize', 8, 'MarkerFaceColor', 'r');
        hold off; grid on;
        ylabel(ax_err, 'Chyba [m]'); xlabel(ax_err, 'Čas [s]');
        legend(ax_err, {sprintf('GPS %dσ',sigma_scale),'3D','XY','cursor'}, ...
               'Location','best','FontSize',7);

        % Zvýrazni oblast peaku na všech grafech
        for ax_h = [ax_x ax_y ax_z ax_err]
            xregion(ax_h, t_peak-video_window, t_peak+video_window, ...
                    'FaceColor',[1 0.8 0],'FaceAlpha',0.08);
        end

        % ---- Frame loop ----
        for fi = 1:length(cam_msgs_win)
            msg         = cam_msgs_win{fi};
            img         = reshape(msg.data, msg.step, msg.height)';
            img         = img(:, 1:msg.width);
            t_frame_abs = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
            t_rel       = t_frame_abs - t_abs_origin;

            if t_rel >= t_common(1) && t_rel <= t_common(end)
                err_now = interp1(t_common, err_3d, t_rel);
            else
                err_now = NaN;
            end

            % Kamera
            imshow(img, [], 'Parent', ax_cam);
            title(ax_cam, sprintf('t = %.2f s', t_rel), 'FontSize', 10);

            % Pohyb kurzorů
            set(hcur_x,   'Value', t_rel);
            set(hcur_y,   'Value', t_rel);
            set(hcur_z,   'Value', t_rel);
            set(hcur_err, 'Value', t_rel);
            if ~isnan(err_now)
                set(hcur_dot, 'XData', t_rel, 'YData', err_now);
            end

            sgtitle(hfig, sprintf('Drift peak %d/%d  |  t = %.1f s  |  peak err = %.2f m', ...
                    k, length(peak_times), t_rel, err_peak), ...
                    'FontSize', 11, 'FontWeight', 'bold');

            drawnow;
            frame     = getframe(hfig);
            frame_img = imresize(frame.cdata, [fig_h fig_w]);
            writeVideo(vw, frame_img);
        end

    end

    close(vw);
    close(hfig);
    fprintf('Video uloženo: %s\n', vid_path);
end

%% -------- VIDEO V2: kamera | chyba + camera delay --------
if make_drift_videos_v2
    cam_bag_path = '/mnt/ros2bags/rosbag2_2026_03_06-10_53_23';
    cam_topic    = '/f450_1/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw';
    video_window = 5.0;
    video_fps    = 20;
    out_dir      = fullfile(fileparts(mfilename('fullpath')), 'drift_videos');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    peak_times = [97, 106, 139, 152];

    fprintf('Načítám camera bag (v2)...\n');
    cam_bag      = ros2bagreader(cam_bag_path);
    t_abs_origin = t_vio(1);

    fig_w = 1400; fig_h = 1000;
    hfig2 = figure('Visible', 'on', 'Position', [0 0 fig_w fig_h], 'Color', 'w');

    vid_path2 = fullfile(out_dir, 'drift_all_peaks_v2.avi');
    vw2 = VideoWriter(vid_path2, 'Motion JPEG AVI');
    vw2.FrameRate = video_fps;
    vw2.Quality   = 90;
    open(vw2);

    for k = 1:length(peak_times)
        t_peak   = peak_times(k);
        t_abs_s  = t_abs_origin + t_peak - video_window;
        t_abs_e  = t_abs_origin + t_peak + video_window;
        err_peak = interp1(t_common, err_3d, t_peak);

        fprintf('V2 peak %d/%d: t=%.1fs, err=%.2fm - načítám frames...\n', ...
                k, length(peak_times), t_peak, err_peak);

        sel          = select(cam_bag, "Topic", cam_topic, "Time", [t_abs_s, t_abs_e]);
        cam_msgs_win = readMessages(sel);

        if isempty(cam_msgs_win)
            fprintf('  -> žádné framy v okně, přeskakuji\n');
            continue;
        end

        clf(hfig2);

        % Layout 4x2: kamera vlevo celá výška, vpravo 4 grafy
        ax_cam2  = subplot(4, 2, [1 3 5 7]);
        ax_err2  = subplot(4, 2, 2);
        ax_dt2   = subplot(4, 2, 4);
        ax_feat2 = subplot(4, 2, 6);
        ax_cov2  = subplot(4, 2, 8);

        % --- Graf chyby (statický) ---
        axes(ax_err2);
        fill([t_common; flipud(t_common)], ...
             [sigma_scale*sigma_3d_common; zeros(size(sigma_3d_common))], ...
             [0.85 0.85 0.85], 'EdgeColor','none','FaceAlpha',0.6); hold on;
        plot(ax_err2, t_common, err_3d, 'k-',  'LineWidth', 1.4);
        plot(ax_err2, t_common, err_xy, 'b--', 'LineWidth', 1.0);
        hcur_err2 = xline(ax_err2, t_peak, 'k-', 'LineWidth', 1.5);
        hcur_dot2 = plot(ax_err2, t_peak, err_peak, 'ro', ...
                         'MarkerSize', 8, 'MarkerFaceColor', 'r');
        hold off; grid on;
        ylabel(ax_err2, 'Chyba [m]'); xlabel(ax_err2, 'Čas [s]');
        title(ax_err2, 'Chyba GPS - VIO');
        legend(ax_err2, {sprintf('GPS %dσ',sigma_scale),'3D','XY'}, ...
               'Location','best','FontSize',7);
        xregion(ax_err2, t_peak-video_window, t_peak+video_window, ...
                'FaceColor',[1 0.8 0],'FaceAlpha',0.08);

        % --- Graf camera delay (statický) ---
        axes(ax_dt2);
        plot(ax_dt2, t_cam_norm(2:end), dt_cam, 'b-', 'LineWidth', 1.0); hold on;
        yline(median(dt_cam), 'g--', 'LineWidth', 1.3, 'DisplayName', ...
              sprintf('median = %.1f ms', median(dt_cam)));
        % výpadky
        thresh2 = 2 * median(dt_cam);
        drops2  = dt_cam > thresh2;
        plot(ax_dt2, t_cam_norm(find(drops2)+1), dt_cam(drops2), 'rv', ...
             'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', 'výpadky');
        hcur_dt2 = xline(ax_dt2, t_peak, 'k-', 'LineWidth', 1.5);
        hold off; grid on;
        ylabel(ax_dt2, 'Frame interval [ms]'); xlabel(ax_dt2, 'Čas [s]');
        title(ax_dt2, 'Camera frame interval');
        legend(ax_dt2, 'Location', 'best', 'FontSize', 7);
        xregion(ax_dt2, t_peak-video_window, t_peak+video_window, ...
                'FaceColor',[1 0.8 0],'FaceAlpha',0.08);

        % --- Graf feature count ---
        axes(ax_feat2);
        plot(ax_feat2, t_obs_norm, feature_count, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.0);
        hold on;
        yline(feat_low_thresh, 'r--', 'LineWidth', 1.2, ...
              'DisplayName', sprintf('práh %d', feat_low_thresh));
        low_mask_v = feature_count < feat_low_thresh;
        plot(ax_feat2, t_obs_norm(low_mask_v), feature_count(low_mask_v), ...
             'r.', 'MarkerSize', 3, 'HandleVisibility', 'off');
        hcur_feat2 = xline(ax_feat2, t_peak, 'k-', 'LineWidth', 1.5);
        hold off; grid on;
        ylabel(ax_feat2, 'Features [#]'); xlabel(ax_feat2, 'Čas [s]');
        title(ax_feat2, sprintf('Feature count  (mean=%.0f)', mean(feature_count)));
        legend(ax_feat2, 'Location', 'best', 'FontSize', 7);
        xregion(ax_feat2, t_peak-video_window, t_peak+video_window, ...
                'FaceColor',[1 0.8 0],'FaceAlpha',0.08);

        % --- Graf kovariance GPS + VIO (dual Y) ---
        sigma_3d_gps = vecnorm(sigma_gps, 2, 2);
        sigma_3d_vio = vecnorm(sigma_vio_aligned, 2, 2);
        axes(ax_cov2);
        yyaxis(ax_cov2, 'left');
        plot(ax_cov2, t_gps_norm, sigma_3d_gps, 'b-', 'LineWidth', 1.0);
        ylabel(ax_cov2, 'GPS \sigma [m]');
        yyaxis(ax_cov2, 'right');
        plot(ax_cov2, t_vio_norm, sigma_3d_vio, 'r-', 'LineWidth', 1.0);
        ylabel(ax_cov2, 'VIO \sigma [m]');
        hcur_cov2 = xline(ax_cov2, t_peak, 'k-', 'LineWidth', 1.5);
        grid(ax_cov2, 'on');
        xlabel(ax_cov2, 'Čas [s]');
        title(ax_cov2, sprintf('Kovariance  GPS mean=%.2fm  VIO mean=%.3fm', ...
              mean(sigma_3d_gps), mean(sigma_3d_vio)));
        legend(ax_cov2, {'GPS 3D\sigma', 'VIO 3D\sigma'}, 'Location', 'best', 'FontSize', 7);
        xregion(ax_cov2, t_peak-video_window, t_peak+video_window, ...
                'FaceColor',[1 0.8 0],'FaceAlpha',0.08);

        % --- Frame loop ---
        for fi = 1:length(cam_msgs_win)
            msg         = cam_msgs_win{fi};
            img         = reshape(msg.data, msg.step, msg.height)';
            img         = img(:, 1:msg.width);
            t_frame_abs = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
            t_rel       = t_frame_abs - t_abs_origin;

            if t_rel >= t_common(1) && t_rel <= t_common(end)
                err_now = interp1(t_common, err_3d, t_rel);
            else
                err_now = NaN;
            end

            imshow(img, [], 'Parent', ax_cam2);
            title(ax_cam2, sprintf('t = %.2f s', t_rel), 'FontSize', 10);

            set(hcur_err2,  'Value', t_rel);
            set(hcur_dt2,   'Value', t_rel);
            set(hcur_feat2, 'Value', t_rel);
            set(hcur_cov2,  'Value', t_rel);
            if ~isnan(err_now)
                set(hcur_dot2, 'XData', t_rel, 'YData', err_now);
            end

            sgtitle(hfig2, sprintf('Drift peak %d/%d  |  t = %.1f s  |  peak err = %.2f m', ...
                    k, length(peak_times), t_rel, err_peak), ...
                    'FontSize', 11, 'FontWeight', 'bold');

            drawnow;
            frame     = getframe(hfig2);
            frame_img = imresize(frame.cdata, [fig_h fig_w]);
            writeVideo(vw2, frame_img);
        end
    end

    close(vw2);
    close(hfig2);
    fprintf('Video v2 uloženo: %s\n', vid_path2);
end
