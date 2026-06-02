%% feature_overlay_video.m
% Video: kamerový záběr přes celou obrazovku s překreslenými feature pointy.
%
% Vstupy:
%   feat_pixels.mat  – vygenerovaný extract_feature_pixels.py
%   camera rosbag    – cam_bag_path níže
%
% Výstup: matlab/drift_videos/feature_overlay.avi

clear all
close all

%% ==================== KONFIGURACE ====================
cam_bag_path = '/mnt/ros2bags/rosbag2_2026_03_06-10_53_23';
cam_topic    = '/f450_1/sensors/realsense_d435/realsense_camera_node/infra1/image_rect_raw';

feat_mat     = fullfile(fileparts(mfilename('fullpath')), 'feat_pixels.mat');

out_dir  = fullfile(fileparts(mfilename('fullpath')), 'drift_videos');
out_file = fullfile(out_dir, 'feature_overlay.avi');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

video_fps     = 15;
fig_size      = [848 480];   % přesně rozlišení kamery (nebo zadat jiné)

feat_max_dt   = 0.15;        % max. časový rozdíl cam↔feature [s]; jinak bez bodů
dot_radius    = 4;           % poloměr kružnice [px]
dot_color     = [0 1 0];     % RGB barva feature bodů
dot_thickness = 1.5;
% ======================================================

%% NAČTENÍ FEATURE DAT
fprintf('Načítám %s ...\n', feat_mat);
F = load(feat_mat);
t_feat = F.t_feat_abs(:);   % (M,1) absolutní časy
feat_px = F.feat_px;        % (M, Nmax)
feat_py = F.feat_py;        % (M, Nmax)

%% NAČTENÍ KAMEROVÝCH SNÍMKŮ
fprintf('Načítám camera frames z bagu (může trvat pár minut)...\n');
cam_bag  = ros2bagreader(cam_bag_path);
cam_msgs = readMessages(select(cam_bag, "Topic", cam_topic));
n_frames = length(cam_msgs);
fprintf('Načteno %d camera frames.\n', n_frames);

t_cam = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1e-9, cam_msgs);

%% SETUP VIDEA
fprintf('Generuji video: %s\n', out_file);
vw = VideoWriter(out_file, 'Motion JPEG AVI');
vw.FrameRate = video_fps;
vw.Quality   = 100;
open(vw);

hfig = figure('Position', [50 50 fig_size(1) fig_size(2)], ...
              'Color', 'k', 'Visible', 'on', 'MenuBar', 'none', 'ToolBar', 'none');
ax = axes('Parent', hfig, 'Position', [0 0 1 1]);
axis(ax, 'off');

%% RENDEROVÁNÍ
fprintf('Renderuji %d frameů...\n', n_frames);

for fi = 1:n_frames
    msg    = cam_msgs{fi};
    t_curr = t_cam(fi);

    % Dekóduj snímek (mono8, step x height → height x width)
    img = reshape(msg.data, msg.step, msg.height)';
    img = img(:, 1:msg.width);
    img_rgb = repmat(img, [1 1 3]);   % grayscale → RGB pro kreslení barev

    % Nejbližší feature frame
    [dt, fi_feat] = min(abs(t_feat - t_curr));
    if dt <= feat_max_dt
        px = feat_px(fi_feat, :);
        py = feat_py(fi_feat, :);
        valid = isfinite(px) & isfinite(py);
        px = px(valid);
        py = py(valid);

        % Nakresli kruhy do obrazu
        img_rgb = draw_circles(img_rgb, px, py, dot_radius, dot_color, dot_thickness);
    end

    % Zobraz přes celé pole
    imshow(img_rgb, 'Parent', ax);
    axis(ax, 'off');

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


%% -------- lokální helper: kreslení prázdných kružnic --------
function img = draw_circles(img, cx, cy, r, color, thickness)
% Vektorizovaně vykreslí kružnice do RGB uint8 obrazu.
% cx, cy  – středy [px] (řádkové vektory), r – poloměr, color – [R G B] 0..1

[H, W, ~] = size(img);
col8 = uint8(color * 255);

% Precompute pixel offsets tvořící "tlustý" prstenec (rmin..rmax)
rmin = max(0, r - thickness);
rmax = r + thickness;
bnd  = ceil(rmax);
[gx, gy] = meshgrid(-bnd:bnd, -bnd:bnd);
dist = sqrt(gx.^2 + gy.^2);
mask = dist >= rmin & dist <= rmax;
ox = gx(mask);   % offsety x
oy = gy(mask);   % offsety y

% Pro všechna středová místa najednou
cx = round(cx(:))';
cy = round(cy(:))';

% Matice všech pixel souřadnic: (n_offsets × n_features)
all_px = bsxfun(@plus, ox, cx);   % (n_off, n_feat)
all_py = bsxfun(@plus, oy, cy);

% Linearní indexy (flatten, oříznutí na obraz)
all_px = all_px(:);
all_py = all_py(:);
in = all_px >= 1 & all_px <= W & all_py >= 1 & all_py <= H;
all_px = all_px(in);
all_py = all_py(in);

idx = sub2ind([H W], all_py, all_px);
idx = unique(idx);

img(idx)         = col8(1);
img(idx + H*W)   = col8(2);
img(idx + 2*H*W) = col8(3);
end
