% Live visualization — GPS vs VIO + SLAM feature count
%
% 1. ros2 bag play <bag_path> --clock
% 2. Run this script
%
% Slidery dole: GPS rotace [°] a VIO scale

node = ros2node("/matlab_live_viz");

sub_gps  = ros2subscriber(node, "/f450_1/aircraft/gps_odometry", "nav_msgs/Odometry");
sub_vio  = ros2subscriber(node, "/f450_1/ov_srvins/odomimu",     "nav_msgs/Odometry");
sub_slam = ros2subscriber(node, "/f450_1/ov_srvins/points_slam", "sensor_msgs/PointCloud2");

fig = figure('Name', 'VIO Live', 'Position', [50 50 1400 720]);

% --- Slidery ---
% GPS rotace
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.05 0.08 0.025], 'String', 'GPS rot [°]:');
sl_angle = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.11 0.05 0.55 0.025], 'Min', -180, 'Max', 180, 'Value', 0);
txt_angle = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.67 0.05 0.06 0.025], 'String', '0.0°');

% VIO scale
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.015 0.08 0.025], 'String', 'VIO scale:');
sl_scale = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.11 0.015 0.55 0.025], 'Min', 0.3, 'Max', 2.0, 'Value', 1.0);
txt_scale = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.67 0.015 0.06 0.025], 'String', '1.00');

% --- Axes ---
ax_xy = subplot(2,2,[1 3]);
ax_xy.Position(2) = ax_xy.Position(2) + 0.09;
ax_xy.Position(4) = ax_xy.Position(4) - 0.09;
h_gps_xy = plot(ax_xy, NaN, NaN, 'Color', [0.2 0.4 1.0], 'LineWidth', 1.5, 'DisplayName', 'GPS');
hold(ax_xy, 'on');
h_vio_xy = plot(ax_xy, NaN, NaN, 'Color', [1.0 0.2 0.2], 'LineWidth', 1.5, 'DisplayName', 'VIO');
legend(ax_xy); grid(ax_xy, 'on'); axis(ax_xy, 'equal');
xlabel(ax_xy, 'X [m]'); ylabel(ax_xy, 'Y [m]'); title(ax_xy, 'Trajektorie XY');

ax_z = subplot(2,2,2);
ax_z.Position(2) = ax_z.Position(2) + 0.09;
ax_z.Position(4) = ax_z.Position(4) - 0.09;
h_gps_z = plot(ax_z, NaN, NaN, 'Color', [0.2 0.4 1.0], 'LineWidth', 1.5, 'DisplayName', 'GPS');
hold(ax_z, 'on');
h_vio_z = plot(ax_z, NaN, NaN, 'Color', [1.0 0.2 0.2], 'LineWidth', 1.5, 'DisplayName', 'VIO');
legend(ax_z); grid(ax_z, 'on');
xlabel(ax_z, 't [s]'); ylabel(ax_z, 'Z [m]'); title(ax_z, 'Výška Z');

ax_slam = subplot(2,2,4);
ax_slam.Position(2) = ax_slam.Position(2) + 0.09;
ax_slam.Position(4) = ax_slam.Position(4) - 0.09;
h_slam_n = animatedline(ax_slam, 'Color', [0.1 0.7 0.3], 'LineWidth', 1.5);
grid(ax_slam, 'on');
xlabel(ax_slam, 't [s]'); ylabel(ax_slam, 'počet'); title(ax_slam, 'SLAM features');
h_slam_txt = text(ax_slam, 0.02, 0.92, '0', 'Units', 'normalized', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.7 0.3]);

% --- Data ---
t0 = []; gps0 = [];
gps_x = []; gps_y = []; gps_z = []; gps_t = [];
vio_x = []; vio_y = []; vio_z = []; vio_t = [];

while ishandle(fig)
    vio_msg = sub_vio.LatestMessage;
    if isempty(vio_msg)
        pause(0.05);
        continue
    end

    t = double(vio_msg.header.stamp.sec) + double(vio_msg.header.stamp.nanosec) * 1e-9;
    if isempty(t0), t0 = t; end
    dt = t - t0;

    % Uložit VIO
    vio_x(end+1) = vio_msg.pose.pose.position.x;
    vio_y(end+1) = vio_msg.pose.pose.position.y;
    vio_z(end+1) = vio_msg.pose.pose.position.z;
    vio_t(end+1) = dt;

    % Uložit GPS
    gps_msg = sub_gps.LatestMessage;
    if ~isempty(gps_msg)
        gx = gps_msg.pose.pose.position.x;
        gy = gps_msg.pose.pose.position.y;
        gz = gps_msg.pose.pose.position.z;
        if isempty(gps0), gps0 = [gx, gy, gz]; end
        gps_x(end+1) = gx - gps0(1);
        gps_y(end+1) = gy - gps0(2);
        gps_z(end+1) = gz - gps0(3);
        gps_t(end+1) = dt;
    end

    % Přečíst slidery
    angle_deg = sl_angle.Value;
    vio_scale = sl_scale.Value;
    set(txt_angle, 'String', sprintf('%.1f°', angle_deg));
    set(txt_scale, 'String', sprintf('%.2f', vio_scale));

    % Překreslit GPS (rotace)
    if ~isempty(gps_x)
        a = deg2rad(angle_deg);
        R = [cos(a) -sin(a); sin(a) cos(a)];
        xy_rot = R * [gps_x; gps_y];
        set(h_gps_xy, 'XData', xy_rot(1,:), 'YData', xy_rot(2,:));
        set(h_gps_z,  'XData', gps_t, 'YData', gps_z);
    end

    % Překreslit VIO (scale)
    if ~isempty(vio_x)
        set(h_vio_xy, 'XData', vio_x * vio_scale, 'YData', vio_y * vio_scale);
        set(h_vio_z,  'XData', vio_t, 'YData', vio_z * vio_scale);
    end

    % SLAM počet
    slam_msg = sub_slam.LatestMessage;
    if ~isempty(slam_msg)
        n = slam_msg.width;
        addpoints(h_slam_n, dt, double(n));
        set(h_slam_txt, 'String', sprintf('%d', n));
    end

    drawnow limitrate;
end

delete(node);
