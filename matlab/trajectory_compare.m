%% trajectory_compare.m
% Skript pro porovnání GPS a VIO trajektorií z více rosbagů
% Autor: Martin Kriz
%
% Použití:
%   1. Nastavte cesty k bagům v sekci KONFIGURACE
%   2. Spusťte skript
%   3. Každý bag se zobrazí v samostatném okně

clear all
close all

%% KONFIGURACE
% Základní cesta k rosbagům (změň podle svého systému)
rosbag_base_path = fullfile(fileparts(mfilename('fullpath')), '..', 'rosbags');
% Alternativně absolutní cesta:
% rosbag_base_path = "/home/martin/ros2_ws/src/visual_inertial_odometry/rosbags";

% Cesta k referenčnímu bagu (obsahuje GPS referenci)
reference_bag_path = fullfile(rosbag_base_path, "winter_flight1/flight_no_imu.bag");

% Seznam bagů k porovnání (každý se zobrazí v samostatném okně)
bags_to_compare = {
    struct('path', fullfile(rosbag_base_path, "winter_flight1/flight_no_imu.bag"), 'name', "No IMU fusion")
    struct('path', fullfile(rosbag_base_path, "winter_flight1/flight_with_imu.bag"), 'name', "With IMU fusion")
    struct('path', fullfile(rosbag_base_path, "winter_flight1/flight_lowpass1hz_madwick0_01"), 'name', "Lowpass 1Hz + Madwick 0.01")
};

% Topic names
gps_topic = "/f450_1/aircraft/gps_odometry";
vio_topic = "/f450_1/vio_isaac/visual_slam/vis/slam_odometry";
imu_topic = "/f450_1/aircraft/imu";

% Manuální korekce rotace GPS kolem osy Z [stupně]
% Uprav hodnotu dokud trajektorie nesedí
gps_yaw_offset_deg = -22;

%% NAČTENÍ REFERENČNÍHO BAGU PRO YAW ALIGNMENT
fprintf('Načítám referenční bag: %s\n', reference_bag_path);
ref_bag = ros2bagreader(reference_bag_path);

% Získání počátečního yaw z IMU pro zarovnání GPS
imu_msgs = readMessages(select(ref_bag, "Topic", imu_topic));
if ~isempty(imu_msgs)
    q = imu_msgs{10}.orientation;
    yaw0 = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
else
    yaw0 = 0;
    warning('IMU data nenalezena, yaw alignment přeskočen');
end

%% ZPRACOVÁNÍ KAŽDÉHO BAGU
for i = 1:length(bags_to_compare)
    bag_info = bags_to_compare{i};
    fprintf('Zpracovávám bag %d/%d: %s\n', i, length(bags_to_compare), bag_info.name);

    try
        % Načtení bagu
        bag = ros2bagreader(bag_info.path);

        % Načtení GPS dat
        gps_msgs = readMessages(select(bag, "Topic", gps_topic));
        [t_gps, pos_gps] = odomCellToPosT(gps_msgs);
        pos_gps = pos_gps - pos_gps(1,:);  % Normalizace na počátek

        % Načtení VIO dat
        vio_msgs = readMessages(select(bag, "Topic", vio_topic));
        [t_vio, pos_vio] = odomCellToPosT(vio_msgs);

        % Zarovnání GPS - rotace o 90° (NED -> ENU), yaw z IMU a manuální offset
        R_90 = [0 1 0; -1 0 0; 0 0 1];
        yaw_total = -yaw0 + deg2rad(gps_yaw_offset_deg);
        R_yaw = [cos(yaw_total) -sin(yaw_total) 0;
                 sin(yaw_total)  cos(yaw_total) 0;
                 0               0              1];
        pos_gps_aligned = (R_yaw * R_90 * pos_gps')';

        %% VÝPOČET METRIK
        % Interpolace VIO na časy GPS pro porovnání
        t_gps_norm = t_gps - t_gps(1);
        t_vio_norm = t_vio - t_vio(1);

        % Najdi společný časový rozsah
        t_start = max(t_gps_norm(1), t_vio_norm(1));
        t_end = min(t_gps_norm(end), t_vio_norm(end));
        t_common = t_gps_norm(t_gps_norm >= t_start & t_gps_norm <= t_end);

        % Interpolace obou trajektorií na společné časy
        pos_gps_interp = interp1(t_gps_norm, pos_gps_aligned, t_common);
        pos_vio_interp = interp1(t_vio_norm, pos_vio, t_common);

        % 1. ATE - Absolute Trajectory Error (po zarovnání počátků)
        pos_vio_interp_aligned = pos_vio_interp - pos_vio_interp(1,:) + pos_gps_interp(1,:);
        ate_errors = vecnorm(pos_gps_interp - pos_vio_interp_aligned, 2, 2);
        ATE_rmse = sqrt(mean(ate_errors.^2));
        ATE_mean = mean(ate_errors);
        ATE_max = max(ate_errors);

        % 2. RPE - Relative Pose Error (chyba mezi po sobě jdoucími pozicemi)
        delta_gps = diff(pos_gps_interp);
        delta_vio = diff(pos_vio_interp_aligned);
        rpe_errors = vecnorm(delta_gps - delta_vio, 2, 2);
        RPE_rmse = sqrt(mean(rpe_errors.^2));
        RPE_mean = mean(rpe_errors);

        % 3. Celková ujetá vzdálenost
        total_dist_gps = sum(vecnorm(diff(pos_gps_interp), 2, 2));
        total_dist_vio = sum(vecnorm(diff(pos_vio_interp_aligned), 2, 2));

        % 4. Drift rate [%] - koncová chyba / ujetá vzdálenost
        final_error = norm(pos_gps_interp(end,:) - pos_vio_interp_aligned(end,:));
        drift_rate = (final_error / total_dist_gps) * 100;

        % 5. Loop closure error (vzdálenost koncového bodu od počátku)
        loop_closure_gps = norm(pos_gps_interp(end,:) - pos_gps_interp(1,:));
        loop_closure_vio = norm(pos_vio_interp_aligned(end,:) - pos_vio_interp_aligned(1,:));

        % Výpis metrik do konzole
        fprintf('\n=== METRIKY: %s ===\n', bag_info.name);
        fprintf('ATE (RMSE):     %.3f m\n', ATE_rmse);
        fprintf('ATE (mean):     %.3f m\n', ATE_mean);
        fprintf('ATE (max):      %.3f m\n', ATE_max);
        fprintf('RPE (RMSE):     %.4f m\n', RPE_rmse);
        fprintf('RPE (mean):     %.4f m\n', RPE_mean);
        fprintf('Drift rate:     %.2f %%\n', drift_rate);
        fprintf('Total dist GPS: %.1f m\n', total_dist_gps);
        fprintf('Total dist VIO: %.1f m\n', total_dist_vio);
        fprintf('Loop closure GPS: %.2f m\n', loop_closure_gps);
        fprintf('Loop closure VIO: %.2f m\n', loop_closure_vio);
        fprintf('================================\n');

        %% FIGURE 1: Trajektorie
        figure('Name', sprintf('Trajektorie: %s', bag_info.name), 'NumberTitle', 'off');

        % 3D plot
        subplot(2,2,[1,3]);
        h1 = plot3(pos_gps_aligned(:,1), pos_gps_aligned(:,2), pos_gps_aligned(:,3), ...
              'b-', 'LineWidth', 1.5);
        hold on;
        h2 = plot3(pos_vio(:,1), pos_vio(:,2), pos_vio(:,3), ...
              'r-', 'LineWidth', 1.5);
        hold off;
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
        legend([h1, h2], {'GPS (ref)', 'VIO'}, 'Location', 'best');
        title(sprintf('3D trajektorie: %s', bag_info.name));
        view(45, 30);

        % XY plot (top view)
        subplot(2,2,2);
        h1 = plot(pos_gps_aligned(:,1), pos_gps_aligned(:,2), 'b-', 'LineWidth', 1.5);
        hold on;
        h2 = plot(pos_vio(:,1), pos_vio(:,2), 'r-', 'LineWidth', 1.5);
        hold off;
        grid on; axis equal;
        xlabel('X [m]'); ylabel('Y [m]');
        title('Pohled shora (XY)');
        legend([h1, h2], {'GPS (ref)', 'VIO'}, 'Location', 'best');

        % Z vs time
        subplot(2,2,4);
        h1 = plot(t_gps_norm, pos_gps_aligned(:,3), 'b-', 'LineWidth', 1.5);
        hold on;
        h2 = plot(t_vio_norm, pos_vio(:,3), 'r-', 'LineWidth', 1.5);
        hold off;
        grid on;
        xlabel('Čas [s]'); ylabel('Z [m]');
        title('Výška vs čas');
        legend([h1, h2], {'GPS (ref)', 'VIO'}, 'Location', 'best');

        %% FIGURE 2: Vizualizace metrik
        figure('Name', sprintf('Metriky: %s', bag_info.name), 'NumberTitle', 'off');

        % ATE v čase
        subplot(2,2,1);
        plot(t_common, ate_errors, 'r-', 'LineWidth', 1.2);
        hold on;
        yline(ATE_mean, 'k--', 'LineWidth', 1.5);
        yline(ATE_rmse, 'b--', 'LineWidth', 1.5);
        hold off;
        grid on;
        xlabel('Čas [s]'); ylabel('ATE [m]');
        title('Absolute Trajectory Error v čase');
        legend({'ATE', sprintf('Mean: %.3f m', ATE_mean), sprintf('RMSE: %.3f m', ATE_rmse)}, ...
               'Location', 'best');

        % Histogram ATE
        subplot(2,2,2);
        histogram(ate_errors, 30, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'white');
        hold on;
        xline(ATE_mean, 'r-', 'LineWidth', 2);
        xline(ATE_rmse, 'g-', 'LineWidth', 2);
        hold off;
        grid on;
        xlabel('ATE [m]'); ylabel('Počet');
        title('Distribuce ATE');
        legend({'Histogram', sprintf('Mean: %.3f', ATE_mean), sprintf('RMSE: %.3f', ATE_rmse)}, ...
               'Location', 'best');

        % Kumulativní chyba (drift)
        subplot(2,2,3);
        cum_dist = [0; cumsum(vecnorm(diff(pos_gps_interp), 2, 2))];
        plot(cum_dist, ate_errors, 'm-', 'LineWidth', 1.2);
        grid on;
        xlabel('Ujetá vzdálenost [m]'); ylabel('ATE [m]');
        title(sprintf('ATE vs vzdálenost (Drift: %.2f%%)', drift_rate));

        % Chyba po osách
        subplot(2,2,4);
        error_xyz = pos_gps_interp - pos_vio_interp_aligned;
        plot(t_common, error_xyz(:,1), 'r-', 'LineWidth', 1.2); hold on;
        plot(t_common, error_xyz(:,2), 'g-', 'LineWidth', 1.2);
        plot(t_common, error_xyz(:,3), 'b-', 'LineWidth', 1.2);
        hold off;
        grid on;
        xlabel('Čas [s]'); ylabel('Chyba [m]');
        title('Chyba po osách');
        legend({'X', 'Y', 'Z'}, 'Location', 'best');

    catch ME
        warning('Chyba při zpracování bagu %s: %s', bag_info.path, ME.message);
    end
end

fprintf('Hotovo!\n');
