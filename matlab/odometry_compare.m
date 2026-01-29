rosDataAnalyzer;
bag = ros2bagreader("flight_no_imu.bag");

gps_topic = "/f450_1/aircraft/gps_odometry";
gps_messages = readMessages(select(bag,"Topic",gps_topic));
[t, pos_gps] = odomCellToPosT(gps_messages);
pos_gps = pos_gps - pos_gps(1,:);

plot3(pos_gps(:,1), pos_gps(:,2), pos_gps(:,3))
grid on
axis equal
vio_topic = "/f450_1/vio_isaac/visual_slam/vis/slam_odometry";
vio_messages = readMessages(select(bag,"Topic",vio_topic));
[t, pos_vio] = odomCellToPosT(vio_messages);

plot3(pos_vio(:,1), pos_vio(:,2), pos_vio(:,3))
grid on
axis equal
R = [0 1 0; -1 0 0; 0 0 1];
pos_gps_rot = (R * pos_gps')'; % Rotace o 90° kolem z

imu_topic = "/f450_1/aircraft/imu";
imu_msgs = readMessages(select(bag,"Topic",imu_topic));

q = imu_msgs{10}.orientation;
qx = q.x; qy = q.y; qz = q.z; qw = q.w;

% yaw (Z axis)
yaw0 = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

R = [cos(-yaw0) -sin(-yaw0) 0;
     sin(-yaw0)  cos(-yaw0) 0;
     0           0          1];

pos_gps_aligned = (R * pos_gps_rot')';



figure;
plot3(pos_gps_aligned(:,1), pos_gps_aligned(:,2), pos_gps_aligned(:,3), 'LineWidth', 1.5);
hold on;
plot3(pos_vio(:,1), pos_vio(:,2), pos_vio(:,3), 'LineWidth', 1.5);
hold off;

grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('GPS', 'VIO');
title('Porovnání trajektorií: GPS vs VIO');

