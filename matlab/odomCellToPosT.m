function [t, pos] = odomCellToPosT(gps_messages)
%
%   [t, pos] = odomCellToPosT(gps_messages)

    % --- position ---
    pos = cellfun(@(m) ...
        [m.pose.pose.position.x ...
         m.pose.pose.position.y ...
         m.pose.pose.position.z], ...
         gps_messages, "UniformOutput", false);

    pos = vertcat(pos{:});

    % --- time ---
    t = cellfun(@(m) double(m.header.stamp.sec) + ...
                      double(m.header.stamp.nanosec)*1e-9, ...
                gps_messages);

end


