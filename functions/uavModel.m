% Simulate UAV system
% Date:     02/06/2020
% Author:   Craig Maxwell

% Notes:
% Improve calculation of agent times - linear so can increase step size
% Queue should always ensure that time to complete is greater than
% control timestep size!

function [  m_scan, m_scan_hist, m_scan_t_hist, ...
            UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
            t_travel_UAV, t_scan_UAV] ...
            = uavModel( n_UAV, ...
                        m_t_scan, m_scan, m_scan_hist, m_scan_t_hist, ...
                        UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
                        t_travel_UAV, t_scan_UAV, ...
                        l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t)
    for UAV = 1:n_UAV
        switch UAV_task(UAV)
            % 1 = travel
            case 1
                t_travel_UAV(UAV) = t_travel_UAV(UAV) - dt_a;
                if t_travel_UAV(UAV) <= 0
                    UAV_task(UAV)   = 2;                            % Set task to scan
                    UAV_loc(UAV, :) = UAV_target(UAV, :, 1);          % Update UAV location
                    UAV_loc_hist    = [UAV_loc_hist; UAV_loc(UAV, :), UAV, t];

                    % Initialise remaining scantime
                    t_scan_UAV(UAV) = m_t_scan(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) + t_travel_UAV(UAV);
                end
            % 2 = scan
            case 2
                t_scan_UAV(UAV) = t_scan_UAV(UAV) - dt_a;
                if t_scan_UAV(UAV) <= 0
                    UAV_task(UAV) = 1;                            % Set task to travel
                    m_scan(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) = 1;   % Set cell to scanned
                    m_scan_hist = [m_scan_hist; m_scan];        % Scan map history
                    m_scan_t_hist = [m_scan_t_hist; t];         % Scan map time history

                    % Initialise remaining travel time
                    % Notes
                    % - is this where to manage this?
                    UAV_target(UAV, :, 1) = UAV_target(UAV, :, 2);
                    UAV_target(UAV, :, 2) = NaN;
                    dist        = sqrt( ( l_c_s_x * (UAV_target(UAV,1,1) - UAV_loc(UAV,1)).^2 ...
                                + ( l_c_s_y * (UAV_target(UAV,2,1) - UAV_loc(UAV,2))).^2));
                    ang_g       = atan2(UAV_target(UAV, 2, 1)-UAV_loc(UAV, 2), ...
                                        UAV_target(UAV, 1, 1)-UAV_loc(UAV, 1));     % Ground angle
                    a_wt        = ang_g - ang_w;                                    % Wind to track angle
                    ang_wca     = asin(v_w*sin(a_wt)/v_as_UAV);                     % Wind correction angle
                    v_gs        = v_as_UAV*cos(ang_wca) + v_w*cos(a_wt);            % Groundspeed                        
                    t_travel_UAV(UAV) = dist/v_gs + t_scan_UAV(UAV);
                end
        end
    end
end

%% Errata
%             % Update flags
%             % Notes
%             % - needs development
%             if UAV_occ(UAV) == true
%                 if UAV_t_occ <= t_occ_horz
%                     UAV_occ(UAV) = false;
%                 end
%             end