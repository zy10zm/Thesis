% Simulate UAV system
% Date:     02/06/2020
% Author:   Craig Maxwell

% To do:
% Improve calculation of agent times - linear so can increase step size
% Queue should always ensure that time to complete is greater than
% control timestep size!

%% Change Log
% 22/06/2020 - reworked UAV task assignment
% 22/06/2020 - corrected UAV distance calculations
% 22/06/2020 - improved readability

function [  m_scan, m_scan_hist, m_scan_t_hist, ...
            UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
            t_travel_UAV, t_scan_UAV] ...
            = uavModel( n_UAV, ...
                m_t_scan, m_scan, m_scan_hist, m_scan_t_hist, ...
                UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
                t_travel_UAV, t_scan_UAV, ...
                l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t)

  for UAV = 1:n_UAV
    if UAV_task(UAV) == 1
      % Travel
      t_travel_UAV(UAV) = t_travel_UAV(UAV) - dt_a;
      if t_travel_UAV(UAV) <= 0
        % Set task to scan
        UAV_task(UAV)   = 2;                            
        % Update UAV location
        UAV_loc(UAV, :) = UAV_target(UAV, :, 1);        
        UAV_loc_hist    = [UAV_loc_hist; UAV_loc(UAV, :), UAV, t];        
        % Initialise remaining scantime
        t_scan_UAV(UAV) = m_t_scan(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) + t_travel_UAV(UAV);
      end
    else
      % Scan
      t_scan_UAV(UAV) = t_scan_UAV(UAV) - dt_a;
      if t_scan_UAV(UAV) <= 0
        % Set task to travel
        UAV_task(UAV) = 1;
        % Set cell to scanned
        m_scan(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) = 1; 
        % Update scan map history
        m_scan_hist   = [m_scan_hist; m_scan];        
        % Update scan map time history
        m_scan_t_hist = [m_scan_t_hist; t];
        
        % Assign targets
        UAV_target(UAV, 1, :)   = circshift(UAV_target(UAV, 1, :), -1);
        UAV_target(UAV, 2, :)   = circshift(UAV_target(UAV, 2, :), -1);        
        UAV_target(UAV, :, end) = [NaN, NaN];
        
        % Start location
        loc_1 = UAV_loc(UAV, :);  
        % End location
        loc_2 = UAV_target(UAV, :, 1);                  
        % Distance in m
        dist        = (sqrt ((l_c_s_x*(loc_2(1) - loc_1(1))).^2) ...
                    +       ((l_c_s_y*(loc_2(2) - loc_1(2))).^2));
        % Ground angle
        ang_g       = atan2(loc_1(2) - loc_2(2), loc_1(1) - loc_2(1));                % Ground angle
        % Wind to track angle
        a_wt        = ang_g - ang_w;
        % Wind correction angle
        ang_wca     = asin(v_w*sin(a_wt)/v_as_UAV);
        % Ground speed
        v_gs        = v_as_UAV*cos(ang_wca) + v_w*cos(a_wt);                
        % Add to travel time
        t_travel_UAV(UAV) = dist/v_gs;
      end
    end
  end
end
