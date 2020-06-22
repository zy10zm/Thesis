% Simulate UAV fuzzy controller
% Date:     02/06/2020
% Author:   Craig Maxwell

%% Change log
% 22/06/2020 - Removed fireModel evaluation from script
% 22/06/2020 - Removed UAV_occ flag - instead "NaN" can be used inside UAV tasks
% 22/06/2020 - Fixed confusing indexing in travel time calculation
% 22/06/2020 - Fixed travel time calculation
% 22/06/2020 - Expanded travel time calculation for any value of l_queue
% 22/06/2020 - Travel times saved after calculation

% Notes
% how to manage whether UAV is occupied? - set a certain time
% horizon after which it should have a plan? - can't make too
% long or too short.

% To do
% - check if any differences in sequence could cause clash in algorithm logic -
% e.g. two UAVs visiting same cell
% - check for bugs

function [UAV_target] = pathPlanner(n_UAV, UAV_target, l_queue, ...
                        n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
                        m_scan, m_t_scan, m_att, m_dw, m_prior, ...
                        fisArray, ...
                        t_travel_UAV, t_scan_UAV, ...
                        ang_w, v_as_UAV, v_w, ...
                        negAtt)
  
  % Scan schedule map
  m_scan_schedule   = zeros(n_x_search, n_y_search);
  
	for q = 1:l_queue
    for UAV=1:n_UAV      
      % Only reallocate first target if it is not assigned
      if q == 1 && ~isnan(UAV_target(UAV, 1, q))
        m_scan_schedule(UAV_target(UAV, 1, q), UAV_target(UAV, 2, q)) = 1;
      else
        % Generate attraction matrix
        for i=1:n_x_search
          for j=1:n_y_search
            % Negative attraction for scanned or scheduled cells
            if m_scan(i,j) == 1 || m_scan_schedule(i,j) == 1
              att = negAtt;
            else
              % Travel time for current task
              t_nextcell = t_travel_UAV(UAV) + t_scan_UAV(UAV);
              % Travel time for tasks in queue - check indexing correct for this
              % section
              for p = 1:q
                loc_1 = UAV_target(UAV, :, p);  % Start location
                loc_2 = [i, j];                 % End location
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
                t_nextcell  = t_nextcell ...
                            + dist/v_gs + m_t_scan(loc_2(1), loc_2(2));
              end
              % Find attraction
              fis = fisArray(UAV);              
              att = evalfis(fis, [t_nextcell, m_prior(i, j), m_dw(i,j)]);              
            end
            m_att(i, j, UAV) = att;
          end
        end
        
        % Find max attraction cells
        [row, col] = find(m_att(:, :, UAV) == max(max(m_att(:, :, UAV))));
        % Assign first max attraction in case of duplicates
        UAV_target(UAV, :, q) = [row(1), col(1)];
        % Update scan schedule map
        m_scan_schedule(row(1), col(1)) = 1;
      end
    end

    %% Check for conflicting targets and reschedule
    % Determine conflicting targets
    while length(UAV_target(:,:,q)) ~= length(unique(UAV_target(:,:,q), 'rows'))
      for UAV_1=1:n_UAV
        for UAV_2=1:n_UAV
          if UAV_1 ~= UAV_2
            UAV_1_target = UAV_target(UAV_1, :, q);
            UAV_2_target = UAV_target(UAV_2, :, q);
            if UAV_1_target == UAV_2_target
              % Reassign UAV with lower attraction
              if m_att(UAV_1_target(1), UAV_1_target(2), i) >=  m_att(UAV_2_target(1), UAV_2_target(2), j)
                % Reassign UAV 2
                m_att(UAV_2_target(1), UAV_2_target(2), UAV_2) = negAtt;               
                % Find max attraction cells
                [row, col] = find(m_att(:, :, UAV_2) == max(max(m_att(:, :, UAV_2))));
                % Assign first max attraction in case of duplicates
                UAV_target(UAV_2, :, q) = [row(1), col(1)];
                % Update scan schedule map
                m_scan_schedule(row(1), col(1)) = 1;
              else
                % Reassign UAV 1
                m_att(UAV_1_target(1), UAV_1_target(2), UAV_1) = negAtt;               
                % Find max attraction cells
                [row, col] = find(m_att(:, :, UAV_1) == max(max(m_att(:, :, UAV_1))));
                % Assign first max attraction in case of duplicates
                UAV_target(UAV_1, :, q) = [row(1), col(1)];
                % Update scan schedule map
                m_scan_schedule(row(1), col(1)) = 1;
              end
            end
          end        
        end
      end
    end
  end
end

%% Errata
% Original task allocation algorithm
%     if isnan(UAV_target(UAV, :, 1))
%       if length(row) > 1
%         UAV_target(UAV, :, 1) = [row(1), col(1)];
%       else
%         UAV_target(UAV, :, 1) = [row, col];
%       end
%     else
%       % Overwrite previous queued task
%       if length(row) > 1
%         UAV_target(UAV, :, 1) = [row(1), col(1)];
%       else
%         UAV_target(UAV, :, 1) = [row, col];
%       end
%     end

% if m_att(UAV_queue_target(i,1), UAV_queue_target(i,1), i) >= m_att(UAV_queue_target(j,1), UAV_queue_target(j,1), j)
%               % Set attraction of cell to zero
%               m_att(UAV_queue_target(j,1), UAV_queue_target(j,1), j) = negAtt;
%               % Find indices of new max attraction and
%               % make sure it is not in schedule
%               scheduleFlag = false;
%               while scheduleFlag == false
%                 [row, col]          = find(m_att(:, :, UAV)==max(max(m_att(:, :, UAV))));
%                 if m_scan_schedule(row, col) == 1
%                   m_att(row, col, j) = negAtt;
%                 else
%                   scheduleFlag = true;
%                 end
%               end
% 
%               if length(row) > 1
%                 UAV_target(j, :, q) = [row(1), col(1)];
%               else
%                 UAV_target(j, :, q) = [row, col];
%               end
%             else
%               % Set attraction of cell to zero
%               m_att(UAV_queue_target(i,1), UAV_queue_target(i,1), i) = negAtt;
%               % Find indices of new max attraction and
%               % make sure it is not in schedule
%               scheduleFlag = false;
%               while scheduleFlag == false
%                 [row, col]          = find(m_att(:, :, UAV)==max(max(m_att(:, :, UAV))));
%                 if m_scan_schedule(row, col) == 1
%                   m_att(row, col, j) = negAtt;
%                 else
%                   scheduleFlag = true;
%                 end
%               end
%               % Reassign target cell
%               if length(row) > 1
%                 UAV_target(i, :, q) = [row(1), col(1)];
%               else
%                 UAV_target(i, :, q) = [row, col];
%               end
%             end
%           end
%         end
%       end
%       end
