% Simulate UAV fuzzy controller
% Date:     02/06/2020
% Author:   Craig Maxwell

%% Change log
% Removed fireModel evaluation from script

% Notes
% how to manage whether UAV is occupied? - set a certain time
% horizon after which it should have a plan? - can't make too
% long or too short.
% - figure out what outputs are required

function [UAV_target] = fisModel( n_UAV, UAV_occ, UAV_loc, UAV_target, l_queue, ...
                                  n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
                                  m_scan, m_att, m_dw, m_prior, ...
                                  fisArray, ...
                                  t_travel_UAV, t_scan_UAV, ...
                                  ang_w, v_as_UAV, v_w, ...
                                  negAtt)

    % For each UAV
    for UAV=1:n_UAV
        % Determine whether new target required
        if UAV_occ(UAV) == false
            % Reset flag
            UAV_occ(UAV) = true;

            % UAV location
            loc = UAV_loc(UAV,:);

            % Generate attraction matrix
            for i=1:n_x_search
                for j=1:n_y_search
                    if m_scan(i,j)  == 1
                        att         = negAtt;
                    else
                        fis         = fisArray(UAV);

                        % Travel time calculation - from next target to
                        % selected cell.
                        % Source - https://drstevenhale.gitlab.io/main/2008/11/triangleofvelocities/
                        dist        = (sqrt( ( l_c_s_x *(i-loc(1)) ).^2 + ( (j-loc(2)) * l_c_s_y).^2)); % Distance in m
                        ang_g       = atan2(UAV_target(UAV, 2, 1)-j, UAV_target(UAV, 1, 1)-i);                % Ground angle
                        a_wt        = ang_g - ang_w;                                                    % Wind to track angle
                        ang_wca     = asin(v_w*sin(a_wt)/v_as_UAV);                                         % Wind correction angle
                        ang_h       = ang_g + ang_wca;                                                  % Heading angle - not required
                        v_gs        = v_as_UAV*cos(ang_wca) + v_w*cos(a_wt);                                % Groundspeed
                        t_nextcell  = t_travel_UAV(UAV) + t_scan_UAV(UAV) + dist/v_gs;              % Travel time to selected cell
                        att         = evalfis(fis, [t_nextcell, m_prior(i, j), m_dw(i,j)]);
                    end
                    m_att(i, j, UAV) = att;
                end
            end
            % Assign task to UAV queue
            % Notes
            % - how to handle assignment for multiple max attraction?
            [row, col]          = find(m_att(:, :, UAV)==max(max(m_att(:, :, UAV))));
            if isnan(UAV_target(UAV, :, 1))
                if length(row) > 1
                    UAV_target(UAV, :, 1) = [row(1), col(1)];
                else
                    UAV_target(UAV, :, 1) = [row, col];
                end
            else
                % Overwrite previous queued task
                if length(row) > 1
                    UAV_target(UAV, :, 1) = [row(1), col(1)];
                else
                    UAV_target(UAV, :, 1) = [row, col];
                end
            end
        end
    end

    %% Check for conflicting targets and reschedule
    % Notes
    % - check this is done correctly
    % - add scan schedule check in later

    % Initialise map of scanning schedule to make sure cells not
    % duplicated in schedule
    m_scan_schedule = m_scan;

    % Check queue
    for q = 1:l_queue
        UAV_queue_target = UAV_target(:,:,q);

        % Update scan schedule map
        for i = 1:n_UAV
            row = UAV_queue_target(1,i);
            col = UAV_queue_target(2,i);
            % Check row and col are real numbers
            if not(isnan(row)) && not(isnan(col))
                m_scan_schedule(row, col) = 1;
            end
        end

        while length(UAV_queue_target) ~= length(unique(UAV_queue_target, 'rows'))
        % Determine conflicting targets
            for i = 1:length(UAV_queue_target)-1
                for j = i+1:length(UAV_queue_target)
                    if UAV_queue_target(i,1) == UAV_queue_target(j,1) && UAV_queue_target(i,2) == UAV_queue_target(j,2)
                        % Reassign lower attraction target
                        if m_att(UAV_queue_target(i,1), UAV_queue_target(i,1), i) >= m_att(UAV_queue_target(j,1), UAV_queue_target(j,1), j)
                            % Set attraction of cell to zero
                            m_att(UAV_queue_target(j,1), UAV_queue_target(j,1), j) = negAtt;
                            % Find indices of new max attraction and
                            % make sure it is not in schedule
                            scheduleFlag = false;
                            while scheduleFlag == false
                                [row, col]          = find(m_att(:, :, UAV)==max(max(m_att(:, :, UAV))));
                                if m_scan_schedule(row, col) == 1
                                    m_att(row, col, j) = negAtt;
                                else
                                    scheduleFlag = true;
                                end
                            end

                            if length(row) > 1
                                UAV_target(j, :, q) = [row(1), col(1)];
                            else
                                UAV_target(j, :, q) = [row, col];
                            end
                        else
                            % Set attraction of cell to zero
                            m_att(UAV_queue_target(i,1), UAV_queue_target(i,1), i) = negAtt;
                            % Find indices of new max attraction and
                            % make sure it is not in schedule
                            scheduleFlag = false;
                            while scheduleFlag == false
                                [row, col]          = find(m_att(:, :, UAV)==max(max(m_att(:, :, UAV))));
                                if m_scan_schedule(row, col) == 1
                                    m_att(row, col, j) = negAtt;
                                else
                                    scheduleFlag = true;
                                end
                            end
                            % Reassign target cell
                            if length(row) > 1
                                UAV_target(i, :, q) = [row(1), col(1)];
                            else
                                UAV_target(i, :, q) = [row, col];
                            end
                        end
                    end
                end
            end
        end
    end
end