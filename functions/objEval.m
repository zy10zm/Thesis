%% Function objEval
% Evaluate objective function

%% Change log
% 22/06/2020 - ignore scanned cells in calculation
% 22/06/2020 - fixed bug with non-integers being used for matrix searches
% 22/06/2020 - fixed bug with converting m_search to same dimension as m_scan

%% To do
% - include consideration of other fire states in calculation

function [s_obj, obj] = objEval(m_f, m_bo, m_scan, ...
                        r_bo, r_f, ...
                        dt_s, s_obj, ...
                        n_x_f, n_y_f, n_x_search, n_y_search, c_f_search)

    % Generate active fire map
    m_fo          = m_f;
    m_fo(m_fo==1) = 0;
    m_fo(m_fo==2) = 0;
    m_fo(m_fo==3) = 1;
    m_fo(m_fo==4) = 0;
    
    % Inverse scan map
    m_scan_inv = ones(size(n_x_search, n_y_search)) - m_scan;
    % Convert inverse scan map to environment map resolution
    m_scan_inv_env = zeros(n_x_f, n_y_f);
    for i=1:n_x_search
      for j=1:n_y_search
        if m_scan(i,j) == 1
          % Calculate range in m_scan_inv_env
          x_min = c_f_search*(i-1) + 1;
          x_max = x_min + c_f_search - 1;
          y_min = c_f_search*(j-1) + 1;
          y_max = y_min + c_f_search - 1;
          % Set range as scanned
          m_scan_inv_env(x_min:x_max,y_min:y_max) = 1;
        end
      end
    end
    % Priority map
    m_P   = r_bo.*m_bo + r_f.*m_fo;
    % Ignore already scanned cells
    m_P   = m_P.*m_scan_inv_env;
    % Objective function for current timestep
    obj   = sum(sum(m_P))*dt_s;       
    % Sum of objective over time
    s_obj = s_obj + obj;              

end
