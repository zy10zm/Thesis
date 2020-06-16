%% To do
% remove variables which don't need to be defined here
% create all inputs

function [] = initialiseSim_01()
  %% Import data
  buildingRaster      = 'inputData\maps\portAuPrince\portAuPrince_campeche.tif';
  [m_p_in, m_p_ref]   = geotiffread(buildingRaster);
  m_p_in              = m_p_in(size(m_p_in,1)/4:size(m_p_in,1)/2, size(m_p_in,2)/4:size(m_p_in,2)/2); % Select desired area

  %% Variables

  % Time steps and counters
  t       = 0;        % Current time (s)
  t_f     = 600;      % Simulation end time (s) - optional end condition
  k       = 1;        % Discrete time step counter
  dt_s    = 6;        % Minimum step size (s)
  dt_a    = 6;        % Agent step size (s)
  dt_c    = 600;      % Control step size (s)
  dt_f    = 60;       % Fire step size (s)
  dt_mpc  = 6000;     % MPC step size (s)
  dt_p    = 60;       % Prediction step size (s)
  dt_v    = 60;       % Save variable step size (s)
  ct_a    = 0;        % Agent counter
  ct_c    = 0;        % Control counter
  ct_f    = 0;        % Fire counter
  ct_mpc  = 0;        % MPC counter
  ct_p    = 0;        % Prediction counter
  ct_v    = 0;        % Save variable counter
  n_p     = 3;        % Prediction horizon (= n_p*dt_p)s

  % Simulation timer
  ct_prog       = 0;  % Progress report counter
  dt_prog       = 10; % Progress report interval (s real time)

  % Flags
  endCondition  = "time"; % Simulation end condition - "time" or "scan"
  finishFlag    = false;

  % Objective function
  obj           = 0;
  s_obj         = 0;
  s_obj_hist    = s_obj; % History of s_obj

  % Quadcopter model
  % Notes:
  % UAV_loc_hist handled completely wrong - fix!
  n_UAV         = 2;                % Number of UAVs in simulation
  l_queue       = 2;                % Queue length for UAV tasks
  v_as_UAV      = 10;               % UAV airspeed (m/s)
  t_travel_UAV  = zeros(n_UAV, 1);  % Time left to complete travel
  t_scan_m      = 1;                % Scan time per square metre
  t_scan_UAV    = nan(n_UAV, 1);   % Time left to complete current scanning task
  UAV_occ       = false(n_UAV, 1);          % Flag for if UAV is occupied
  UAV_path      = ones(n_UAV, 2);           % Path of UAVs
  UAV_task      = ones(n_UAV, 1);           % Current task for UAVs
  UAV_target    = nan(n_UAV, 2, l_queue);   % Current target queue for UAVs - can queue up to 2 targets
  UAV_loc       = [ 1, 1;
                    1, 2];                  % Current locations of UAVs
  UAV_loc_hist  = zeros(1, 4);              % [loc(1), loc(2), UAV, t]
  for i = 1:n_UAV
      UAV_loc_hist(i,:) = [UAV_loc(i, 1), UAV_loc(i, 2), i, t];
  end

  % Risk model
  r_bo        = 0.5;         % Risk weighting due to building occupancy
  r_f         = 0.5;         % Risk weighting due to environmental fire

  % Wind variables
  % Notes
  % - tune wind model to point where system can adapt behaviour accordingly
  v_w         = 4;        % Wind speed (m/s)
  ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  c_w1        = 0.2;      % Wind constant 1 (for fire model)
  c_w2        = 0.2;      % Wind constant 2 (for fire model)

  % State maps
  l_c_state   = 3;                        % dynamic state cell size (m)
  [m_bo, m_s, l_c_f_x, l_c_f_y] = coarsen(m_p_in, m_p_ref, l_c_state);
  n_x_f       = size(m_bo,1);             %
  n_y_f       = size(m_bo,2);             %
  n_n_s       = t_f/dt_f;                 % Number of steps in simulation
  n_f_i       = 5;                        % Number of initial fire outbreaks
  m_f_i       = zeros(n_x_f, n_y_f);      % Initial fire map
  m_bt        = zeros(n_x_f,n_y_f);       % Burntime map
  m_bt_hist   = zeros(n_x_f, n_y_f);      % Burntime map history
  m_dw        = zeros(n_x_f, n_y_f);      % Downwind map
  m_dw_hist   = zeros(n_x_f, n_y_f);      % Downwind map history
  m_f_hist    = zeros(n_x_f, n_y_f);      % Fire map history
  % d_max_scan  = sqrt(n_x_f.^2 + n_y_f.^2);% Diagonal distance of scan map (cells)

  % Search maps
  l_c_search        = 4*l_c_state;
  [m_search, ~, l_c_s_x, l_c_s_y] = coarsen(m_p_in, m_p_ref, l_c_search);
  t_scan_s          = t_scan_m*l_c_s_x*l_c_s_y;             % Scan time per search map cell
  n_x_search        = size(m_search, 1);                    % Scan map size in x direction
  n_y_search        = size(m_search, 2);                    % Scan map size in y direction
  m_att             = zeros(n_x_search, n_y_search, n_UAV); % Attraction map
  m_dist            = zeros(n_x_search, n_y_search, n_UAV); % Distance map
  m_scan            = zeros(n_x_search, n_y_search);        % Scan map
  m_scan_hist       = zeros(n_x_search, n_y_search);        % Scan map history
  m_scan_t_hist     = 0;                                    % Scan map time history - times at which scan map is updated.
  m_t_scan          = t_scan_s.*ones(n_x_search, n_y_search); % Scan time map (s) - time to scan each cell

  % Priority map
  % Method 1 - Assign constant to non-occupied cells and coarsen
  negAtt            = NaN; % Large negative attraction for scan map cells
  c_prior_building  = 1;    % Priority constant for building
  c_prior_open      = 0.1;  % Priority constant for open space
  m_prior           = zeros(n_x_search, n_y_search);
  for i = 1:n_x_search
    for j = 1:n_y_search
      m_prior(i,j) = c_prior_building*m_search(i,j) + c_prior_open*(1-m_search(i,j));
    end
  end

  % Fire outbreak method 2 - n fire outbreaks in random buildings.
  rng(1) % Seed to ensure consistency of initial firemap generation
  for i = 1:n_x_f
  for j = 1:n_y_f
      if m_bo(i,j) ~= 0
        m_f_i(i,j) = 1;
      end
    end
  end
  while n_f_i > 0
    coords = [randi([1 n_x_f]), randi([1 n_y_f])];
    if m_bo(coords(1), coords(2)) > 0
      m_f_i(coords(1), coords(2)) = 2;
      n_f_i = n_f_i - 1;
    end
  end
  m_f = m_f_i;
end