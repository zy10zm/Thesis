%% Main simulation script

%% To do
% Include txt file in data and figure exports which describes simulation
% Fix fuzzy rules not firing
% Tune parameters:
% - fire spread - too fast currently.
% - number of actions to plan ahead for each agent
% - scale of fuzzy inputs
% - risk model weights
% - wind and fire spread model weights
% - membership function parameters 
% Implement multiple simulation functionality.
% Add earthquake data to model - https://www.gislounge.com/haiti-maps-and-gis-data-resources/
% Add selection criteria for which cells to consider in path-planning algorithm
% for each UAV - n closest cells instead of entire map.
% Get simulation working for UAV's without MPC part
% Implement error checking / code termination condition script.
% Initial optimisation of output MF parameters
% - How to go about designing this optimisation?
% - How to select and design rule base and output MFs?
% - Create three FIS systems - with 1, 2, and 3 output MF
% - # optimisation variables = (inputs+1)*nUAVs (MF parameters)
%       -- also some dependency on rule base - changes how MFs interact.
% Initialise UAV_target at start of simulation
% Function that calculates number of optimisation variables and
% fuzzy rules for entire FIS
% Check downwind proximity calculation - is low value for nearby or
% far
% Add full descriptions for models, functions, etc. Also expand
% assumptions list. Add readme.
% Functions to make sure data arrays don't grow too large?
% Implement wind speed noise model.

%% Assumptions
% - Minimise risk to victims until the point at which they are rescued.
% Assumes victims are rescued quickly after detection and little extra risk
% to them during that time. Also doesn't account for additional factors
% which may be important such as equipment / process required to rescue
% victims.
% - Wind is assumed to be in fixed direction and velocity

%% Bugs
% UAVs seem to scan one cell and then do nothing

%% Change log
% Moved code from main to separate functions
% Implemented new tab spacing across entire script
% Added fire component to victim risk calculation
% - constant weighting added while fire is active
% Designed input for FIS to characterise fire spread
% - downwindfire proximity - linear function of distance and windspeed related to nearest active fire
% Implemented fire spread probability matrix based on wind speed.
% Fixed bug where fuzzy rules were not being fired.
% Plot changes:
% - animated plots
% - added lat/lon to axes
% - multiplot
% Implemented separate timesteps for all components of the simulation - MPC,
% FIS, Control, Dynamics.
% Implemented UAV actions independent of others.
% Implemented model for scan cell priority - constant function of building area
% coverage
% Implemented function to save figures and simulation data to specified folder.
% Implemented simulation progress report

%% Potential changes
% Variable scan time
% - based on cell size or cell occupancy
% Load variables from mat or m file
% Include seeded victim locations
% Noise model for wind direction
% Implement categories to change log - bug fix / feature / etc

%% Clear workspace
clear all;

%% Preprocessing

% Setup paths
addpath('functions', 'inputData', 'outputData', 'figures')

% Import building raster
buildingRaster      = 'inputData\maps\portAuPrince\portAuPrince_campeche.tif';
[m_p_in, m_p_ref]   = geotiffread(buildingRaster);
m_p_in              = m_p_in(size(m_p_in,1)/4:size(m_p_in,1)/2, size(m_p_in,2)/4:size(m_p_in,2)/2); % Select desired area

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

% % Import simulation data
% [] = initialiseSim_01();

% Generate FIS
[fisArray] = genFis_01( n_UAV );

%% Simulation
while finishFlag == false

  % Start simulation timer
  tic
  
  %% Environment model
  if ct_f*dt_f <= t
    % Update firemap and downwind map
    [m_f, m_bt, m_dw] = fireModel(  m_f, m_s, m_bo, m_bt, ...
                                    dt_f, k, n_x_f, n_y_f, ...
                                    v_w, ang_w, ...
                                    c_w1, c_w2);
    ct_f = ct_f + 1;
  end

  %% MPC
  if ct_mpc*dt_mpc <= t
    ct_mpc = ct_mpc + 1;

%     [outputs] = mpcModel(inputs); 
  end 

  %% Path planning
  if ct_c*dt_c <= t
    ct_c = ct_c + 1;
    [UAV_target] = fisModel(n_UAV, UAV_occ, UAV_loc, UAV_target, l_queue, ...
                            n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
                            m_scan, m_att, m_dw, m_prior, ...
                            fisArray, ...
                            t_travel_UAV, t_scan_UAV, ...
                            ang_w, v_as_UAV, v_w, ...
                            negAtt);
  end

  %% Agent actions
  if ct_a*dt_a <= t
    ct_a = ct_a + 1;
    [   m_scan, m_scan_hist, m_scan_t_hist, ...
        UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
        t_travel_UAV, t_scan_UAV] ...
                    = uavModel( n_UAV, ...
                    m_t_scan, m_scan, m_scan_hist, m_scan_t_hist, ...
                    UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
                    t_travel_UAV, t_scan_UAV, ...
                    l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t);
  end


  %% Store variables
  if ct_v*dt_v <= t
    ct_v = ct_v + 1;
    m_dw_hist(:,:,ct_v) = m_dw;
    m_bt_hist(:,:,ct_v) = m_bt;
    m_f_hist(:,:,ct_v)  = m_f;
    s_obj_hist(ct_v)    = s_obj;
  end

  %% Objective function evaluation
  [s_obj, obj] = objEval(m_f, m_bo, r_bo, r_f, dt_s, s_obj);

  %% Advance timestep
  t = t + dt_s;
  k = k + 1;

  %% Progress report
  if ct_prog*dt_prog <= toc
    ct_prog = ct_prog + 1;
    progReport(endCondition, t, t_f, m_scan, n_x_search, n_y_search);
  end
  
  %% Check end condition
  [finishFlag] = simEndCondition(endCondition, t, t_f, m_scan, n_x_search, n_y_search);
end

%% Postprocessing
% To do
% - check file / directory management works as intended
% - create all required colormaps

% Generate and export plots (in progress)
% Axes may not be entirely accurate as coarsening may remove some
% rows/columns from original map.

% Axes for dynamic environment states
ax_lat = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_f);
ax_lon = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_f);

% Axes for search map
ax_lat_scan = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_search);
ax_lon_scan = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_search);

% Colourmaps
m_f_colourMap       = [ 1,   1,   1;    % 0
                        0.5, 0.5, 0.5;  % 1
                        1,   0.5, 0;    % 2
                        1,   0,   0;    % 3
                        0,   0,   0];   % 4
m_s_colourMap       = [ 1, 1, 1;        % 0
                        0, 0, 0];       % 1

plot_exp_saveDir   = "test";
data_exp_folder    = "figures";
plot_exp_fileName  = "testSim";
% plotData  = {  
%         'm_bt_hist',        m_bt_hist,        true;
%         'm_dw_hist',        m_dw_hist,        true;    
%         'm_f_hist',         m_f_hist,         true;
%         'm_scan_hist',      m_scan_hist,      true;
%         'm_scan_t_hist',    m_scan_t_hist,    true;
%         'UAV_loc_hist',     UAV_loc_hist,     true;
%         'fis_param_hist',   fis_param_hist,   true;
%         'sum_obj_hist',     sum_obj_hist,     true;
%         'opt_res_hist',     opt_res_hist,     true;
%         'fis',              fisArray,         true};
plotData  = {
        'm_bt_hist',        m_bt_hist,        true;
        'm_dw_hist',        m_dw_hist,        true;    
        'm_f_hist',         m_f_hist,         true;
        'fis',              fisArray,         true};
genPlots( plotData, plot_exp_saveDir, data_exp_folder, ...
          ax_lon, ax_lat, ax_lon_scan, ax_lat_scan, ...
          m_f_colourMap);

% Export data
data_exp_saveDir  = "test";
data_exp_folder   = "data";
data_exp_fileName = "testSim";
% data = {  
%         'm_bt_hist',        m_bt_hist,        true;
%         'm_dw_hist',        m_dw_hist,        true;
%         'm_f_hist',         m_f_hist,         true;
%         'm_scan_hist',      m_scan_hist,      true;
%         'm_scan_t_hist',    m_scan_t_hist,    true;
%         'UAV_loc_hist',     UAV_loc_hist,     true;
%         'fis_param_hist',   fis_param_hist,   true;
%         'sum_obj_hist',     sum_obj_hist,     true;
%         'opt_res_hist',     opt_res_hist,     true};
data = {  
        'm_bt_hist',        m_bt_hist,        true;
        'm_dw_hist',        m_dw_hist,        true;    
        'm_f_hist',         m_f_hist,         true;
        'm_scan_hist',      m_scan_hist,      true;
        'm_scan_t_hist',    m_scan_t_hist,    true;
        'UAV_loc_hist',     UAV_loc_hist,     true;
        's_obj_hist',       s_obj_hist,       true};
exportData(data, data_exp_saveDir, data_exp_folder);

%% Errata
% Travel time calculation
%                             % Calculate time to complete tasks and reach
%                             % cell
%                             % Groundspeed calculation using http://hyperphysics.phy-astr.gsu.edu/hbase/lcos.html#c3
%                             % Other source: https://drstevenhale.gitlab.io/main/2008/11/triangleofvelocities/
%                             % Notes
%                             % - change notation for velocities and angles
%                             % - if t normalised, then calculate att after
%                             % all t calculated - no possible value for max
%                             % t as well? unless considering absolute worst
%                             % case scenario - travel to opposite side of
%                             % map, scan then return back to opposite side.
%                             theta       = ang_w - ang_g;
%                             beta        = asin(v_w*sin(theta)/v_as);
%                             v_gs        = sqrt(v_as.^2 + v_w.^2 - 2*v_as*v_w*cos(pi-theta-beta));
%                             t           = UAV_travelTime(UAV) + UAV_scanTime(UAV) + dist/v_gs;
%                             prior       = m_prior(i, j);
%                             att         = evalfis(fis, [t, prior]);