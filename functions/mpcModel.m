%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

%% Notes
% Should prediction horizon be equal to the time between MPC optimsiations?
% Can hist varibales be removed?

% To do
% - initialise mpc simulation
% - seed all use of random number with timestep so it is identical to real
% simulation. - Makes assumption that future environment states can be
% perfectly predicted - future probability based prediction would be required.
% - check no duplicates in function inputs
% Remove unneccessary inputs
% - simulation over prediction horizon will need to be moved to separate
% function?

function [s_obj_pred] ...
          = mpcModel( fisArray, ...
          m_f, m_s, m_bo, m_bt, m_scan, m_t_scan, ...
          dt_a, dt_f, dt_p, dt_s,  ...
          n_UAV, n_p, n_x_search, n_y_search, n_x_f, n_y_f, ...
          UAV_occ, UAV_loc, UAV_target, UAV_task, t_travel_UAV, t_scan_UAV, ...
          k, negAtt, ...
          l_c_s_x, l_c_s_y, ...
          c_w1, c_w2, v_as_UAV, v_w, ang_w, ...
          r_bo, r_f)
        
  %% Initialise prediction variables
  % Counters
  t_pred = 0;
  ct_a_pred   = 0;
  ct_c_pred   = 0;
  ct_f_pred   = 0;
  s_obj_pred  = 0;
  ct_mpc_pred = 0;
  
  % Maps
  m_att   = zeros(n_x_search, n_y_search, n_UAV); % Attraction map
  m_dw    = zeros(n_x_f, n_y_f);                  % Downwind map
  m_prior = zeros(n_x_search, n_y_search);
  
  % FIS
  % 
  
  %% Simulation over prediction horizon
  while t_pred <= dt_p*n_p
    
    %% Update FIS parameters
    if ct_mpc_pred*dt_mpc <= t_pred
      fisArray(params) = fisParams; %% Example code - replace!
    end
    
    %% Environment model
    if ct_f_pred*dt_f <= t_pred
      % Update firemap and downwind map
      [m_f, m_bt, m_dw] = fireModel(  m_f, m_s, m_bo, m_bt, ...
                                      dt_f, k, n_x_f, n_y_f, ...
                                      v_w, ang_w, ...
                                      c_w1, c_w2);
      ct_f_pred = ct_f_pred + 1;
    end

    %% Fuzzy Controller
    if ct_c_pred*dt_c <= t_pred
      ct_c_pred = ct_c_pred + 1;
      [UAV_target] = fisModel(n_UAV, UAV_occ, UAV_loc, UAV_target, ...
                              n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
                              m_scan, m_att, m_dw, m_prior, ...
                              fisArray, ...
                              t_travel_UAV, t_scan_UAV, ...
                              ang_w, v_as_UAV, ...
                              negAtt);
    end

    %% Agent actions
    if ct_a_pred*dt_a <= t_pred
      ct_a_pred = ct_a_pred + 1;
      [   m_scan, m_scan_hist, m_scan_t_hist, ...
          UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
          t_travel_UAV, t_scan_UAV] ...
                      = uavModel( n_UAV, ...
                      m_t_scan, m_scan, m_scan_hist, m_scan_t_hist, ...
                      UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
                      t_travel_UAV, t_scan_UAV, ...
                      l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t);
    end

    %% Objective function evaluation
    [s_obj_pred, obj] = objEval(m_f, m_bo, ...
                           r_bo, r_f, ...
                           dt_s, s_obj_pred);


    %% Advance timestep
    t_pred = t_pred + dt_s;
    k      = k + 1;
  end
end