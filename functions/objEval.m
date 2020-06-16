%% Function objEval
% Evaluate objective function

%% Notes
% Both outputs required?
% Include consideration of other fire states in priority map calculation.

function [s_obj, obj] = objEval(m_f, m_bo, ...
                                r_bo, r_f, ...
                                dt_s, s_obj)

    % Generate active fire map
    m_fo            = m_f;
    m_fo(m_fo==1)   = 0;
    m_fo(m_fo==2)   = 0;
    m_fo(m_fo==3)   = 1;
    m_fo(m_fo==4)   = 0;
    
    % Calculate priority map
    m_P     = r_bo.*m_bo + r_f.*m_fo;   % Priority map
    obj     = sum(sum(m_P))*dt_s;       % Objective function for current timestep
    s_obj   = s_obj + obj;              % Sum of objective over time

end