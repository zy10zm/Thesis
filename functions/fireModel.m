%% Function fireModel.m
% Description:  Fire state model for the disaster area. Based on article by
% Ohgai, Gohnai, and Watanabe: "Cellular automata modeling of fire spread in
% built-up areas�A tool to aid community-based
% planning for disaster mitigation"

% Inputs:   m_f - fire map - tracks states of fire in each cell. 
%               cell states:
%               0 - unburnable
%               1 - not burning yet - able to burn
%               2 - catching fire   - not able to spread
%               3 - burning         - able to spread
%               4 - extinguished
%           w_s - wind speed
%           w_d - wind direction - 0, pi/2, pi, 3*pi/2
%           m_s - map of flammability of map area.
%               cell states:
%               1 - wooden
%               0.6 - fire prevention wooden
%               0 - fireproof
%           m_p - ratio of cell area occupied by building
%           gridSize - length of grid cells (m)
%               
% Author:   Craig Maxwell
% Date:     28/02/2020

%% Notes
% m_p - set using indoor temperature standard curve of wooden house on fire
% t_i and t_b are estimated for a grid size of 9m^2 - so set fire map cell
% size to 3m.

%% Model of fire spread using cellular automa
function [m_f, m_bt, W_ws] = fireModel(m_f, m_s, m_p, m_bt, del_t, k, n_x, n_y, w_s, w_d, a, b)
    
    %% Initialise variables
%     a   = 1;                % Coefficient to tune degree of slowdown in fire spread
%     b   = 1;                % Coefficient to tune range and direction of spread
    t_i = 120;              % Ignition time (s)
    t_b = 600;              % Burnout time (s)
    F   = zeros(n_x,n_y);   % Fire spread probability map
    fire_count = 0;
    
    % Downwind map
    W_dir_ws    = zeros(n_x, n_y);
    W_dis_ws    = zeros(n_x, n_y);
    W_ws        = zeros(n_x, n_y);
    W_ws_temp   = zeros(n_x, n_y);

    % Seed
    rng(k);                 % Seed random number generator using timestep
    
    %% Calculate W
    
    % Wind fire spread probability model
    % Source: https://www.nat-hazards-earth-syst-sci.net/19/169/2019/#bib1.bibx7
    % Problem with this model is that it does not allow for fire spread
    % over multiple cells - could multiply with distance modifier?
    % Make sure to align angles in one direction.
    % Need to decide on frame of reference for coordinates.
    % Rename variables n_W and n_wm?
    % - need to make sure spread probabilities are well tuned
    % - Think about how to use this model to create spread time estimation.
    % - Selection of W_rad based off windspeed, as done in other papers
    % - Limit maximum value to 1? And how to ensure that while everything
    % else stays consistent? - tuning of the constants or by fitting with
    % some function? - has to be below 1 - is this well balanced for
    % windspeed - does changing make it too likely to spread compared to
    % low windspeed - in the end, well-tuned model will allow control
    % system to perform as intented.
    
    W_rad   = 4;
    W_n     = W_rad*2+1;
    W_dir   = zeros(W_n, W_n);
    W_dis   = zeros(W_n, W_n);
    c_wm_1  = 0.5;   % How to select?
    c_wm_2  = 0.5;   % How to select?
    c_wm_d  = 0.9;   % Distance constant - Range [0 1]
    
    for i = 1:W_n
        for j = 1:W_n
            % Direction modifier
            f_d         = atan2((i-(W_rad+1)),(j-(W_rad+1))); % Fire direction [-pi, pi]
            ang         = w_d-f_d;
            W_dir(i,j)  = exp(w_s*(c_wm_1 + c_wm_2*(cos(ang)-1)));
            
            % Distance modifier - Percent drop per cell distance
            W_dis(i,j)= c_wm_d^(sqrt((i-(W_rad+1)).^2 + (j-(W_rad+1)).^2));
            W = W_dir.*W_dis;
        end
    end
    
    for i = 1:n_x
        for j = 1:n_y
            %% Advance fireMap
            % If active fire state
            if m_f(i,j) == 2
                m_bt(i,j) = m_bt(i,j) + del_t;
                % Advance ignition to combustion
                if m_bt(i,j) >= t_i
                    m_f(i,j) = 3;
                end
            elseif m_f(i,j) == 3
                m_bt(i,j) = m_bt(i,j) + del_t;
                % Advance combustion to burnout
                if m_bt(i,j) >= t_b
                    m_f(i,j) = 4;
                    m_bt(i,j) = 0;
                end
            end
            
            % Calculate fire spread probabilities
            if m_f(i,j) == 3
                t_ckl = m_bt(i,j);
                if t_ckl <= (t_b-t_i)/5+t_i
                    p = 4/(t_b-t_i)*t_ckl+(0.2*t_b-4.2*t_i)/(t_b-t_i);
                elseif t_ckl <= t_b
                    p = 5/(4*(t_b-t_i))*(-t_ckl+t_b);
                end
                % Add fire spread probability in neighborhood
                for ii = 1:size(W,1)
                    for jj = 1:size(W,1)
                        iii = i+ii-W_rad;
                        jjj = j+jj-W_rad;
                        if iii > 0 && iii <= n_x && jjj > 0 && jjj <= n_y
                            F(iii,jjj) = a*(m_s(iii,jjj)* m_p(iii,jjj))*W(ii,jj).^b*p; % Change ws to wm
                        end
                    end
                end
            end                        
        end
    end
    
    % Determine if fire spread occurs    
    for i=1:n_x
        for j=1:n_y
            if m_f(i,j) == 1 && rand <= F(i,j)
                % Ignition occurs
                m_f(i,j) = 2;
            end
        end
    end
    
    % Calculate downwind map
    % Likely time for fire to spread to cell + time to become active - take
    % lowest value and calculate for all active fires in grid. May be some
    % problems with this calculation.
    for i = 1:n_x
        for j = 1:n_y
            % For each active fire spot calculate likely time to spread to
            % rest of map - only interested in cells immediately around W.
            % Create temp map for each active fire spot. Because chance of
            % spread is assumed linear should be good approximation. Also
            % note - if linear, should be much easier way to do this. Any
            % matrix method to do entire calculation at once? Other options
            % - interpolation? - test several methods for this - first one
            % just using extended fire spread model for entire map
            % Treat m_f == 2 and m_f == 3 differently?
            % Should also be related to a and b coefficients?
            
            if m_f(i,j) == 2 || m_f(i,j) == 3
                
                % Initialise temp layer and circle count
                fire_count = fire_count + 1;
                
                % Method 1 - expand W for each fire to match entire map
                % Notes:
                % - No need to save W_dir_ws and W_dis_ws data for all
                % active fires
                % - Also consider fireMap states 2 and 4?
                
                for m = 1:n_x
                    for n=1:n_y
                        
                        % Linear direction modifier
                        f_d             = atan2((m-i),(n-j));
                        ang             = w_d-f_d;
                        W_dir_ws(m,n)   = exp(w_s*(c_wm_1 + c_wm_2*(cos(ang)-1))); 
                        
                        % Linear distance modifier - linear decrease down
                        % to maximum possible separation.
                        W_dis_ws(m,n)   = 1 - sqrt((m-i).^2 + (n-j).^2)/sqrt(n_x.^2 + n_y.^2);
                    end
                end
              
                % Normalise W_dir_ws - maybe too high for cell with fire?
                W_dir_ws = mat2gray(W_dir_ws);
                % Wind map
                W_ws_temp(:,:,fire_count)   = W_dir_ws(:,:).*W_dis_ws(:,:);
                % Set fire spot to maximum value
                W_ws_temp(i,j,fire_count)   = 1;
            end
        end
    end
    
    % Take max values 
    for i = 1:n_x
        for j = 1:n_y
            W_ws(i,j) = max(W_ws_temp(i,j,:));
        end
    end
end

%% Model notes
% Fire in cell burns for fixed time t_burn
% Fire has probability of spreading according to windMap and state of fire.
% Random number generator initialised with string and timestep - therefore
% changes with every timestep
% Structure states - open space, wooden, fire-prevention wooden, fireproof
% How to calculate downwind map parameter? - must account for wind speed
% and direction.

% From another paper - wind model effect - if we have a good model for
% generating wind maps then we may be able to calculate spread time better.
% Ang - angle between wind direction and the fire propagation

%% Research
% Cellular automa model - https://www.researchgate.net/publication/256662471_A_Cellular_Automata_Model_for_Fire_Spreading_Prediction
% Physics-based fire spread model - http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.157.3638&rep=rep1&type=pdf

%% Errata
% Programmatically creating W
%         n_ws = 2;
%         W = zeros(2*n_ws+1);
%         for i = -n_ws:n_ws
%             for j = -n_ws:n_ws
%                 if abs(i) == 2 || abs(j) == 2 && abs(i) ~= abs(j)
%                     W(i,j) = 0.3;
%                 elseif abs(i) == 1 || abs(j) == 1
%                     W(i,j) = 0.5;
%                 end
%             end
%         end

% W matrices from paper
%     % Wind speed effect
%     if w_s < 1
%         W = [   0.0, 0.3, 0.3, 0.3, 0.0;
%                 0.3, 0.5, 0.5, 0.5, 0.3;
%                 0.3, 0.5, 1.0, 0.5, 0.3;
%                 0.3, 0.5, 0.5, 0.5, 0.3;
%                 0.0, 0.3, 0.3, 0.3, 0.0];
%     elseif w_s < 5
%         W = [   0.0, 0.0, 0.3, 0.3, 0.0;
%                 0.0, 0.2, 0.6, 1.0, 0.5;
%                 0.0, 0.2, 1.0, 1.0, 0.5;
%                 0.0, 0.2, 0.6, 1.0, 0.5;
%                 0.0, 0.0, 0.3, 0.3, 0.0];
%     elseif w_s < 8
%         W = [   0.0, 0.0, 0.0, 0.1, 0.3, 0.0, 0.0;
%                 0.0, 0.0, 0.1, 0.2, 0.6, 0.4, 0.0;
%                 0.0, 0.1, 0.2, 0.5, 1.0, 0.6, 0.3;
%                 0.0, 0.1, 0.2, 1.0, 1.0, 0.6, 0.3;
%                 0.0, 0.1, 0.2, 0.5, 1.0, 0.6, 0.3;
%                 0.0, 0.0, 0.1, 0.2, 0.6, 0.4, 0.0;
%                 0.0, 0.0, 0.0, 0.1, 0.3, 0.0, 0.0];
%     else
%         W = [   0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0;
%                 0.0, 0.0, 0.0, 0.1, 0.4, 0.5, 0.0;
%                 0.0, 0.0, 0.1, 0.2, 1.0, 0.6, 0.3;
%                 0.0, 0.0, 0.1, 1.0, 1.0, 0.6, 0.3;
%                 0.0, 0.0, 0.1, 0.2, 1.0, 0.6, 0.3;
%                 0.0, 0.0, 0.0, 0.1, 0.4, 0.5, 0.0;
%                 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0];
%     end
%     % Wind direction effect
%     if w_d == 0
%         W = rot90(W,1);
%     elseif w_d == pi/2
%         W = W;
%     elseif w_d == pi
%         W = rot90(W,3);
%     elseif w_d == 3*pi/2
%         W = rot90(W,2);
%     else
%         errorStatement("windDirection")
%     end

% Idea for fire spread range based on windspeed
%     % Neighbourhood model
%     % One used here seems to have a lot of neighbours
%     % https://www.nat-hazards-earth-syst-sci.net/19/169/2019/#bib1.bibx7
%     if w_s < 1
%         n_wm = 1;
%     elseif w_s < 4
%         n_wm = 2;
%     elseif w_s < 8
%         n_wm = 3;
%     else
%         n_wm = 4;
%     end

%% Downwind map idea
                % Method 2 - idea - more elaborate
%                 circle_bt = 1; % Current prediction time radius
%                 % Time to become active in immediate fire neighborhood
%                 circle_ts = W(n_wm-1:n_wm+1,n_wm-1:n_wm+1)./del_t + t_i*ones(3,3);
%                 % Termination condition of radius
%                 while i - circle_bt > 0 || i + circle_bt < n_x || j - circle_bt > 0 || j + circle_bt < n_y
%                     % Time calculation
%                     % Draw rectangle with radius circle_bt
%                     for ii = -circle_bt:circle_bt
%                         for jj = -circle_bt:circle_bt
%                             iii = i + ii;
%                             jjj = j + jj;
%                             % Check within map
%                             if iii > 0 && iii < n_x && jjj > 0 && jjj < n_y
%                                 % Extend
%                                 if ii < 0 && jj < 0     % LD
%                                     m_dw_temp(iii,jjj,fire_count) = m_dw_temp(iii-1,jjj-1,fire_count) + ; % This is wrong method
%                                 elseif ii < 0 && jj > 0 % LU
%                                     m_dw_temp(iii,jjj,fire_count) = ;                                    
%                                 elseif ii > 0 && jj < 0 % RD
%                                     m_dw_temp(iii,jjj,fire_count) = ;                                    
%                                 elseif ii > 0 && jj > 0 % RU
%                                     m_dw_temp(iii,jjj,fire_count) = ;                                    
%                                 end
%                             end
%                         end
%                     end
%                 end
