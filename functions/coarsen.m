%% Coarsen raster grid
% m_occ - occupancy map (percent of each cell occupied)
% m_r   - raster map (binary, identifies if any occupancy in cell)

%% Assumptions
% - some error introduced by rounding - ignoring rows and column on sides
% of matrix.

%% To do
% - check distance estimations accurate
% - return coarsen factor
% - fix problem of scan map and environment map cell sizes not being an integer
% factor of each other
% - must be a better way of managing the environment map

%% Change log
% 22/06/2020 - improved readability
% 22/06/2020 - removed errata with idea of randomising building flammability

%% Coarsen raster
function [m_occ, m_r, l_c_x, l_c_y] = coarsen(m_p_in, ref, l_d)
    
    % Difference in lat negligible for small scale
    lat = ref.LatitudeLimits(1);
    m_per_deg_lat = 111132.954 - 559.822 * cos( 2 * lat ) + 1.175 * cos( 4 * lat);
    m_per_deg_lon = 111132.954 * cos ( lat );
    l_r_lat = ref.CellExtentInLatitude * m_per_deg_lat;
    l_r_lon = ref.CellExtentInLongitude * m_per_deg_lon;
    
    % Coarsen to desired cell size - side to right is removed if necessary
    n_lat   = round(l_d/l_r_lat);
    n_lon   = round(l_d/l_r_lon);
    % length of new cells in x direction
    l_c_x   = n_lat * m_per_deg_lat * ref.CellExtentInLatitude;  
    % length of new cells in y direction
    l_c_y   = n_lon * m_per_deg_lon * ref.CellExtentInLongitude;
    
    g_d     = [floor(size(m_p_in,1)/n_lat), floor(size(m_p_in,2)/n_lon)];
    m_occ   = zeros(g_d(1), g_d(2));
    m_r     = zeros(g_d(1), g_d(2));
       
    if n_lat < 2 || n_lon < 2
        error("Coarsen ratio not high enough")
    end

    for i = 1:g_d(1)
        for j = 1:g_d(2)
            ii = (i-1)*n_lat+1;
            jj = (j-1)*n_lon+1;        
            mat = m_p_in(ii:ii+n_lat-1,jj:jj+n_lon-1); % Extract data from original raster
            occ = nnz(mat)/(n_lat*n_lon);
            
            % Building occupancy ratio
            m_occ(i,j) = occ;
            
            % Building flammability
            if occ > 0 
                m_r(i,j) = 1;
            end
        end
    end
end
