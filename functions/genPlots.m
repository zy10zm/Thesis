%% Function genPlots 

%% To do
% Plots to add
% - save figures using titles of plots?
% - input and output membership functions
% - history plots as GIF - https://stackoverflow.com/questions/42132955/matlab-how-to-save-animated-plot-to-a-gif
% - single timestamp map plots
% Add file and directory management
% - test save function
% - figure out which colormaps are required
% - individually test all plots
% - add correct time to plot titles

%% Bugs

%% Script
function [] = genPlots( dataSet, saveDir, folder, ...
                        ax_lon, ax_lat, ax_lat_scan, ax_lon_scan, ...
                        m_f_colourMap)
                      
	% Close any open figures
  close all

  % Save working directory path
  workingDir = pwd;
  % Change to save directory
  cd(saveDir); 
  % Create save directory
  dateTime = datestr(now,'yyyy-mm-dd-HH-MM');
  if folder ~= ""
    folder = strcat(dateTime, '-', folder);
  else
    folder = dateTime;
  end
  mkdir(folder);
  cd(folder);

  for i = 1:length(dataSet)
    dataName  = dataSet{i,1};
    data      = dataSet{i,2};
    plotData  = dataSet{i,3};

    if plotData == true
      if dataName == "m_f_hist"
        h = figure;
        filename = 'testAnimated.gif';
        for n = 1:size(data, 3)
          legend("Fire state");
          xlabel("Latitude");
          ylabel("Longitude");
          imagesc(ax_lon, ax_lat, data(:,:,n));
          axis tight manual
          daspect([1 1 1]);
          caxis([0 4]);
          colormap(m_f_colourMap);
          title(strcat("Fire map history t = ", int2str(n)));
          drawnow
          % Capture plot as image
          frame = getframe(h);
          im = frame2im(frame); 
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end
        end
        close(h); % Prevent saving
      elseif dataName == "m_bt_hist"
        
      elseif dataName == "m_dw_hist"
        
      elseif dataName == "m_scan_hist"
        
      elseif dataName == "m_f"        
        
      elseif dataName == "m_bt"
        
      elseif dataName == "m_dw"
        
      elseif dataName == "m_scan"
        
      elseif dataName == "m_bo"
        
      elseif dataName == "sum_obj_hist"
        
      elseif dataName == "UAV_loc_hist"
        
      elseif dataName == "fis_param_hist"
        
      elseif dataName == "fis"
        for iFis=1:length(data)
          fis = data(iFis);
          % Inputs
          h = figure;
          sgtitle(sprintf("FIS Inputs for Agent %d", iFis));
          for iFisIn = 1:length(fis.Inputs)
            subplot(length(fis.Inputs),1,iFisIn);
            plotmf(fis,'input',iFisIn);
            title(fis.Inputs(iFisIn).name);
          end
          % Plot outputs
          h = figure;
          sgtitle(sprintf("FIS Outputs for Agent %d", iFis));
          gensurf(fis);
        end
      end
    end
  end

%   %% Save figures
%   FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
%   for iFig = 1:length(FigList)
%     FigHandle = FigList(iFig);
%     FigName   = get(FigHandle, 'Name');
%     savefig(FigHandle, fullfile(saveDir, FigName, '.fig'));
%   end

  %% Go back to working directory
  cd(workingDir);

end

%% Errata
%       elseif dataName == "sum_obj_hist" % Scatter of cumulative objective function
%         sum_obj_hist_t = ;
%         scatter(sum_obj_hist,sum_obj_hist_t);
%         legend();
%         title("Accumulated priority over time");
%       elseif dataName == "UAV_loc_hist" % Arrow plot % Also want the times at which they visit?
%         UAV_loc_hist_t = ;
%         legend();
%         title("UAV paths");
%       elseif dataName == "fis_param_hist" % How to plot this?
%         % Notes
%         % - separate subplot for each agent
%         % - probably better to use a table for this
%         fis_param_hist_t = ;
%         legend();
%         title("FIS parameters over time");
%     elseif dataName == "m_bo"  % Test!
%       imagesc(ax_lon, ax_lat, data);
%       c = colorbar;
%       c.Label.String = 'Building state';
%       caxis([0 4])
%       axis ij
%       daspect([1 1 1])
%       title("Building occupancy map");  % Test!
%     elseif dataName == "m_f"
%       imagesc(ax_lon, ax_lat, data);
%       colormap(m_f_colourMap)
%       c = colorbar;
%       c.Label.String = 'Fire State';
%       caxis([0 4])
%       axis ij
%       daspect([1 1 1])
%       title("Fire map");
%     elseif dataName == "m_scan"  % Test!
%       imagesc(ax_lon_scan, ax_lat_scan, data);
%       c = colorbar;
%       c.Label.String = 'Scan State';
%       caxis([0 4])
%       axis ij
%       daspect([1 1 1])
%       title("Scan map");
%     elseif dataName == "m_bt" % Test!
%       imagesc(ax_lon, ax_lat, data);
%       c = colorbar;
%       c.Label.String = 'Burntime State';
%       caxis([0 4])
%       axis ij
%       daspect([1 1 1])
%       title("Burntime map");
 
%% Errata - old plots for reference
%% Old plots
% % Fire map
% figure;
% for i = 1:size(m_f_hist,3)
%     subplot(2, 2, 1);
%     imagesc(ax_lon, ax_lat, m_f_hist(:,:,i));
%     colormap(m_f_colourMap)
%     c = colorbar;
%     c.Label.String = 'Fire State';
%     caxis([0 4])
%     axis ij
%     daspect([1 1 1])
%     
%     subplot(2, 2, 2);
%     imagesc(ax_lon, ax_lat, m_dw_hist(:,:,i));
% %     c = colorbar;
%     c.Label.String = 'Downwind Time';
%     axis ij
%     daspect([1 1 1])
%     
%     subplot(2, 2, 3);
%     imagesc(ax_lon, ax_lat, m_bt_hist(:,:,i));
% %     c = colorbar;
%     c.Label.String = 'Downwind Time';
%     axis ij
%     daspect([1 1 1])
%     
% %     subplot(2, 2, 4);
% %     imagesc(ax_lon, ax_lat, m_scan_hist(:,:,i));
% %     c.Label.String = 'Scanned Cells';
% %     axis ij
% %     daspect([1 1 1])
%     pause(0.5)
% end

% figure;
% for i = 1:size(m_dw_hist,3)
%     imagesc(ax_lon, ax_lat, m_dw_hist(:,:,i));
%     c = colorbar;
%     c.Label.String = 'Downwind Time';
%     axis ij
%     daspect([1 1 1])
%     pause(0.5)
% end

% figure;
% imagesc(ax_lon, ax_lat, m_f_hist(:,:,1));
% colormap(m_f_colourMap)
% caxis([0 4])
% c = colorbar;
% c.Label.String = 'Fire State';
% axis ij
% daspect([1 1 1])
% 
% figure;
% imagesc(ax_lon, ax_lat, m_bt_hist(:,:,10));
% c = colorbar;
% c.Label.String = 'Downwind Time';
% axis ij
% daspect([1 1 1])

% figure;
% h = imagesc(ax_lon, ax_lat, m_search);
% title("Search map")
% % set(h, 'EdgeColor', 'none');
% c = colorbar;
% c.Label.String = 'Building occupancy';
% caxis([0 1])
% axis ij
% daspect([1 1 1])
% 
% % Downwind map
% figure;
% h = imagesc(ax_lon, ax_lat, m_search);
% title("Search map")
% % set(h, 'EdgeColor', 'none');
% c = colorbar;
% c.Label.String = 'Building occupancy';
% caxis([0 1])
% axis ij
% daspect([1 1 1])
% colormap(m_f_colourMap)

%% Animate plot
%     % Update plot
%     h = pcolor(m_f);
%     set(h, 'EdgeColor', 'none');
%     colormap(m_f_colourMap)
%     caxis([0 5])
%     axis ij
%     daspect([1 1 1])
% %     pause(0.1)
