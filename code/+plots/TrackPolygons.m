classdef TrackPolygons < plots.Base
    % plot discretized track's polygon creation process
    %
    % usage example
    % ```
    % [checkpoints, track_creation_scale] = model.track.HockenheimShort()
    % plots.TrackPolygons().plot(checkpoints, track_creation_scale, 0.5)
    % ```
    %
    % FIXME sync with controller.track_SCR's functions (instead of calling
    % the
    
    methods
        function plot(obj, checkpoints, track_tesselated, track_merged, track_overlapped, track_creation_scale)
            % epsilon_area_tolerance [m^2]: maximal poylgon differnce for
            % merging
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            set(obj.figure_handle, 'Name', sprintf('Track - SCR (scale %s)', utils.rat2str(track_creation_scale)), 'WindowState', 'maximized');
            
            if verLessThan('matlab', '9.7')
                warning('Unlinked figures for track generation plots due to old MATLAB version')
            else
                t = tiledlayout(2,2);
                if verLessThan('matlab', '9.10')
                    t.Padding = 'compact';
                    t.TileSpacing = 'compact';
                else
                    t.Padding = 'tight';
                    t.TileSpacing = 'tight';
                end
            end

            % 0) original
            if verLessThan('matlab', '9.7')
                figure
                xlabel('x [m]'); ylabel('y [m]')
            else
                ax1 = nexttile;
            end
            title(sprintf('0) Original Track - Discretized with %i Checkpoints', length(checkpoints)))
            plots.TrackCheckpoints(NaN).plot(checkpoints)

            % 1) tesselation (based on discret track checkpoints)
            if verLessThan('matlab', '9.7')
                figure
                xlabel('x [m]'); ylabel('y [m]')
            else
                ax2 = nexttile;
            end
            plotTrackPolygons_(track_tesselated, sprintf('1) Tesselation to %i Polygons', length(track_tesselated.polygons)))

            % 2) merge polygons
            if verLessThan('matlab', '9.7')
                figure
                xlabel('x [m]'); ylabel('y [m]')
            else
                ax3 = nexttile;
            end
            plotTrackPolygons_(track_merged, sprintf('2) Merged to %i Polygons', length(track_merged.polygons)))

            % 3) add overlaps
            if verLessThan('matlab', '9.7')
                figure
                xlabel('x [m]'); ylabel('y [m]')
            else
                ax4 = nexttile;
            end
            plotTrackPolygons_(track_overlapped, sprintf('3) Overlapped %i Polygons', length(track_overlapped.polygons)))

            if ~verLessThan('matlab', '9.7')
                linkaxes([ax1 ax2 ax3 ax4],'xy')
                xlabel(t, 'x [m]'); ylabel(t, 'y [m]')
            end
                
            function plotTrackPolygons_(track, name)
                title(name)
                hold on

                % plot every polygon hull
                for i = 1:length(track.polygons)
                    vertices = track.vertices(:, track.polygons(i).vertex_indices);
                    [K, ~] = convhull(vertices');
                    vertices = vertices(:, K(:,1));
                    plot(vertices(1,:), vertices(2,:), '-');
                end
                
                axis equal
            end
        end
    end
end