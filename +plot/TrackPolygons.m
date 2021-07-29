classdef TrackPolygons < plot.Base
    % plot discretized track's polygon creation process
    %
    % usage example
    % ```
    % [checkpoints, track_creation_scale] = model.track.HockenheimShort()
    % plot.TrackPolygons().plot(checkpoints, track_creation_scale, 0.5)
    % ```
    
    methods
        function plot(obj, checkpoints, track_creation_scale, epsilon_area_tolerance)
            % epsilon_area_tolerance [m^2]: maximal poylgon differnce for
            % merging
            set(groot, 'CurrentFigure', obj.figure_handle); % same as 'figure(f)' but without focusing
            set(obj.figure_handle, 'Name', sprintf('Track - SCR (scale %s)', utils.rat2str(track_creation_scale)), 'WindowState', 'maximized');
            
            t = tiledlayout(2,2);
            t.Padding = 'tight';
            t.TileSpacing = 'tight';
            
            % 0) original
            ax1 = nexttile;
            title(sprintf('0) Original Track - Discretized with %i Checkpoints', length(checkpoints)))
            plot.TrackCheckpoints(NaN).plot(checkpoints)

            % 1) tesselation (based on discret track checkpoints)
            ax2 = nexttile;
            track = controller.gen_track_SCR.t1_tesselate(checkpoints);
            plotTrackPolygons_(track, sprintf('1) Tesselation to %i Polygons', length(track.polygons)))
            % 2) merge polygons
            ax3 = nexttile;
            track = controller.gen_track_SCR.t2_merge(track, track_creation_scale, epsilon_area_tolerance);
            plotTrackPolygons_(track, sprintf('2) Merged to %i Polygons', length(track.polygons)))
            % 3) add overlaps
            ax4 = nexttile;
            track = controller.gen_track_SCR.t3_overlap(track, track_creation_scale);
            plotTrackPolygons_(track, sprintf('3) Overlapped %i Polygons', length(track.polygons)))
            
            linkaxes([ax1 ax2 ax3 ax4],'xy')
            xlabel(t, 'x [m]'); ylabel(t, 'y [m]')
                
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