function set_figure_properties(figHandle, preset, paperheight_in)
% SET_FIGURE_PROPERTIES     Set Properties used for figures based on the type of export.

    switch lower(preset)
    case 'paper' % \the\linewidth=252.0pt, 1pt=0.3515mm --> 88.578mm
        fontsize    = 6;
        paperwidth  = 8;    % picture width in cm
        paperheight = 4;    % picture height in cm
%         paperwidth  = 8.8;    % picture width in cm
%         paperheight = 4.4;    % picture height in cm
        linewidth=0.5;
        fontname    = 'CMU Serif';
        units       = 'centimeters';
    
    case 'presentation'
        fontsize    = 18;
        paperwidth  = 31.77; % picture width in cm
        paperheight = 14.01; % picture height in cm
        linewidth=1;
        fontname    = 'Arial';
        units       = 'centimeters';
    
    case 'document'
        fontsize    = 9;
        paperwidth  = 15.7; % picture width in cm
        paperheight = 7.85; % picture height in cm
        linewidth=0.5;
        fontname    = 'CMU Serif';
        units       = 'centimeters';

    case 'video'
        fontsize    = 20;
        paperwidth  = 1920;
        paperheight = 1080;
        linewidth   = 1;
        fontname    = 'Arial';
        units       = 'pixels';
    
    otherwise % default
        error('No valid preset selected.')
    end
    if nargin == 3
        paperheight = paperheight_in;
    end
    
    % beauty corrections
    allchildren = get(figHandle, 'Children'); % get handle figure
    for a=1:length(allchildren)
        try % set title handle
            h_title=get(allchildren(a),'Title');
            set(h_title,...
                'FontWeight','normal',...
                'FontSize',fontsize+1,...
                'FontName',fontname,...
                'Interpreter','latex');
        catch
            % continue
        end
        try % redefine x- and y-labels
            h_xlabel = get(allchildren(a), 'xlabel');
            h_ylabel = get(allchildren(a), 'ylabel');
            set(h_xlabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
            set(h_ylabel,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'Interpreter','latex')
        catch
            % continue
        end
        % set axes
        try
            h_axes=get(allchildren(a),'Axes');
            set(h_axes,...
                'FontSize',fontsize,...
                'FontName',fontname,...
                'LineWidth',linewidth, ...
                'Box','on');
        catch
            % continue
        end
        % set subplotaxes
        try
            set(allchildren(a)...
                ,'FontSize',fontsize...
                ,'FontName',fontname...
                ,'LineWidth',linewidth...
                ,'Box','on'...
            );
                % ,'XAxisLocation','origin'...
                % ,'YAxisLocation','origin'...
        catch
            % continue
        end
        % set legend
        if strcmpi(get(allchildren(a),'Tag'),'legend')
            h_legend=allchildren(a);
            if isequal(get(h_legend,'Interpreter'),'none')
                set(h_legend,'FontSize',fontsize+1)
            else
                set(h_legend,'FontSize',fontsize)
            end
            set(h_legend,...
                'LineWidth',linewidth,...
                'FontName',fontname,...
                'Interpreter','latex',...
                'Box','on');
        end
        % Set graphic objects
        
        h_graphics = get(allchildren(a),'Children');
        for h_graphic = h_graphics'
            try
                set(h_graphic ...
                    ,'LineWidth', linewidth ...
                );
            catch
                % continue
            end
        end
    end
    
    % background color
    set(figHandle, 'Color', 'w');
    % format
    set(figHandle,'Units',units);
    screenpos = get(figHandle,'Position');
    set(figHandle ...
        ,'Position',[screenpos(1:2), paperwidth, paperheight]...  % px, py, w, h, of figure on screen
    );
%         
%     if ~strcmp(preset, 'video')
%         % Make axes span whole window
%         ax = get(figHandle,'CurrentAxes');
%         outerpos = ax.OuterPosition;
%         ti = ax.TightInset; 
%         left = outerpos(1) + ti(1);
%         bottom = outerpos(2) + ti(2);
%         ax_width = outerpos(3) - ti(1) - ti(3) - 2e-3; %box was sometimes cut off
%         ax_height = outerpos(4) - ti(2) - ti(4);
%         ax.Position = [left bottom ax_width ax_height];
%         set(figHandle,'PaperUnits',units)
%         set(figHandle ...
%             ,'PaperSize',[paperwidth, paperheight] ...
%             ,'PaperPosition',[0, 0, paperwidth, paperheight] ...
%         );
%     end
    
end