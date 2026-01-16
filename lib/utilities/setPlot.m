function setPlot(fontSize,lineWidth,gridType, plotspec, printspec)
%function setPlot(fontSize,lineWidth,gridType, plotspec)
%
%  Sets common properties for plots
%    fontsize and linewidth apply to all lines/fonts in a figure
%    grid line width and marker size scale relaive to line width
%    gridType=0,1,2 for one, major, major+minor grids
%    plotspec defaults to 'gcf', for all figures use 'all' instead
%
%  Sensible defaults exist, try setPlot() on existing figure
%

% Defaults
if ~exist('fontSize','var') || isempty(fontSize), fontSize=16; end
if ~exist('lineWidth','var') || isempty(lineWidth), lineWidth=1.5; end
if ~exist('gridType','var') || isempty(gridType), gridType=2; end
if ~exist('plotspec','var') || isempty(plotspec), plotspec='gcf'; end
if ~exist('printspec','var') || isempty(printspec), printspec=false; end

if plotspec =='all'
    figHandles = findobj('Type', 'figure');
else 
    figHandles = get(groot, 'CurrentFigure');
end

for ploop = 1:size(figHandles,1)
    cur_fig = figHandles(ploop);
    % Set all fonts and line sizes
    set(findobj(cur_fig,'-property','FontSize'),'FontSize',fontSize);
    set(findobj(cur_fig,'-property','LineWidth'),'LineWidth',lineWidth);

    % Set marker size,based on line size
    set(findobj(cur_fig,'-property','MarkerSize'),'MarkerSize',min(max(6,lineWidth*3.5),12));
    alist=findobj(cur_fig,'-property','XGrid');

    % Change to filled markers, unless already specified
    mlist=findobj(cur_fig,'-property','MarkerFaceColor','-property','Color');
    for i=1:length(mlist), 
     if strcmp(get(mlist(i),'MarkerFaceColor'),'none'),
       set(mlist(i),'MarkerFaceColor',get(mlist(i),'Color')); 
     end
    end


    % Set grid, and reduce grid to line size to be less than plot line size

    % Grid
    switch(gridType),
      case 0
       for i=1:length(alist),
         set(alist(i),'XGrid','Off','YGrid','Off');
         set(alist(i),'XMinorGrid','Off','YMinorGrid','Off');
         set(alist(i),'LineWidth',min(max(lineWidth/2.5,0.5),2));     
       end
      case 1
       for i=1:length(alist),
         set(alist(i),'XGrid','On','YGrid','On');
         set(alist(i),'XMinorGrid','Off','YMinorGrid','Off');
         set(alist(i),'LineWidth',min(max(lineWidth/2.5,0.5),2));     
       end
      case 2
       for i=1:length(alist),
         set(alist(i),'XGrid','On','YGrid','On');
         set(alist(i),'XMinorGrid','On','YMinorGrid','On');
         set(alist(i),'LineWidth',min(max(lineWidth/2.5,0.5),2));
       end
    end

    % Properties
    % set custom print settings if request
    % dev note - I'm doing this to disable these settings when I set the
    % specs because it seems to not go well with the plots I'm generating.
    % It shrinks everything.          -DRH 20210927
    if printspec
      %Set paperpositionmode to auto, which allows the -r0 print option to match screen resolutions
      % This is the default setting in R2016a and beyond
      % Setting here ensures this more sensible default for prior versions
      set(cur_fig,'PaperPositionMode','auto')

      % Set the paper size to be tight around the figure axes
      % This causes pdf prints to have a tight bounding box, better for latex integration
      fig_pos = get(cur_fig,'PaperPosition');
      set(cur_fig,'PaperSize',[fig_pos(3) fig_pos(4)]);
    end
end % for ploop = 1:...