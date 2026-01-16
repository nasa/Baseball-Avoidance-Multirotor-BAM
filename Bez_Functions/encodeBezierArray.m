function [wptVect, timeVect, parseVect] = encodeBezierArray(wptCellArray, timeCellArray)
%#codegen

  N_dim = numel(wptCellArray);
  N_wpt = zeros(1, N_dim);
  N_der = zeros(1, N_dim);
  for i = 1:N_dim
    [N_wpt(i), N_der(i)] = size(wptCellArray{i});
  end

  % column stacked ontop of column
%  wptVect = cell2mat(arrayfun(@(Q) wptCellArray{Q}(:),1:N_dim,'UniformOutput',false)');
% codegen doesn't support arrayfun with 'UniformOutput' arg set to false
   wptVect = zeros(sum(N_wpt.*N_der), 1);
   startIdx = 1;
   for i = 1:N_dim
     wptVect(startIdx:(startIdx + N_wpt(i)*N_der(i) - 1) ) = wptCellArray{i}(:);
     startIdx = startIdx + N_wpt(i)*N_der(i);
   end

%  timeVect = cell2mat(arrayfun(@(Q) timeCellArray{Q}(:),1:N_dim,'UniformOutput',false)');
% codegen doesn't support arrayfun with 'UniformOutput' arg set to false
   timeVect = zeros(sum(N_wpt), 1);
   startIdx = 1;
   for i = 1:N_dim
     timeVect(startIdx:(startIdx + N_wpt(i) - 1) ) = timeCellArray{i}(:);
     startIdx = startIdx + N_wpt(i);
   end

  % vector specifying number of dimensions, waypoints, and derivatives
  % used to decode the encoded waypoint array
  parseVect = [N_dim, N_wpt, N_der];
  
end

