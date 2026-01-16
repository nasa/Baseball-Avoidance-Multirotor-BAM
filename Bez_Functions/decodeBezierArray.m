function [wptCellArray, timeCellArray] = decodeBezierArray(wptVect, timeVect, parseVect)
%#codegen

% unpack the vector back into a cell array
N_dim  = parseVect(1);
N_wpts = parseVect(2:(2 + N_dim - 1));
N_der  = parseVect((2 + N_dim):end);

wptCellArray = cell(1, N_dim);
start_idx = 1;
for n_dim = 1:N_dim
  % the number of vector elements per dimension
  n_ele_to_get = N_wpts(n_dim) * N_der(n_dim);

  % wptVect elements for this dimension
  X = wptVect(start_idx:(start_idx + n_ele_to_get - 1), 1);

  % reshape into a cell, nwpts x nder per dimension
  wptCellArray{n_dim} = reshape(X, [N_wpts(n_dim) N_der(n_dim)]);

  start_idx = start_idx + n_ele_to_get;
end

timeCellArray = cell(1, N_dim);
start_idx = 1;
for n_dim = 1:N_dim
  % the number of vector elements per dimension
  n_ele_to_get = N_wpts(n_dim);

  % timeVect elements for this dimension
  X = timeVect(start_idx:(start_idx + n_ele_to_get - 1), 1);

  % reshape into a cell, 1 x nwpts per dimension
  timeCellArray{n_dim} = reshape(X, [1 N_wpts(n_dim)]);

  start_idx = start_idx + n_ele_to_get;
end

end

