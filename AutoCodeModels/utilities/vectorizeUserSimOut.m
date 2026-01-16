function [out, totalRows] = vectorizeUserSimOut(u, numEl)
%#codegen

totalRows = 0;
y=zeros(numEl,1); % preallocate storage
out=zeros(numEl,1); % preallocate storage

fn = fieldnames(u);

for z = 1:numel(fn)

  % get the current field
  x = u.(fn{z});
  
  if isstruct(x)
    [n,~] = size(x);
    for i = 1:n
      [xvec, numRows]= vectorizeUserSimOut(x(i), numEl);
      y(totalRows+1:totalRows+numRows,1) = xvec;
      totalRows = totalRows + numRows;
    end
  else
    [n,m] = size(x);
    numRows = n * m;
    xvec = reshape(x, [numRows,1]); % reshape any matrices into a column vector
    y(totalRows+1:totalRows+numRows,1) = xvec;
    totalRows = totalRows + numRows;
  end % end if isstruct(x)

end % end for z = 

out = y(1:totalRows,1);

end