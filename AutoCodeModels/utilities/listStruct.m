function fpn = listStruct(n,S)
% listStruct List full path of all fields in a structure array 
%   fpn = listStruct(n,S) Returns a cell array of the full path for a
%   structure.  The input n is the parent structure name and S is the
%   structure.  Works with arrays of structures.

% eheim 20151211
% % $Id: listStruct.m 515 2018-05-08 20:02:40Z decox $

% Initialize full path name cell array.
fpn = {};
if ~isstruct(S)
    error('Input is not a structure.')
else
    % Supporting multidimensional arrays is possible but requires
    % unnecessary formating. So we will work with the element number
    % instead of indicies.    
    ndim = ndims(S);% Number of dimensions of structure
    sizes = zeros(ndim,1);
    for dim = 1:ndim
        sizes(dim) = size(S,dim);
    end
    nel = prod(sizes);% Total number of elements.
    % Cycle through structure array.
    for j = 1:nel
        % Select one element of structure array.
        s = S(j);
        % Get field names.
        fn = fieldnames(s);
        % Cycle through field names.
        for i = 1:length(fn)
            if isstruct(s.(fn{i}))
                % Call this function again.
                temp = listStruct(fn{i},s.(fn{i}));
                % Prepend parent field name to cell array of child field paths
                if nel > 1
                    temp = strcat([n '(' int2str(j) ').'],temp);
                else
                    temp = strcat([n '.'],temp);
                end                   
            else
                % Prepend parent field name to cell array of child field paths
                if nel > 1
                    temp = {[n '(' int2str(j) ').' fn{i}]};
                else
                    temp = {[n '.' fn{i}]};
                end
            end
            % Append path name to list
            fpn = [fpn;temp];
        end
    end
end