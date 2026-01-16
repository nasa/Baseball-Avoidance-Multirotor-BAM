function [b, numEl] = buildBusObject(varargin)
% function buildBusObject(s,prefix,name,props,options)
%% Builds bus objects from structure with prefix string.
%
% Usage: buildBusObject(s,prefix,name)
%        buildBusObject(s,prefix,name,props)
%        buildBusObject(s,prefix,name,props,options)
%

% eheim 20151009
% $Id: buildBusObject.m 2154 2017-03-27 16:25:22Z eheim $

% macheson, tbritton 20220926
%   modified to support an array of substructures
%
%   modified original function to return the top-level bus object as an
%   output instead of writing it to the workspace, due to other changes
%   related to structure field names with the same name.  As a result, the
%   prefix and name arguments aren't really needed anymore, unless the
%   options.name parameter is set to true

% s = SimIn.Modeling.model0
% name = MODEL
% prefix = 'BUS_Modeling_';

% TODO: improve error checking on inputs

s      = varargin{1};
prefix = varargin{2};
name   = varargin{3};

% Set default Bus Object Properties
props.Description = '';
props.DataScope   = 'Auto';
props.HeaderFile  = '';
props.Alignment   = -1;

options.names = false;

if nargin < 3
  error('To few input arguments.')
end
if nargin > 3 
  if ~isempty(varargin{4})
    props = varargin{4};
  end
end
if nargin > 4
  options = varargin{5};
end
if nargin > 5
  error('To many input arguments.')
end

numEl = 0;

fn = fieldnames(s);% Get field names of structure

e = repmat(Simulink.BusElement,numel(fn),1);% Preallocate elements
% Cycle through field names
for i = 1:numel(fn)
  x = s.(fn{i}); % Pull current field
  e(i).Name = fn{i}; % Set name of bus object element
  % Is this field another structure?
  if isstruct(x)
      % Call this function again
      if options.names
        % Expand path name with top level prefix
        str = strcat(prefix,name,'_');
      else
        % Use top level prefix
        str = prefix;
      end
      
      [~, dims, ~] = getValueAttr(x);
      % check for an array of structs
      if (dims(1) == 1)
        dim = dims(2);
      elseif (dims(2) == 1)
        dim = dims(1);
      else
        error('Unsupported dimensions, both dimensions greater than 1');
      end

      % default bus name/data type
      dataTypeName = strcat(str,fn{i});

      % check if bus by same name already exists
      existCheck = sprintf('exist(''%s'')', dataTypeName);
      busExists = evalin('base', existCheck);
      if busExists == true
        exist_bus = evalin('base', dataTypeName);
      end

      [k, numElIter] = buildBusObject(x,str,fn{i},props,options); % str and fn{i} options aren't really used anymore due to having it return the object and not just assign it to the workspace

      % bus name exists and not identical so create a new name
      if (busExists == true) && ~isequal(exist_bus, k)
        % buses have same default name but a different structure
        % Create a unique bus name...
        dataTypeName = sprintf('%s_1', dataTypeName);
        e(i).DataType = dataTypeName; % Rename the data type with prefix to match bus object name
      else
        e(i).DataType = dataTypeName; % Rename the data type with prefix to match bus object name
      end

      assignin('base', dataTypeName, k);  % create the bus in the workspace

      for j = 1:dim
        numEl = numEl + numElIter; % running total on number of data elements
      end

      e(i).Dimensions = dim; % Set dimensions
  else
    [dt, dims, compl] = getValueAttr(x);
    e(i).Complexity = compl;

    % check if either row or column vector, if so, treat as 1-D data
    if (dims(1) == 1)
      dim = dims(2);
    elseif (dims(2) == 1)
      dim = dims(1);
    else
      dim = dims;
    end
    e(i).Dimensions = dim; % Set dimensions

    e(i).DataType   = dt; % Set Data type

    numEl = numEl + dims(1)*dims(2);
  end
end
b = Simulink.Bus;
b.Description = props.Description;
b.DataScope   = props.DataScope;
b.HeaderFile  = props.HeaderFile;
b.Alignment   = props.Alignment;
b.Elements = e;
%assignin('base',[prefix name],b);% % Rename the bus object with prefix
end


% Get DataType, Dims and Complexity
function [dt, dims, compl] = getValueAttr(val)
  p = Simulink.Parameter;
  dataValue = val;
  if isa(val,'timeseries')
    dataValue = val.Data;
  end
  p.Value = dataValue;
  compl = p.Complexity;
  dt = p.DataType;
  if isequal(dt, 'auto')
    dt = 'double';
  end
  if isa(val,'timeseries')
    dims = Simulink.SimulationData.TimeseriesUtil.getSampleDimensions(val);
  else
    dims = p.Dimensions;
  end
end

