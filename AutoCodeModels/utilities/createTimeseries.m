function [tsStruct, finalOffset] = createTimeseries(busName, initialOffset, signalData, timeData)

offset = initialOffset;
ts = {};

busDef = evalin('base',busName);

num_el = busDef.getNumLeafBusElements;
elemList = busDef.getLeafBusElements;

for j = 1:num_el
  dims = elemList(j).Dimensions;
  if (length(dims) == 1)
      ts{j} = timeseries(signalData.values(:,offset+1:offset+dims),...
                         timeData);
      ts{j}.name = elemList(j).Name;
      offset = offset + dims;
  elseif length(dims)>=2
      time_len = length(timeData);
      array_size = 1;
      for h = 1:length(dims)
          array_size = array_size*dims(h);
      end
      % use the transpose of the signal values to properly handle matrices
      ts{j} = timeseries(reshape(signalData.values(:,offset+1:offset+array_size)',[dims,time_len]),...
          timeData);
      ts{j}.name = elemList(j).Name;
      offset = offset + array_size;
  end
end
tsStruct = Simulink.SimulationData.createStructOfTimeseries(busName,ts);

finalOffset = offset;

end

