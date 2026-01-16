function dataOutType = selectDataOutType()

% select vehicle type

[dataOut_m,dataOut_s]=enumeration('DataOutEnum');
dataOut_len=length(dataOut_m);

fprintf('\n\n');
fprintf('\n---------------------------------------\n')
fprintf('dataOut Type:\n')
for i=1:dataOut_len
  fprintf(' (%d) %s\n', int8(dataOut_m(i)), dataOut_s{i}); 
end
selDataOut = input('Select dataOut: ');

dataOutType = dataOut_m(selDataOut);

fprintf('\n---------------------------------------');
fprintf('\n**  dataOut Type: %s **', dataOutType);
fprintf('\n---------------------------------------\n');
