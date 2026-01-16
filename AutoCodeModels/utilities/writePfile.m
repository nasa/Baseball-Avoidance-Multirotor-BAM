function writePfile(Pstruct,filename,validate)
%  Write a structure to a file as packed binary stream.
%
% Arguments
% ---------
% Pstruct : struct
%   SimPar.Value parameter structure, modified for a given setup.
% filename : string
%   Output filename to be loaded into the real-time code
% Validate : logical, default=true
%   If true, checks dataType and Size against expected structure. Requires
%   BUS_PARAM objects be preloaded into the workspace.

arguments
    Pstruct struct
    filename string
    validate logical = true
end

fid=fopen(filename,'wb');
if fid == -1
    errordlg(sprintf('Unable to open file: %s',filename),'Write Failure');
    return;
end
list=listStruct('Pstruct',Pstruct);

if validate
  % Catch data-type or dimension error on existing fields
  busSource = 'BUS_PARAM_SimPar';
  partialValues = Pstruct;
  try
    structFromBus = Simulink.Bus.createMATLABStruct(busSource,partialValues);
  catch ME
    fprintf(2,ME.message);
    fprintf('\n');
  end

  % Catch case where Pstruct has extra or missing fields
  fnOrg = listStruct('S',structFromBus);
  fnMod  = listStruct('S',Pstruct);
  if length(fnOrg) ~= length(fnMod)
    error('Invalid SimPar structure, different total number of fields:   Compiled %d  Modified %d',length(fnOrg),length(fnMod));
  end
  for i=1:length(fnMod)
    if ~strcmp(fnOrg{i},fnMod{i})
      error('Invalid SimPar structure, different field order:  At listStruct position %d, Compiled "%s"   Modified "%s"',i,fnOrg{i},fnMod{i});
    end
  end
end

% Write binary parameter file  
for i=1:length(list)
   vec=eval(list{i});
   if isenum(vec), vec=int32(vec); end
   if ~isreal(vec)
       re = real(vec);
       im = imag(vec);
       vec = reshape([re(:)';im(:)'],2*numel(vec),1);
   end
   fwrite(fid,vec,class(vec));
end

fclose(fid);
