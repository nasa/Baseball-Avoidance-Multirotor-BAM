%% In the event that byte packing has to be addressed, due to the use of
%% boolean types within the SimPar variable, an approach like this can work,
%% with inserting pragma statements around the structure definitions.  The
%% initial code generation has to be done with the GenCodeOnly parameter set
%% to 'on' though, and then re-run rtwbuild after the header file is
%% modified but with the GenCodeOnly parameter set to 'off'.

function bytePacking(rtw_buildfolder)
arguments
  rtw_buildfolder (1,:) char;
end

%% Add #pragma to Parameters.h file
disp(' ')
disp('Pragma-stanza added to Parameters.h file.  This will not show up in the html.')
disp(' ')
paramfile = sprintf('%s/Parameters.h', rtw_buildfolder);

ParHFstr = importdata(paramfile);% Read contents of Parameters.h
% Insert #pragmas.
indx = find(~cellfun(@isempty,strfind(ParHFstr,'typedef')),1);
ParHFstr = [ParHFstr(1:indx-1);
    ' ';
    '#pragma pack(1)';
    ParHFstr(indx:end-1,1);
    '#pragma pack()';
    ' ';
    ParHFstr(end,1)];
% Overwrite Parameters.h file.
fileID = fopen(paramfile,'w');
for i = 1:numel(ParHFstr)
    fprintf(fileID,'%s\n',ParHFstr{i,:});
end
fclose(fileID);

end

