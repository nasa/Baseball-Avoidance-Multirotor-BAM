function setupFiles(userStruct)
% copy the user-specified (or default) output function to a general name used in the
% model
funcFullPath = which(userStruct.outputFunc);
[funcPath, funcName, funcExt] = fileparts(funcFullPath);
[bSuccess,msg] = copyfile([funcPath filesep funcName funcExt], ...
                          [funcPath filesep 'mapToUserSimOut.m'], 'f');
if ~bSuccess
    error(msg)
end
end
