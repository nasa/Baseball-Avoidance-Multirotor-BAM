bExist = isfield(userStruct, 'outputFunc');
if (bExist && ...
    (ischar(userStruct.outputFunc) || isstring(userStruct.outputFunc)) && ...
    ~isempty(userStruct.outputFunc))
  % disp('userStruct.outputFunc exists, using user-specified output function.');
else
  % disp('userStruct.outputFunc does not exist, using full sim output function.');
  userStruct.outputFunc = 'userOutFull';
end

clear bExist;