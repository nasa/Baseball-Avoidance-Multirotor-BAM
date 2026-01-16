function Sdata = readPfile(So,filename)
%function Sdata = readPfile(So,filename);
%
% Reads back values from a serialized binary parameter file 
% Requries prototype structure and filename.
%

fid=fopen(filename,'rb');
if (fid<0), error('Cannot open file: %s',filename); end

% Make flat list of structure
%list = flatNestedStructAccessList(So);
listIn  = listStruct('So',So);
listOut = listStruct('Sdata',So);

 % Read from binary file each field and populate output structure  
for i=1:length(listIn)
    % Check for empty values, these can indicate missing path to class
    % definition and will through off bytecount in binary read
    if eval(sprintf('isempty(%s)',listIn{i}))
        error('Empty Value in SimPar prototype at: %s\n',listIn{i});
    end
    
   % read next value into one a dataVector
   typeStr  = eval(sprintf('class(%s);',listIn{i}));
   fieldLen = eval(sprintf('length(%s(:));',listIn{i}));
   
   % Handle case of enums in SimPar, convert to superclass (which is the
   % underlying storage class
   if eval(sprintf('isenum(%s)',listIn{i}))
      % Determine superclass type on enum... (is there a better way?)
      checkTypes={ 'int8','uint8','int16','uint16','int32','uint32' };
      superClass='';
       for j=1:length(checkTypes)
          if eval(sprintf('isa(%s,''%s'')',listIn{i},checkTypes{j}))
            superClass=checkTypes{j};
          end
      end
    if isempty(superClass),error('Unable to determine superclass for enumerated value:%s',listIn{i}),end
    typeStr=superClass;
   end
   
   dataVec=fread(fid,fieldLen,sprintf('%s=>%s',typeStr,typeStr));
   
   % Handle enums, convert back to original class
   if eval(sprintf('isenum(%s)',listIn{i}))
     clsCast=eval(sprintf('class(%s);',listIn{i}));
     eval(sprintf('dataVec=%s(dataVec);',clsCast));
   end
   
   % Check sizes on prototype field, if matrix reshape, otherwise assign
   Fo=eval(sprintf('%s',listIn{i}));
   if min(size(Fo))~=1 % It's a matrix
     eval(sprintf('%s=reshape(dataVec,size(Fo,1),size(Fo,2));',listOut{i}));
   else
     if (size(Fo,2)>1)  % row vector target, transponse assignment
       eval(sprintf('%s=transpose(dataVec);',listOut{i}));
     else  % Default: column vector or scalar
       eval(sprintf('%s=dataVec;',listOut{i}));
     end
   end
 end
fclose(fid);
