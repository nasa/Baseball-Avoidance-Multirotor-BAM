function [] = parseRspFile(rspfile)
arguments
  rspfile (1,:) char;
end

if exist(rspfile,'file') > 0
  fid = fopen(rspfile);
  tline = fgetl(fid);
  tlines = cell(0,1);
  while ischar(tline)
    if ~strcmp(tline,'rt_malloc_main.obj')
      tlines{end+1,1} = tline;
    else
      disp('Removing rt_malloc_main.obj from command file')
    end
    tline = fgetl(fid);
  end
  fclose(fid);

  % Overwrite command file
  fid = fopen(rspfile,'w','n','UTF-8');
  for i = 1:numel(tlines)
    fprintf(fid,'%s\n',tlines{i,:});
  end
  fclose(fid);
end

end

