function [] = parseMakefile(makefile)
arguments
  makefile (1,:) char;
end

if exist(makefile,'file') > 0
  fid = fopen(makefile);
  tline = fgetl(fid);
  tlines = cell(0,1);
  while ischar(tline)
    tlines{end+1,1} = tline;
    tline = fgetl(fid);
  end
  fclose(fid);
  
  bFoundMainSrc = false;
  mainSrcContains = contains(tlines,'MAIN_SRC =');
  if any(mainSrcContains)
    indx = find(mainSrcContains,1);
    tline = tlines{indx};
    newtline = sprintf('#%s',tline);
    disp('disabling MAIN_SRC line in makefile...');
    tlines{indx} = newtline;
    bFoundMainSrc = true;
  end

  bFoundMainObj = false;
  mainObjContains = contains(tlines,'MAIN_OBJ =');
  if any(mainObjContains)
    indx = find(contains(tlines,'MAIN_OBJ ='),1);
    tline = tlines{indx};
    newtline = sprintf('#%s',tline);
    disp('disabling MAIN_OBJ line in makefile...');
    tlines{indx} = newtline;
    bFoundMainObj = true;
  end
  
  if (bFoundMainSrc || bFoundMainObj)
    % Overwrite makefile file
    fid = fopen(makefile,'w','n','UTF-8');
    for i = 1:numel(tlines)
        fprintf(fid,'%s\n',tlines{i,:});
    end
    fclose(fid);
  end
end

end

