function [] = parseCMakeLists(cmakelistsfile, rosPackages, exeFlag)
% function [] = parseCMakeLists(cmakelistsfile, rosPackages, exeFlag)
%
% Used to parse auto-generated CMakeLists.txt file, and insert find_package
% statements, library dependecies, and include directories based on
% required ROS2 packages.  Also sets the C++ standard to C++17.
%
% Inputs:
%   cmakelistsfile - character array containing the full path and filename
%                    for the auto-generated CMakeLists.txt file
%   rosPackages - array of strings containing the names of the ROS2
%                 packages that are required
%   exeFlag - boolean flag controlling the install target path 
%
% AUTHORS:
%   Thomas C. Britton
%     Science and Technology Corporation (STC), RSES Contract,
%     NASA Langley Research Center
% 
% Revisions:
%   tbritton 20250519 - Initial version
%   tbritton 20250605 - Modified to insert include directories
arguments
  cmakelistsfile (1,:) char = 'CMakeLists.txt';
  rosPackages (:,1) string = strings(0,1);
  exeFlag logical = false;
end

if exist(cmakelistsfile,'file') > 0
  fid = fopen(cmakelistsfile);
  tline = fgetl(fid);
  tlines = cell(0,1);
  
  numRosPackages = 0;
  if ~isempty(rosPackages)
    numRosPackages = length(rosPackages);
  end

  while ischar(tline)
    % insert ros libraries into target_link_libraries
    if (numRosPackages > 0) && contains(tline, '$<$<BOOL:${FOUND_LIBM}>:m>')
      pkglibs = '';
      for idx = 1:numRosPackages
        fprintf('Inserting ROS2 library, %s, for target_link_libraries...\n', rosPackages(idx,:));
        pkglibs = sprintf('%s${%s_LIBRARIES} ', pkglibs, rosPackages(idx,:));
      end
      k = strfind(tline,'$<$<BOOL:${FOUND_LIBM}>:m>');
      newLine = insertAfter(tline, k-1, pkglibs);
      tline = newLine;
    end
  
    % insert CXX 17 standard
    if contains(tline, 'c_std_99')
      newLine = insertAfter(tline, 'c_std_99', ' cxx_std_17');
      tline = newLine;
    end
  
    % add the line to the cell array of lines
    tlines{end+1,1} = tline;
  
    % insert find_package calls after find_library
    if (numRosPackages > 0) && contains(tline, 'find_library')
      for idx = 1:numRosPackages
        fprintf('Inserting ROS2 find_package call for %s...\n', rosPackages(idx,:));
        pkg = sprintf('find_package(%s REQUIRED)', rosPackages(idx,:));

        % add the line for find_package for the ROS2 package
        tlines{end+1,1} = pkg;
      end
    end

    % insert ROS2 package includes
    if (numRosPackages > 0) && contains(tline, 'target_include_directories')
      for idx = 1:numRosPackages
        fprintf('Inserting ROS2 include dirs for %s...\n', rosPackages(idx,:));
        pkg_include = sprintf('${%s_INCLUDE_DIRS}', rosPackages(idx,:));
        
        % add the line for the ROS2 package include directory
        tlines{end+1,1} = pkg_include;
      end
    end
    
    % get the next line
    tline = fgetl(fid);
  end  % while ischar(tline)

  % insert install target at the end of the file
  disp('Inserting install target...')
  tlines{end+1,1} = '';
  if (exeFlag == true)
    tlines{end+1,1} = 'install(TARGETS ${PROJECT_NAME} DESTINATION bin)';
  else
    tlines{end+1,1} = 'install(TARGETS ${PROJECT_NAME} DESTINATION lib)';
  end

  fclose(fid);

  % Overwrite CMakeLists.txt file
  fid = fopen(cmakelistsfile,'w','n','UTF-8');
  for i = 1:numel(tlines)
    fprintf(fid,'%s\n',tlines{i,:});
  end
  fclose(fid);
end

end

