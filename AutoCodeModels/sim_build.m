function systemCmd = sim_build(options)
    % Convenience function to compile the BAM simulation.
    %
    % sim_build is expected to be run from the top-level directory using
    % keyword arguments, for example,
    %   sim_build(workInBackground=true)
    % This function is meant to aid the user in autocoding the BAM
    % simulation by providing default options. For more customization
    % options, see AutoCodeModels.
    %
    % sim_build compiles an executable or static library from the BAM
    % simulation by calling AutoCodeModels/buildExe.m or
    % AutoCodeModels/buildLib.m, respectively. These options are specified
    % by setting buildTarget='executable' (default) or
    % buildTarget='library', respectively. The simulation is expected to be
    % set up using the user-specified setupScript option (default='setup').
    %
    % sim_build works by constructing and calling a system command to start
    % a new instance of MATLAB and run the setupScript and build function
    % there. Sensible defaults are created for Windows, MacOS, and Linux
    % systems, but system commands can be highly-system specific, so
    % several options are provided to modify these. Overall, the system
    % command is a string given by
    %   systemCmd = sprintf('%s %s %s -batch "%s;%s"', ...
    %                       shellCmd, matlabCmd, licenseOpt, 
    %                       setupScript, buildScript);
    % where licenseOpt = ['-c ', licensePath] if licensePath is specified,
    % or '' if licensePath is empty, and buildScript = 'buildExe' or 
    % 'buildLib' if buildTarget = 'executable' or 'library', respectively.
    % Explanations of each argument are provided below.
    %
    % KEYWORD ARGUMENTS
    % -----------------
    % setupScript : (1, :) char, default='setup'
    %   Name of the script used to set up the BAM simulation.
    % buildTarget : {'executable', 'library'}, default='executable'
    %   Specifies whether to generate a standalone executable or a static
    %   library.
    % licensePath : (1, :) char, optional
    %   Optional filepath to pass to the -c option when starting the new
    %   MATLAB instance. Can be helpful in managing license usage during
    %   model compilation.
    % shellCmd : (1, :) char, optional
    %   Commands to append to the start of systemCmd passed to the system
    %   function. For example, on Linux one could use shellCmd='xterm -e'
    %   to run the command in a new xterm window.
    % matlabCmd : (1, :) char, optional
    %   Non-default command to start MATLAB from the command line. On
    %   Windows and Linux, the default is matlabCmd='matlab'. On MacOS, the
    %   default is matlabCmd=[matlabroot, '/bin/matlab'].
    % workInBackground : logical,  default=false
    %   If workInBackground=true, appends a '&' to systemCmd to run the
    %   command silently in the background.
    %
    % RETURNS
    % -------
    % systemCmd : (1, :) char
    %   String passed to the system command for debugging/modification for
    %   the user's system.
    %
    % AUTHORS
    % -------
    % Thomas C. Britton
    %   Science and Technology Corporation (STC), RSES Contract,
    %   NASA Langley Research Center
    % Tenavi Nakamura-Zimmerer
    %   NASA Langley Research Center
    arguments
        options.setupScript (1, :) char = 'setup';
        options.buildTarget (1, :) char {mustBeMember(options.buildTarget, {'executable', 'library'})} = 'executable'
        options.licensePath (1, :) char = '';
        options.shellCmd (1, :) char = ''
        options.matlabCmd (1, :) char = ''
        options.workInBackground (1, 1) logical = false
    end
    
    % check if AutoCodeModels folder is on the Matlab path
    pathCell = regexp(path, pathsep, 'split');
    onPath = any(contains(pathCell, 'AutoCodeModels'));
    if onPath == false
        addpath(genpath('AutoCodeModels'));
    end
    
    fprintf('setupScript = %s, buildTarget = %s\n', ...
            options.setupScript, options.buildTarget);
    
    switch options.buildTarget
        case 'executable'
            buildScript = 'buildExe';
        case 'library'
            buildScript = 'buildLib';
        otherwise
            error('Unknown build target:  %d\n', options.buildTarget);
    end

    % Default license option is left unspecified
    if isempty(options.licensePath)
        licenseOpt = '';
    else
        licenseOpt = sprintf('-c %s', options.licensePath);
    end

    % Default command to launch matlab from a shell
    if isempty(options.matlabCmd)
        if ismac
            matlabCmd = fullfile(matlabroot, 'bin', 'matlab');
        else
            matlabCmd = 'matlab';
        end
    else
        matlabCmd = options.matlabCmd;
    end

    systemCmd = sprintf( ...
        '%s %s %s -batch "%s;%s"', ...
        options.shellCmd, matlabCmd, licenseOpt, options.setupScript, ...
        buildScript);

    if options.workInBackground
        systemCmd = sprintf('%s &', systemCmd);
    end
    
    fprintf('Performing %s code generation with system command\n%s\n', ...
           options.buildTarget, systemCmd)

    % spawn another instance of matlab for code generation
    tic
    system(systemCmd);
    if ~options.workInBackground
        toc
    end
end
