function downgradeModelsTo2024a(rootDir)
    % DOWNGRADEMODELS Converts all Simulink models from R2024b to R2024a
    % 
    % Author: Newton Campbell
    % Date: 2024
    
    % Default directory if none provided
    if nargin < 1
        rootDir = pwd;
        disp(['No directory specified. Using current directory: ' rootDir]);
    end
    
    % First, find and convert the main BAM.slx in the root directory
    mainModel = fullfile(rootDir, 'BAM.slx');
    if exist(mainModel, 'file')
        disp('Found main simulator model: BAM.slx');
        convertModel(mainModel);
    else
        disp('Main simulator model BAM.slx not found in root directory.');
    end
    
    % Now process the Ref_Models directory
    refModelsDir = fullfile(rootDir, 'Ref_Models');
    if exist(refModelsDir, 'dir')
        disp(['Searching for Simulink models in ' refModelsDir ' and subdirectories...']);
        files = findAllSlxFiles(refModelsDir);
        disp(['Found ' num2str(length(files)) ' Simulink models in Ref_Models.']);
        
        % Process each model file
        for i = 1:length(files)
            convertModel(files{i}, i, length(files));
        end
    else
        disp('Ref_Models directory not found.');
    end
    
    disp('===========================================');
    disp('Conversion process completed.');
    disp('===========================================');
end

function convertModel(modelFile, index, total)
    % Convert a single model file
    [~, modelName, ~] = fileparts(modelFile);
    
    % Display progress
    if nargin >= 3
        disp(['[' num2str(index) '/' num2str(total) '] Processing ' modelName '...']);
    else
        disp(['Processing ' modelName '...']);
    end
    
    try
        % Create a temporary file for output
        tempFile = fullfile(tempdir, ['temp_' num2str(randi(999999)) '.slx']);
        
        % Load the model
        disp(['  Loading ' modelFile '...']);
        load_system(modelFile);
        
        % Get model root
        mdl = bdroot;
        
        % Export to R2024a
        disp('  Converting to R2024a...');
        Simulink.exportToVersion(mdl, tempFile, 'R2024a');
        
        % Close the model
        close_system(mdl, 0);
        
        % Replace the original file with the converted one
        disp('  Replacing original file with converted version...');
        copyfile(tempFile, modelFile, 'f');
        delete(tempFile);
        
        disp(['  SUCCESS: ' modelName]);
        
    catch ME
        disp(['  ERROR converting ' modelName ': ' ME.message]);
        
        % Make sure model is closed
        if bdIsLoaded(modelName)
            close_system(modelName, 0);
        end
        
        % Clean up temp file if it exists
        if exist('tempFile', 'var') && exist(tempFile, 'file')
            delete(tempFile);
        end
    end
end

function files = findAllSlxFiles(dirPath)
    % Find all .slx files recursively
    files = {};
    
    % Get files in current directory
    items = dir(fullfile(dirPath, '*.slx'));
    for i = 1:length(items)
        if ~items(i).isdir
            files{end+1} = fullfile(dirPath, items(i).name);
        end
    end
    
    % Process subdirectories
    subdirs = dir(dirPath);
    for i = 1:length(subdirs)
        if subdirs(i).isdir && ~strcmp(subdirs(i).name, '.') && ~strcmp(subdirs(i).name, '..')
            subFiles = findAllSlxFiles(fullfile(dirPath, subdirs(i).name));
            files = [files subFiles];
        end
    end
end