classdef GeneralUtils
    methods(Static)
        function com = determineCoM(poses)

            sumX = 0;
            sumY = 0;
            sumZ = 0;

            numPoses = size(poses, 1);
            for i = 1:numPoses
                pose = poses(i,:);
                sumX = sumX + pose(1);
                sumY = sumY + pose(2);
                sumZ = sumZ + pose(3);
            end

            % Calculate average position
            avgX = sumX / numPoses;
            avgY = sumY / numPoses;
            avgZ = sumZ / numPoses;

            com = [avgX, avgY, avgZ];
        end
        function exportMatricesToCSV(baseDir, csvDir)
            % Create CSV directory if it doesn't exist
            if ~exist(csvDir, 'dir')
                mkdir(csvDir);
            end

            % Process each .mat file
            GeneralUtils.processDirectory(baseDir, baseDir, csvDir);
        end

        function processDirectory(baseDir, currentDir, csvDir)
            items = dir(currentDir);
            for i = 1:length(items)
                if items(i).isdir && ~ismember(items(i).name, {'.', '..'})
                    % If it's a subdirectory, recursively process it
                    GeneralUtils.processDirectory(baseDir, fullfile(currentDir, items(i).name), csvDir);
                elseif ~items(i).isdir
                    % Process .mat file
                    matFilePath = fullfile(currentDir, items(i).name);
                    data = load(matFilePath);
                    fieldName = fieldnames(data);
                    if ~isempty(fieldName)
                        matrix = data.(fieldName{1});
                        if isnumeric(matrix) && size(matrix, 2) == 3
                            % Construct CSV file path
                            relPath = strrep(currentDir, baseDir, ''); % Relative path
                            csvFileName = strrep(items(i).name, '.mat', '.csv');
                            csvFilePath = fullfile(csvDir, relPath, csvFileName);

                            % Ensure subdirectory exists
                            [csvFileDir, ~, ~] = fileparts(csvFilePath);
                            if ~exist(csvFileDir, 'dir')
                                mkdir(csvFileDir);
                            end

                            % Write matrix to CSV
                            GeneralUtils.writeMatrixToCSV(matrix, csvFilePath);
                        end
                    end
                end
            end
        end

        function writeMatrixToCSV(matrix, csvFilePath)
            % Get the number of speeds from the third dimension of the matrix
            numSpeeds = size(matrix, 3);

            for speedIndex = 1:numSpeeds
                % Determine the filename based on the number of speeds
                if numSpeeds > 1
                    % Check if csvFilePath already ends with '.csv', if so, strip it before appending
                    if endsWith(csvFilePath, '.csv')
                        baseFilename = csvFilePath(1:end-4);  % Remove '.csv' from the end
                    else
                        baseFilename = csvFilePath;
                    end
                    filename = sprintf('%s_speed%d.csv', baseFilename, speedIndex);
                else
                    % Check similarly for single speed files
                    if endsWith(csvFilePath, '.csv')
                        filename = csvFilePath;
                    else
                        filename = sprintf('%s.csv', csvFilePath);
                    end
                end

                % Open a new CSV file for writing
                fileId = fopen(filename, 'w');
                if fileId == -1
                    error('Failed to open file for writing: %s', filename);
                end

                % Write each row of the matrix for the current speed
                for i = 1:size(matrix, 1)
                    fprintf(fileId, '%f,%f,%f\n', matrix(i, :, speedIndex));
                end

                % Close the file
                fclose(fileId);
            end
        end

        function projectRoot = findProjectRoot(currentDir, targetDirName)
            % Initialize projectRoot with the current directory
            projectRoot = currentDir;

            % Get the name of the current directory
            [~, currentFolderName] = fileparts(currentDir);

            % Loop until the current folder's name matches targetDirName or we hit the root directory
            while ~strcmp(currentFolderName, targetDirName)
                % Move up one directory level
                projectRoot = fileparts(projectRoot);

                % Break if we've reached the root directory
                if isempty(projectRoot) || strcmp(projectRoot, filesep)
                    error('Target directory "%s" not found in the path.', targetDirName);
                end

                % Update currentFolderName
                [~, currentFolderName] = fileparts(projectRoot);
            end
        end
        function radPerSec = rpmToRadPerSec(rpm)
            % rpmToRadPerSec Converts rotational speed from revolutions per minute to radians per second.
            % Input:
            %   rpm - Rotational speed in revolutions per minute.
            %   radPerSec - Rotational speed in radians per second.
            radPerSec = rpm * (2 * pi / 60);
        end
    end
end