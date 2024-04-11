classdef StressSolverUtils
    methods(Static)
        function Mechanism = StressSolver(Mechanism, scenarios)
            % Initialize stress analysis storage
            Mechanism = StressSolverUtils.initializeStressAnalysisStorage(Mechanism);

            %            scenario(1), scenario(2), scenario(3)
            % Iterate over each scenario
            for scenarioIndex = 1:length(scenarios(1))
                % Extract current scenario flags
                newtonFlag = scenarios(scenarioIndex, 1);
                gravityFlag = scenarios(scenarioIndex, 2);
                frictionFlag = scenarios(scenarioIndex, 3);

                % Construct suffix based on current scenario
                suffix = '';
                if newtonFlag
                    suffix = [suffix, 'NewtonForce'];
                else
                    suffix = [suffix, 'StaticForce'];
                end
                if gravityFlag
                    suffix = [suffix, 'Grav'];
                else
                    suffix = [suffix, 'NoGrav'];
                end
                if frictionFlag
                    suffix = [suffix, 'Friction'];
                else
                    suffix = [suffix, 'NoFriction'];
                end

                % Iterate over links to perform stress analysis
                linkNames = fieldnames(Mechanism.LinkCoM);
                for i = 1:length(linkNames)
                    linkID = linkNames{i};
                    % Pass the current scenario to performLinkStressAnalysis
                    Mechanism = StressSolverUtils.performLinkStressAnalysis(Mechanism, linkID, suffix);
                end

                baseFolder = 'Stress';
                % Save Force Data
                StressSolverUtils.saveStressData(baseFolder, Mechanism);
            end
        end

        function Mechanism = initializeStressAnalysisStorage(Mechanism)
            linkNames = fieldnames(Mechanism.LinkCoM);
            numIterations = size(Mechanism.Joint.A, 1); % Assuming 'A' joint has the full number of iterations
            % TODO: pass in the scneario and update that from scenario to
            % adjust for hardcoding
            for i = 1:length(linkNames)
                linkID = linkNames{i};
                Mechanism.Stress.StaticForceGravNoFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceNoGravNoFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceGravNoFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceNoGravNoFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceGravFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceNoGravFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceGravFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceNoGravFriction.axialStress.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceGravNoFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceNoGravNoFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceGravNoFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceNoGravNoFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceGravFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.StaticForceNoGravFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceGravFriction.deformation.(linkID) = zeros(numIterations, 1);
                Mechanism.Stress.NewtonForceNoGravFriction.deformation.(linkID) = zeros(numIterations, 1);
            end
        end

        function Mechanism = performLinkStressAnalysis(Mechanism, linkID, forceAnalysisType)
            jointNames = {};

            % Split linkID into its comprising joint names
            % Assuming linkID format like 'ABE', 'BCFG', etc.
            for i = 1:length(linkID)
                currentJointName = linkID(i); % Current joint name as a character

                % Check if this joint name is present in the Mechanism.(forceAnalysisType).Joint
                if isfield(Mechanism.(forceAnalysisType).Joint, currentJointName)
                    jointNames{end+1} = currentJointName; % Add if it's part of the force analysis
                end
            end

            % Continue with the logic only if there are joints to analyze
            if isempty(jointNames)
                warning('No joints from %s are involved in %s analysis.', linkID, forceAnalysisType);
                return; % Exit if no relevant joints found
            end

            % Accessing beamLength and crossSectionalArea based on linkID
            beamLengthProperty = strcat(linkID, 'Length');
            crossSectionalAreaProperty = strcat('crossSectionalArea', linkID);
            beamLength = Mechanism.(beamLengthProperty);
            crossSectionalArea = Mechanism.(crossSectionalAreaProperty);
            modulusElasticity = Mechanism.modulusElasticity; % Same for all links


            for iter = 1:size(Mechanism.Joint.A, 1)
                % Assuming the first and last letters in linkID correspond to the actual joint positions
                A = Mechanism.Joint.(jointNames{1})(iter, :); % Start joint of the link
                B = Mechanism.Joint.(jointNames{end})(iter, :); % End joint of the link

                % Extract force acting on the first joint in the link for the current iteration
                Fa = Mechanism.(forceAnalysisType).Joint.(jointNames{1})(iter, :);

                % Perform stress analysis
                [axialStress, deformation] = StressSolverUtils.performStressAnalysis(A, B, Fa, crossSectionalArea, beamLength, modulusElasticity);

                % Store results
                Mechanism.Stress.(forceAnalysisType).axialStress.(linkID)(iter) = axialStress;
                Mechanism.Stress.(forceAnalysisType).deformation.(linkID)(iter) = deformation;
            end
        end

        function [axialStress, deformation] = performStressAnalysis(A, B, Fa, crossSectionalArea, beamLength, modulusElasticity)
            posVec = B - A; % Calculate position vector for beam
            unitVector = posVec / norm(posVec); % Calculate the unit vector for the beam
            Fnormal = dot(Fa, unitVector); % Find the Axial Force by using the dot product
            axialStress = Fnormal / crossSectionalArea; % Pa  % Determine axial stress of section AB
            deformation = (Fnormal * beamLength) / (crossSectionalArea * modulusElasticity); % meters  % Determine the deformation for section AB
        end

        % function saveStressData(baseFolder, Mechanism)
        %     % Define categories for stress analysis
        %     categories = {'AxialStress', 'Deformation'};
        % 
        %     % Iterate through each category
        %     for iCategory = 1:length(categories)
        %         category = categories{iCategory};
        % 
        %         % Construct the base directory path for the current category
        %         categoryFolder = fullfile(baseFolder, category);
        % 
        %         % Ensure the base directory exists
        %         if ~exist(categoryFolder, 'dir')
        %             mkdir(categoryFolder);
        %         end
        % 
        %         % Depending on the category, choose the appropriate field from Mechanism
        %         switch category
        %             case 'AxialStress'
        %                 dataField = Mechanism.axialStress;
        %             case 'Deformation'
        %                 dataField = Mechanism.deformation;
        %         end
        % 
        %         % Process and save data for each link
        %         linkNames = fieldnames(dataField);
        %         for iLink = 1:length(linkNames)
        %             linkName = linkNames{iLink};
        %             % Prepare folder for the current link within the category
        %             linkFolder = fullfile(categoryFolder, linkName);
        % 
        %             % Ensure folder exists
        %             if ~exist(linkFolder, 'dir')
        %                 mkdir(linkFolder);
        %             end
        % 
        %             % Prepare data to be saved
        %             data = dataField.(linkName);
        % 
        %             % Save the data to a .mat file named after the link
        %             dataFilePath = fullfile(linkFolder, [linkName '.mat']);
        %             save(dataFilePath, 'data');
        %         end
        %     end
        % end
        % 
               function saveStressData(baseFolder, Mechanism)
            % Define stress analysis scenarios
            scenarios = {
                'StaticForceNoGravNoFriction', 'Static/NoGrav/NoFriction';
                'StaticForceNoGravFriction', 'Static/NoGrav/Friction';
                'StaticForceGravNoFriction', 'Static/Grav/NoFriction';
                'StaticForceGravFriction', 'Static/Grav/Friction';
                'NewtonForceNoGravNoFriction', 'Newton/NoGrav/NoFriction';
                'NewtonForceNoGravFriction', 'Newton/NoGrav/Friction';
                'NewtonForceGravNoFriction', 'Newton/Grav/NoFriction';
                'NewtonForceGravFriction', 'Newton/Grav/Friction';
            };

            % Iterate over each stress analysis scenario
            for iScenario = 1:size(scenarios, 1)
                scenarioField = scenarios{iScenario, 1};
                scenarioPath = scenarios{iScenario, 2};

                % Construct the full path for the current scenario
                fullScenarioPath = fullfile(baseFolder, 'Stress', scenarioPath);

                % Ensure the directory exists
                if ~exist(fullScenarioPath, 'dir')
                    mkdir(fullScenarioPath);
                end

                % Check if the current scenario exists in the Mechanism
                if isfield(Mechanism.Stress, scenarioField)
                    % Get the data for the current scenario
                    data = Mechanism.Stress.(scenarioField);

                    % Save the data to a .mat file within the scenario's directory
                    dataFilePath = fullfile(fullScenarioPath, [scenarioField '.mat']);
                    save(dataFilePath, 'data');
                end
            end
        end
    end
end
