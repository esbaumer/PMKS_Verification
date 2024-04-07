classdef ForceSolverUtils
    methods(Static)
        function Mechanism = ForceSolver(Mechanism, scenarios, performForceAnalysisFunc)

            % Assuming numIterations is defined by the size of an array in Mechanism
            numIterations = size(Mechanism.Joint.A, 1);

            % Initialize fields for storing static analysis data
            [Mechanism] = ForceSolverUtils.initializeForceSolvers(Mechanism, numIterations);

            % Iterate through all iterations for static analysis
            for iter = 1:numIterations
                % Extract joint and link center of mass positions for this iteration
                JointPos = ForceSolverUtils.extractJointPositions(Mechanism, iter);
                LinkCoMPos = ForceSolverUtils.extractLinkCoMPositions(Mechanism, iter);

                % Scenarios: [newtonFlag, gravityFlag, frictionFlag]
                % scenarios = [0 0 0; 0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0; 1 1 1];
                for scenario = scenarios.'
                    Mechanism = ForceSolverUtils.updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, scenario(1), scenario(2), scenario(3), performForceAnalysisFunc);
                end
            end

            % Save the updated Mechanism with static analysis results
            save('Mechanism.mat', 'Mechanism');

            baseFolder = 'Force';
            % Save Force Data
            ForceSolverUtils.saveForceData(baseFolder, Mechanism);
        end

        function Mechanism = updateMechanismForces(Mechanism, iter, JointPos, LinkCoMPos, newtonFlag, gravityFlag, frictionFlag, performForceAnalysisFunc)
            % Define the suffix based on the provided flags for readability
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

            % Perform force analysis
            solution = feval(performForceAnalysisFunc, Mechanism, iter, JointPos, LinkCoMPos, newtonFlag, gravityFlag, frictionFlag);
            jointNames = fieldnames(Mechanism.Joint);

            % Update forces and torques in the mechanism structure
            for i = 1:length(jointNames)
                jointName = jointNames{i};
                Mechanism.(suffix).Joint.(jointName)(iter, :) = [double(solution.([jointName, 'x'])), double(solution.([jointName, 'y'])), 0];
            end
            Mechanism.(suffix).Torque(iter,:) = [0 0 double(solution.T)];
        end
        function [Mechanism] = initializeForceSolvers(Mechanism, numIterations)
            % Initialize with zeros for storing forces and moments
            jointNames = fieldnames(Mechanism.Joint);
            for i = 1:length(jointNames)
                Mechanism.StaticForceGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.StaticForceNoGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.NewtonForceGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.NewtonForceNoGravNoFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.StaticForceGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.StaticForceNoGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.NewtonForceGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
                Mechanism.NewtonForceNoGravFriction.Joint.(jointNames{i}) = zeros(numIterations, 3); % Assuming 3D forces
            end
            Mechanism.StaticForceGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceNoGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceNoGravNoFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceNoGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceNoGravFriction.Torque = zeros(numIterations, 3); % Assuming 3D forces

            % Only for the slider will we need to include the normal force
            Mechanism.StaticForceGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceNoGravNoFriction.NormalForce= zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceNoGravNoFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.StaticForceNoGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
            Mechanism.NewtonForceNoGravFriction.NormalForce = zeros(numIterations, 3); % Assuming 3D forces
        end
        function JointPos = extractJointPositions(Mechanism, iteration)
            % Extract joint positions for a specific iteration
            JointPos = struct();
            jointNames = fieldnames(Mechanism.Joint);
            for i = 1:length(jointNames)
                JointPos.(jointNames{i}) = Mechanism.Joint.(jointNames{i})(iteration, :);
            end
        end
        function LinkCoMPos = extractLinkCoMPositions(Mechanism, iteration)
            % Extract link center of mass positions for a specific iteration
            LinkCoMPos = struct();
            linkNames = fieldnames(Mechanism.LinkCoM);
            for i = 1:length(linkNames)
                LinkCoMPos.(linkNames{i}) = Mechanism.LinkCoM.(linkNames{i})(iteration, :);
            end
        end
        function pos = momentVec(pos, fixPos, force)
            % Position Vector
            r = pos - fixPos;
            pos = cross(r,force);
        end
        function saveForceData(baseFolder, Mechanism)
            % Define categories, conditions, and friction states
            categories = {'Static', 'Newton'};
            conditions = {'Grav', 'NoGrav'};
            frictions = {'Friction', 'NoFriction'};

            % Iterate through each combination of categories, conditions, and frictions
            for iCategory = 1:length(categories)
                for iCondition = 1:length(conditions)
                    for iFriction = 1:length(frictions)
                        category = categories{iCategory};
                        condition = conditions{iCondition};
                        friction = frictions{iFriction};

                        % Construct force field name e.g., StaticForceGravFriction
                        forceFieldName = [category 'Force' condition friction];

                        % Prepare folders for Joint and Torque
                        jointFolder = fullfile(baseFolder, [category 'Force'], condition, friction, 'Joint');

                        % Ensure folders exist
                        if ~exist(jointFolder, 'dir')
                            mkdir(jointFolder);
                        end

                        % Process and save Joint data
                        if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Joint')
                            jointNames = fieldnames(Mechanism.(forceFieldName).Joint);
                            for iJoint = 1:length(jointNames)
                                jointName = jointNames{iJoint};
                                tempStruct = struct(jointName, Mechanism.(forceFieldName).Joint.(jointName));
                                save(fullfile(jointFolder, jointName), '-struct', 'tempStruct', jointName);
                            end
                        end

                        % Process and save Torque data
                        if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'Torque')
                            torqueFilePath = fullfile(baseFolder, [category 'Force'], condition, friction, 'Torque.mat');
                            Torque = Mechanism.(forceFieldName).Torque;
                            save(torqueFilePath, 'Torque');
                        end

                        % Process and save Normal Force data
                        if isfield(Mechanism, forceFieldName) && isfield(Mechanism.(forceFieldName), 'NormalForce')
                            normalForceFilePath = fullfile(baseFolder, [category 'Force'], condition, friction, 'NormalForce.mat');
                            NormalForce = Mechanism.(forceFieldName).NormalForce; % Extract normal force data
                            save(normalForceFilePath, 'NormalForce');
                        end
                    end
                end
            end
        end
    end
end
