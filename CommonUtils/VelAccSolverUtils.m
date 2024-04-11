classdef VelAccSolverUtils
    methods(Static)
        function Mechanism = VelAccSolver(Mechanism, determineAngVelFunc, determineLinVelFunc, determineAngAccFunc, determineLinAccFunc)

            % Determine the number of iterations (rows in Joints)
            numIterations = size(Mechanism.Joint.A, 1); % Assuming 'A' is a joint in Mechanism.Joints

            % Initialize fields for storing kinematics data
            Mechanism.AngVel = struct();
            Mechanism.LinVel = struct();
            Mechanism.AngAcc = struct();
            Mechanism.LinAcc = struct();

            blankVector = [0 0 0];
            % Initialize blank joint vector dynamically
            jointNames = fieldnames(Mechanism.Joint); % Get all joint names from Mechanism
            initialBlankJointVector = struct();
            for i = 1:length(jointNames)
                initialBlankJointVector.(jointNames{i}) = blankVector;
            end

            % Initialize blank link vector dynamically
            linkNames = fieldnames(Mechanism.LinkCoM); % Get all link names from Mechanism
            initialBlankLinkVector = struct();
            for i = 1:length(linkNames)
                initialBlankLinkVector.(linkNames{i}) = blankVector;
            end


           numSpeeds = size(Mechanism.inputSpeed, 2); % Assuming inputSpeed has dimensions: iterations x speeds
           [Mechanism] = VelAccSolverUtils.initializeAngVels(Mechanism, initialBlankLinkVector, numIterations, numSpeeds);
           [Mechanism] = VelAccSolverUtils.initializeLinVels(Mechanism, initialBlankJointVector, initialBlankLinkVector, numIterations, numSpeeds);
           [Mechanism] = VelAccSolverUtils.initializeAngAccs(Mechanism, initialBlankLinkVector, numIterations, numSpeeds);
           [Mechanism] = VelAccSolverUtils.initializeLinAccs(Mechanism, initialBlankJointVector, initialBlankLinkVector, numIterations, numSpeeds);

            % Iterate through all iterations
            numSpeeds = size(Mechanism.inputSpeed, 2); % Assuming inputSpeed is 2D: iterations x speeds

            for speedIndex = 1:numSpeeds
                for iter = 1:numIterations
                    % Extract joint positions for this iteration
                    JointPos = VelAccSolverUtils.extractJointPositions(Mechanism, iter);
                    LinkCoMPos = VelAccSolverUtils.extractLinkCoMPositions(Mechanism, iter);

                    % Assuming input_speed is defined or extracted from Mechanism
                    input_speed = Mechanism.inputSpeed(iter, speedIndex); % Placeholder, adjust based on your Mechanism structure

                    % Calculate kinematics for the current iteration and store within the Mechanism
                    Mechanism = VelAccSolverUtils.determineKinematics(Mechanism, iter, speedIndex, JointPos, LinkCoMPos, input_speed, determineAngVelFunc, determineLinVelFunc, determineAngAccFunc, determineLinAccFunc);
                end
            end

            % Save the updated Mechanism
            save('Mechanism.mat', 'Mechanism');

            % Define the base folder name for Velocities and Accelerations
            baseVelFolder = 'Kin/Vel';
            baseAccFolder = 'Kin/Acc';

            % Directories for velocities
            linVelJointFolder = fullfile(baseVelFolder, 'LinVel', 'Joint');
            linVelLinkCoMFolder = fullfile(baseVelFolder, 'LinVel', 'LinkCoM');
            angVelFolder = fullfile(baseVelFolder, 'AngVel');

            % Directories for accelerations
            linAccJointFolder = fullfile(baseAccFolder, 'LinAcc', 'Joint');
            linAccLinkCoMFolder = fullfile(baseAccFolder, 'LinAcc', 'LinkCoM');
            angAccFolder = fullfile(baseAccFolder, 'AngAcc');

            % Create the directories if they don't exist
            folders = {linVelJointFolder, linVelLinkCoMFolder, angVelFolder, linAccJointFolder, linAccLinkCoMFolder, angAccFolder};
            for i = 1:length(folders)
                if ~exist(folders{i}, 'dir')
                    mkdir(folders{i});
                end
            end

            % Example usage:
            VelAccSolverUtils.saveData(linVelJointFolder, Mechanism.LinVel.Joint);
            VelAccSolverUtils.saveData(linVelLinkCoMFolder, Mechanism.LinVel.LinkCoM);
            VelAccSolverUtils.saveData(angVelFolder, Mechanism.AngVel);
            VelAccSolverUtils.saveData(linAccJointFolder, Mechanism.LinAcc.Joint);
            VelAccSolverUtils.saveData(linAccLinkCoMFolder, Mechanism.LinAcc.LinkCoM);
            VelAccSolverUtils.saveData(angAccFolder, Mechanism.AngAcc);

        end

        function JointPos = extractJointPositions(Mechanism, iteration)
            % Extract joint positions for a specific iteration
            JointPos = struct();
            jointNames = fieldnames(Mechanism.Joint);
            for i = 1:length(jointNames)
                JointPos.(jointNames{i}) = Mechanism.Joint.(jointNames{i})(iteration, :);
            end
            tracerPointNames = fieldnames(Mechanism.TracerPoint);
            for i = 1:length(tracerPointNames)
                JointPos.(tracerPointNames{i}) = Mechanism.TracerPoint.(tracerPointNames{i})(iteration, :);
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

        % Desired functions to do cross products appropriately
        function vel = velSolver(w, r)
            vel = cross(w,r);
        end
        function acc = accSolver(w,a,r)
            acc = cross(w,cross(w,r)) + cross(a,r);
        end

        % Initialize AngVel, LinVel, AngAcc, LinAcc
        function Mechanism = initializeAngVels(Mechanism, initialBlankLinkVector, numIterations, numSpeeds)
            angVelNames = fieldnames(initialBlankLinkVector);
            for i = 1:length(angVelNames)
                % Allocate a 3D array: iterations x 3 (dimensions) x speeds
                Mechanism.AngVel.(angVelNames{i}) = zeros(numIterations, 3, numSpeeds);
            end
        end
        function Mechanism = initializeLinVels(Mechanism, initialBlankJointVector, initialBlankLinkVector, max_iterations, numSpeeds)
            linJointVelNames = fieldnames(initialBlankJointVector);
            for i = 1:length(linJointVelNames)
                Mechanism.LinVel.Joint.(linJointVelNames{i}) = zeros(max_iterations, 3, numSpeeds); % Initialize with zeros for each dimension (assuming 3D angular velocities)
            end
            linLinkCoMVelNames = fieldnames(initialBlankLinkVector);
            for i = 1:length(linLinkCoMVelNames)
                Mechanism.LinVel.LinkCoM.(linLinkCoMVelNames{i}) = zeros(max_iterations, 3, numSpeeds); % Initialize with zeros for each dimension (assuming 3D angular velocities)
            end
        end
        function Mechanism = initializeAngAccs(Mechanism, initialBlankLinkVector, max_iterations, numSpeeds)
            angAccNames = fieldnames(initialBlankLinkVector);
            for i = 1:length(angAccNames)
                Mechanism.AngAcc.(angAccNames{i}) = zeros(max_iterations, 3, numSpeeds); % Initialize with zeros for each dimension (assuming 3D angular velocities)
            end
        end
        function Mechanism = initializeLinAccs(Mechanism, initialBlankJointVector, initialBlankLinkVector, max_iterations, numSpeeds)
            linJointNames = fieldnames(initialBlankJointVector);
            for i = 1:length(linJointNames)
                Mechanism.LinAcc.Joint.(linJointNames{i}) = zeros(max_iterations, 3, numSpeeds); % Initialize with zeros for each dimension (assuming 3D angular velocities)
            end
            linLinkCoMAccNames = fieldnames(initialBlankLinkVector);
            for i = 1:length(linLinkCoMAccNames)
                Mechanism.LinAcc.LinkCoM.(linLinkCoMAccNames{i}) = zeros(max_iterations, 3, numSpeeds); % Initialize with zeros for each dimension (assuming 3D angular velocities)
            end
        end

        function Mechanism = determineKinematics(Mechanism, iter, speedIndex, JointPos, LinkCoMPos, input_speed, determineAngVelFunc, determineLinVelFunc, determineAngAccFunc, determineLinAccFunc)
            % Determine angular velocity
            [Mechanism, AngVel] = feval(determineAngVelFunc, Mechanism, iter, speedIndex, JointPos, input_speed);

            % Determine linear velocity
            [Mechanism] = feval(determineLinVelFunc, Mechanism, iter, speedIndex, JointPos, LinkCoMPos, AngVel);

            % Determine angular acceleration
            [Mechanism, AngAcc] = feval(determineAngAccFunc, Mechanism, iter, speedIndex, JointPos, AngVel);

            % Determine linear acceleration
            [Mechanism] = feval(determineLinAccFunc, Mechanism, iter, speedIndex, JointPos, LinkCoMPos, AngVel, AngAcc);
        end

        % Save function for clarity and reusability
        function saveData(folder, dataStruct)
            % Ensure the folder exists
            if ~exist(folder, 'dir')
                mkdir(folder);
            end

            names = fieldnames(dataStruct); % Get all field names of the structure
            for i = 1:length(names)
                name = names{i};
                data = dataStruct.(name); % Extract data

                % Create a temporary struct with the extracted data
                tempStruct = struct(name, data);

                % Correctly use the -struct option by providing the variable name of the temporary struct
                save(fullfile(folder, name), '-struct', 'tempStruct', name);
            end
        end
    end
end