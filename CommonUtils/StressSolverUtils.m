classdef StressSolverUtils
    methods(Static)
        function Mechanism = StressSolver(Mechanism)
        
            % First, we want to initialize the mechanism to contain empty
            % number of rows. The desired info will be like this
            % Mechanism.axialStress.(LinkID) = zeros(361,1)
            % Mechanism.deformation.(LinkID) = zeros(361,1)


            % Want to iterate through each link and determine the stress
            % analysis accordingly

            % Example of what each analysis should look like
            % [axialStress, deformation] = performStressAnalysis(A, B, Fa, crossSectionalArea, beamLength, modulusElasticity)

            % Based on each link, store the desired link info in the
            % desired position
            % Mechanism.axialStress.(LinkID) = axialStress;
            % Mechanism.deformation.(LinkID) = deformation;
        end
        
        function [axialStress, deformation] = performStressAnalysis(A, B, Fa, crossSectionalArea, beamLength, modulusElasticity)
            % Calculate position vector for beam
            posVec = [B(1) - A(1), B(2) - A(2)];

            % Calculate the unit vector for the beam
            unitVector = posVec / norm(posVec);

            % Find the Axial Force by using the dot product
            Fnormal = dot(Fa, unitVector);

            % Determine axial stress of section AB
            axialStress = Fnormal / crossSectionalArea; % Pa

            % Determine the deformation for section AB
            deformation = (Fnormal * beamLength) / (crossSectionalArea * modulusElasticity);
        end
    end
end
