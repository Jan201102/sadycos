classdef GPUshading < ExampleMission.DefaultConfiguration
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = configureParameters@ExampleMission.DefaultConfiguration();

            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.gpu_shading = true;
        end
    end
end