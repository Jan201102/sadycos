classdef newmodel < ExampleMission.DefaultConfiguration
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = configureParameters@ExampleMission.DefaultConfiguration();
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.model = 2;
            [this_folder,~,~] = fileparts(mfilename("fullpath"));
            lut_path = string(fullfile(this_folder, 'aerodynamic_coefficients_panel_method.csv'));
            LUT_data = readmatrix(lut_path);
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.LUT_data = LUT_data;
        end
    end
end