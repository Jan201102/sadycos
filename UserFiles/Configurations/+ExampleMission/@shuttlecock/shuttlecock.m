classdef shuttlecock < ExampleMission.newmodel
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = configureParameters@ExampleMission.newmodel();
            import vleo_aerodynamics_core.*
            % Get absolute path of test folder
            [test_folder,~,~] = fileparts(mfilename("fullpath"));
            %[test_folder,~,~] = fileparts(matlab.desktop.editor.getActiveFilename);
            gmsh_file = string(fullfile(test_folder, 'obj_files', 'Shuttlecock_copy_BA.m'));
            disp(gmsh_file);
            energy_accommodation_coefficient = parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies{1}.energy_accommodation_coefficients(1);
            surface_temp__K = parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies{1}.temperatures__K(1);
            surface_temperatures__K = num2cell(surface_temp__K*ones(1,5));
            surface_energy_accommodation_coefficients = num2cell(energy_accommodation_coefficient*ones(1,5));
            lx__m = 0.3; % Length of the shuttlecock in meters
            ly__m = 0.1; % Width of the shuttlecock in meters
            lz__m = 0.1; % Height of the shuttlecock in meters
            mass__kg = 0.907;
            inertia__kg_m2 = mass__kg/12 * [ly__m^2 + lz__m^2, 0, 0;...
                                        0, lx__m^2 + lz__m^2, 0;...
                                        0, 0, lx__m^2 + ly__m^2];

            % 1. RigidBodyMechanics model parameters
            parameters_cells{1}.Plant.RigidBodyMechanics.mass__kg = mass__kg;
            parameters_cells{1}.Plant.RigidBodyMechanics.inertia_B_B__kg_m2 = inertia__kg_m2;
            
            % 2. PointMassGravity model parameters  
            parameters_cells{1}.Plant.PointMassGravity.mass__kg = mass__kg;

            rotation_hinge_points_CAD = [0,-0.15,-0.15,-0.15,-0.15;...
                                        0,0.05,0,-0.05,0;...
                                        0,0,-0.05,0,0.05];

            rotation_directions_CAD = [0,0,0,0,0;...
                                        1,0,-1,0,1;...
                                        0,-1,0,1,0];

            DCM_B_from_CAD = eye(3);
            CoM_CAD = [0; 0; 0.0];

            bodies = importMultipleBodies(gmsh_file,...
            rotation_hinge_points_CAD, ...
            rotation_directions_CAD, ...
            surface_temperatures__K, ...
            surface_energy_accommodation_coefficients, ...
            DCM_B_from_CAD, ...
            CoM_CAD);

            showBodies(bodies, zeros(size(bodies)));

            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies = bodies;
        end
        
     
    end
end