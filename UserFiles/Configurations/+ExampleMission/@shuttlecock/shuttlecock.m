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

            %showBodies(bodies, zeros(size(bodies)));

            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies = bodies;

            %set shuttlecock controlsurface angels
            parameters_cells{1}.Actuators.control_surfaces_angles__rad = [0; pi/4; pi/4; pi/4; pi/4]; % constant angles for control surfaces

            %set stoptime
            stop_time_setting = parameters_cells{1}.Settings(1); % StopTime is the first setting
            stop_time_setting.value = "5";
            parameters_cells{1}.Settings(1) = stop_time_setting;
        end

        function BusesInfo = configureBuses(parameters_cells)
            % Call parent method to get base configuration
            BusesInfo = configureBuses@ExampleMission.newmodel(parameters_cells);
            
            % Create new BusesInfoCreator and load existing buses
            busesInfoCreator = BusesInfoCreator(parameters_cells{1});
            
            % Load all existing buses from parent
            for i = 1:length(BusesInfo.buses_list)
                busesInfoCreator.setBus(BusesInfo.buses_list(i).Name, BusesInfo.buses_list(i).Bus);
            end
            
            % Import the helper function
            import BusesInfoCreator.simpleBusElement
            
            % Create control surfaces nested bus
            control_surface_elems = simpleBusElement('control_surface_angles__rad', 5);
            busesInfoCreator.setBusByElements('ControlSurfacesCommands', control_surface_elems);
            
            % Create control surfaces outputs bus
            control_surface_outputs = simpleBusElement('control_surface_angles__rad', 5);
            busesInfoCreator.setBusByElements('ControlSurfacesOutputs', control_surface_outputs);
            
            % Extend ActuatorsCommands with control surfaces
            additional_commands_elem = simpleBusElement('ControlSurfaces', 1, 'Bus: ControlSurfacesCommands');
            busesInfoCreator.extendBusByElements('ActuatorsCommands', additional_commands_elem);
            
            % Extend ActuatorsOutputs with control surfaces
            additional_outputs_elem = simpleBusElement('ControlSurfaces', 1, 'Bus: ControlSurfacesOutputs');
            busesInfoCreator.extendBusByElements('ActuatorsOutputs', additional_outputs_elem);
            
            % Get the updated BusesInfo
            BusesInfo = busesInfoCreator.getBusesInfo();
        end

        [ActuatorsOutputs,...
        LogActuators, ...
        StatesUpdateInput] ...
        = actuators(ActuatorsOutputs, ...
                    LogActuators, ...
                    EnvironmentConditions, ...
                    DynamicsOutputs, ...
                    ActuatorsCommands, ...
                    ActuatorsStates, ...
                    ParametersSatellite)
                                
        [PlantFeedthrough, ...
        LogPlantDynamics, ...    
        PlantStatesDerivatives] ...
        = plantDynamics(PlantFeedthrough, ...
                            LogPlantDynamics, ...
                            EnvironmentConditions, ...
                            ActuatorsOutputs, ...
                            PlantStates, ...
                            ParametersSatellite)
    end
    methods
        function plot_actuator_outputs(obj)
            actuatorsLogs = getElement(obj.simulation_outputs.logsout,"LogActuators");

            actuatorsOutputs = actuatorsLogs.Values.ActuatorsOutputs;
            % Access specific command signals
            controlSurfaceCommands = actuatorsOutputs.ControlSurfaces.control_surface_angles__rad;
            reactionWheelCommands = actuatorsOutputs.ReactionWheels.torque_commands__N_m;
            magneticTorquerCommands = actuatorsOutputs.MagneticTorquers.magnetic_dipole_moment_B__A_m2;

            figure;
            subplot(3,1,1);
            plot(controlSurfaceCommands.Time, controlSurfaceCommands.Data);
            title('Control Surface angles');
            xlabel('Time (s)');
            ylabel('Angle (rad)');
            legend('Surface 1', 'Surface 2', 'Surface 3', 'Surface 4', 'Surface 5');
            grid on;
            subplot(3,1,2);
            plot(reactionWheelCommands.Time, reactionWheelCommands.Data);
            title('Reaction Wheel outputs');
            xlabel('Time (s)');
            ylabel('Torque (N·m)');
            grid on;
            subplot(3,1,3);
            plot(magneticTorquerCommands.Time, magneticTorquerCommands.Data);
            title('Magnetic Torquer outputs');
            xlabel('Time (s)');
            ylabel('Magnetic Dipole Moment (A·m²)');
            grid on;
        end
    end
end