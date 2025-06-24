classdef newmodel < ExampleMission.DefaultConfiguration
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = repmat(configureParameters@ExampleMission.DefaultConfiguration(),2,1);
            parameters_cells{2}.Plant.SimplifiedVleoAerodynamics.model = 2;
            [this_folder,~,~] = fileparts(mfilename("fullpath"));
            lut_path = string(fullfile(this_folder, 'aerodynamic_coefficients_panel_method.csv'));
            LUT_data = readmatrix(lut_path);
            parameters_cells{2}.Plant.SimplifiedVleoAerodynamics.LUT_data = LUT_data;

            % PARAMETERS FROM DSMC
            n = 4.698e14;
            temperature__K = 934;
            surface_temperature__K = 300;
            particles_mass__kg = 16 * 1.6605390689252e-27;
            density__kg_per_m3 = particles_mass__kg * n;
            orbital_velocity__m_per_s = 7800;
            energy_accommodation = (7.5e-17 * n * temperature__K)/(1+7.5e-17 * n * temperature__K);
            %END PARAMETERS FROM DSMC
            gravitational_parameter_Earth = 3.986004e14;

            for i = 1:2
                %set correct parameters for the geometric model
                parameters_cells{i}.Plant.SimplifiedVleoAerodynamics.bodies{1}.energy_accommodation_coefficients = energy_accommodation*ones(1,12);
                parameters_cells{i}.Plant.SimplifiedVleoAerodynamics.bodies{1}.temperatures__K = surface_temperature__K*ones(1,12);

                %add correct atmospheric parameters
                parameters_cells{i}.Environment.atmospheric_mass_density__kg_per_m3 = density__kg_per_m3;
                parameters_cells{i}.Environment.atmospheric_number_density__1_per_m3 = n;
                parameters_cells{i}.Environment.atmospheric_temperature__K = temperature__K;

                %set correct orbital velocity
                %parameters_cells{i}.General.States.InitialStates.Plant.RigidBody.velocity_BI_I__m_per_s = [ 0 ; orbital_velocity__m_per_s ; 0 ];
                %parameters_cells{i}.General.States.InitialStates.Plant.RigidBody.position_BI_I__m = [ gravitational_parameter_Earth/(orbital_velocity__m_per_s^2) ; 0; 0];
                parameters_cells{i}.General.States.InitialStates.Plant.RigidBody.velocity_BI_I__m_per_s = [ orbital_velocity__m_per_s ; 0 ; 0 ];
                parameters_cells{i}.General.States.InitialStates.Plant.RigidBody.position_BI_I__m = [0 ; 0; -gravitational_parameter_Earth/(orbital_velocity__m_per_s^2)];
                
                %reduce sim time for faster verification
                stop_time_setting = parameters_cells{i}.Settings(1); % StopTime is the first setting
                stop_time_setting.value = "500"; % Change from default 1000s to 500s
                parameters_cells{i}.Settings(1) = stop_time_setting;
            end
        end

        % Override environment function to ensure correct atmoshperic parameters for new model
        [EnvironmentConditions, ...
            LogEnvironment, ...
            EnvironmentStatesDerivatives] ...
            = environment(EnvironmentConditions, ...
                            LogEnvironment, ...
                            EnvironmentStatesDerivatives, ...
                            PlantOutputs, ...
                            simulation_time__s, ...
                            EnvironmentStates, ...
                            ParametersEnvironment)

        
        function BusesInfo = configureBuses(parameters_cells)

            num_simulations = numel(parameters_cells);
            BusesInfo = repmat(struct('buses_list',{},'BusTemplates',{}), 1, num_simulations);

            for index = 1:num_simulations
                BusesInfo(index) = configureBuses@ExampleMission.DefaultConfiguration(parameters_cells(index));                
            end

        end

        function simulation_inputs = configureSimulationInputs(parameters_cells, BusesInfo)

            num_simulations = numel(parameters_cells);
            simulation_inputs(num_simulations) = Simulink.SimulationInput;

            for index = 1:num_simulations
                simulation_inputs(index) = configureSimulationInputs@ExampleMission.DefaultConfiguration(parameters_cells(index), BusesInfo(index));
            end

        end

    end
    methods (Access = public)
        function fig = plotRms(obj)
            error_sentman = getElement(obj.simulation_outputs(1).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;
            error_new = getElement(obj.simulation_outputs(2).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;
            [~, angles_sentman] = smu.unitQuat.rot.toAxisAngle((sign(error_sentman.Data(:,1)) .* error_sentman.Data)');
            [~, angles_new] = smu.unitQuat.rot.toAxisAngle((sign(error_new.Data(:,1)) .* error_new.Data)');

            fig = figure;
            plot(error_sentman.Time,angles_sentman, 'LineWidth', 2);
            hold on;
            plot(error_new.Time, angles_new,'LineWidth', 2);
            grid on;
            ylabel('Angle (rad)');
            xlabel('Time (s)');
            legend('Sentman', 'New Model');
            title('Quaternion Error Comparison');
        end
    end
end