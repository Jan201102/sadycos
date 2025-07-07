classdef geometry < ExampleMission.shuttlecock
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = repmat(configureParameters@ExampleMission.shuttlecock(),4,1);
            newmodel_params = configureParameters@ExampleMission.newmodel();
            parameters_cells{1} = newmodel_params{1};
            parameters_cells{2} = newmodel_params{1};
            % set sentman model for 2nd simulation
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.model = 1; 
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.LUT_data = [];
            parameters_cells{3}.Plant.SimplifiedVleoAerodynamics.model = 1;
            parameters_cells{3}.Plant.SimplifiedVleoAerodynamics.LUT_data = [];
            
            stop_time_setting = parameters_cells{1}.Settings(1); % StopTime is the first setting
            stop_time_setting.value = "1000"; % Change from default 1000s to 500s
            parameters_cells{1}.Settings(1) = stop_time_setting;
        end
        
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
            error_shuttle_sentman = getElement(obj.simulation_outputs(3).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;
            error_shuttle_new = getElement(obj.simulation_outputs(4).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;
            error_sentman = getElement(obj.simulation_outputs(1).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;
            error_new = getElement(obj.simulation_outputs(2).logsout,"LogGncAlgorithms").Values.error_quaternion_RB;

            [~, angles_shuttle_sentman] = smu.unitQuat.rot.toAxisAngle((sign(error_shuttle_sentman.Data(:,1)) .* error_shuttle_sentman.Data)');
            [~, angles_shuttle_new] = smu.unitQuat.rot.toAxisAngle((sign(error_shuttle_new.Data(:,1)) .* error_shuttle_new.Data)');
            [~, angles_sentman] = smu.unitQuat.rot.toAxisAngle((sign(error_sentman.Data(:,1)) .* error_sentman.Data)');
            [~, angles_new] = smu.unitQuat.rot.toAxisAngle((sign(error_new.Data(:,1)) .* error_new.Data)');

            fig = figure;
            plot(error_sentman.Time,angles_shuttle_sentman, 'LineWidth', 2);
            hold on;
            plot(error_new.Time, angles_shuttle_new,'LineWidth', 2);
            plot(error_sentman.Time,angles_sentman, 'LineWidth', 2);
            plot(error_new.Time, angles_new,'LineWidth', 2);
            grid on;
            ylabel('Angle (rad)');
            xlabel('Time (s)');
            legend('shuttle sentman', 'shuttle new', 'sentman', 'new');
            title('Quaternion Error Comparison');
        end
        function verify_settings(obj)
            for i =1:4
                fprintf("Simulation %d:\n", i);
                fprintf("model: %d\n",obj.parameters_cells{i}.Plant.SimplifiedVleoAerodynamics.model);
                fprintf("alpha_e %f\n", obj.parameters_cells{i}.Plant.SimplifiedVleoAerodynamics.bodies{1}.energy_accommodation_coefficients(1));
                fprintf("T_w: %f\n", obj.parameters_cells{i}.Plant.SimplifiedVleoAerodynamics.bodies{1}.temperatures__K(1));
            end
        end
    end
end