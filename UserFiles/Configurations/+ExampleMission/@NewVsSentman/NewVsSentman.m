classdef NewVsSentman < ExampleMission.newmodel
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = repmat(configureParameters@ExampleMission.newmodel(),2,1);
            % set sentman model for 2nd simulation
            parameters_cells{2}.Plant.SimplifiedVleoAerodynamics.model = 1; 
            parameters_cells{2}.Plant.SimplifiedVleoAerodynamics.LUT_data = [];
        end
        
        function BusesInfo = configureBuses(parameters_cells)

            num_simulations = numel(parameters_cells);
            BusesInfo = repmat(struct('buses_list',{},'BusTemplates',{}), 1, num_simulations);

            for index = 1:num_simulations
                BusesInfo(index) = configureBuses@ExampleMission.newmodel(parameters_cells(index));                
            end

        end

        function simulation_inputs = configureSimulationInputs(parameters_cells, BusesInfo)

            num_simulations = numel(parameters_cells);
            simulation_inputs(num_simulations) = Simulink.SimulationInput;

            for index = 1:num_simulations
                simulation_inputs(index) = configureSimulationInputs@ExampleMission.newmodel(parameters_cells(index), BusesInfo(index));
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