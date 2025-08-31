classdef openloopSweep < ExampleMission.openloop
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = repmat(configureParameters@ExampleMission.openloop(),8,1);
            % set sentman model for 2nd simulation
            for model_index = 1:2
                for angle_index = 0:3
                    index = (model_index-1)*4 + angle_index+1;
                    parameters_cells{index}.Plant.SimplifiedVleoAerodynamics.model = model_index; 
                    angle = angle_index * pi/6; % angles: 0, pi/6, pi/3, pi/2
                    parameters_cells{index}.Actuators.control_surfaces_angles__rad = [0; angle * ones(4,1)]; % constant angles for control surfaces
                end
            end
        end
        
        function BusesInfo = configureBuses(parameters_cells)

            num_simulations = numel(parameters_cells);
            BusesInfo = repmat(struct('buses_list',{},'BusTemplates',{}), 1, num_simulations);

            for index = 1:num_simulations
                BusesInfo(index) = configureBuses@ExampleMission.openloop(parameters_cells(index));                
            end

        end

        function simulation_inputs = configureSimulationInputs(parameters_cells, BusesInfo)

            num_simulations = numel(parameters_cells);
            simulation_inputs(num_simulations) = Simulink.SimulationInput;

            for index = 1:num_simulations
                simulation_inputs(index) = configureSimulationInputs@ExampleMission.openloop(parameters_cells(index), BusesInfo(index));
            end

        end

    end
    methods (Access = public)

        function [eval_angles,frequencys] = get_pitch_frequencies(obj)
            % get frequency for each simulation
            num_simulations = numel(obj.simulation_outputs);
            frequencys = zeros(num_simulations/2, 2);
            eval_angles = zeros(num_simulations/2, 1);
            for model_index = 1:2
                for angle_index = 0:3
                    index = (model_index-1)*4 + angle_index + 1; % +1 for MATLAB indexing
                    logPlant = getElement(obj.simulation_outputs(index).logsout, "LogPlantDynamics");
                    t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';
                    q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
                    N = size(q_BI,2);

                    % Convert quaternions to direction cosine matrices and then to Euler angles
                    pitch = zeros(N,1);
                    
                    for k = 1:N
                        % Convert attitude quaternion to DCM
                        dcm_BI = smu.unitQuat.att.toDcm(q_BI(:,k));
                        % Extract pitch angle from DCM
                        pitch(k) = asin(-dcm_BI(1,3));
                    end
            
                    % Find peaks in pitch angle
                    [pks, locs] = findpeaks(-pitch); % Pitch is the second element of quaternion
                    if length(pks) > 1
                        period = mean(diff(t(locs)));
                        pitch_freq = 2*pi / period;
                    else
                        pitch_freq = NaN; % Not enough peaks to determine frequency
                    end
                    
                    frequencys(angle_index+1,model_index) = pitch_freq;
                end
            end
            for index = 1:num_simulations/2
                eval_angles(index) = obj.parameters_cells{index}.Actuators.control_surfaces_angles__rad(2);
            end
        end

        function plot_pitch(obj)
            % Plot pitch angle for each simulation
            figure;
            for model_index = 1:2
                for angle_index = 0:3
                    index = (model_index-1)*4 + angle_index + 1; % +1 for MATLAB indexing
                    logPlant = getElement(obj.simulation_outputs(index).logsout, "LogPlantDynamics");
                    t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';
                    q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
                    N = size(q_BI,2);

                    % Convert quaternions to direction cosine matrices and then to Euler angles
                    pitch = zeros(N,1);
                    
                    for k = 1:N
                        % Convert attitude quaternion to DCM
                        dcm_BI = smu.unitQuat.att.toDcm(q_BI(:,k));
                        % Extract pitch angle from DCM
                        pitch(k) = asin(-dcm_BI(1,3));
                    end
            
                    subplot(2, 4, index);
                    plot(t, rad2deg(pitch)); % Convert to degrees for plotting
                    title(sprintf('Model %d, Angle %.2f rad', model_index, angle_index * pi/6));
                    xlabel('Time (s)');
                    ylabel('Pitch Angle (deg)');
                    grid on;
                end
            end
        end

        function compare_pitch_freq(obj)
            % load analytical data
            [test_folder,~,~] = fileparts(mfilename("fullpath"));
            file = string(fullfile(test_folder, 'ol_parameters.mat'));
            analytical_data = load(file);
            omega0_IRS_analytical = analytical_data.ol_parameters(:,2,1);
            omega0_sentman_analytical = analytical_data.ol_parameters(:,1,1);
            angle_analytical = rad2deg(analytical_data.control_surface_angles__rad');

            [angle_sim,frequencys] = obj.get_pitch_frequencies();
            angle_sim = rad2deg(angle_sim); % Convert simulation angles to degrees
            disp(frequencys)
            %plot analyticaldata and add markers for simulation data
            figure('Name','Pitch Frequency Comparison','NumberTitle','off');
            hold on;
            grid on;
            plot(angle_analytical', squeeze(omega0_IRS_analytical), 'r-', 'DisplayName', 'IRS Analytical');
            plot(angle_analytical', squeeze(omega0_sentman_analytical), 'b-', 'DisplayName', 'Sentman Analytical');
            plot(angle_sim, frequencys(:,2), 'ro', 'DisplayName', 'IRS Simulation');
            plot(angle_sim, frequencys(:,1), 'bo', 'DisplayName', 'Sentman Simulation');
            xlabel('Control Surface Angle [°]');
            ylabel('\omega_0 [rad/s]');
            legend('Location', 'southeast');
        end

    end
end