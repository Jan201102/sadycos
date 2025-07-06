classdef openloop < ExampleMission.shuttlecock
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = configureParameters@ExampleMission.shuttlecock();
            parameters_cells{1}.General.States.InitialStates.Plant.RigidBody.velocity_BI_I__m_per_s = [7800; 0;0];
            parameters_cells{1}.General.States.InitialStates.Plant.RigidBody.position_BI_I__m = [7000000; 0 ;0];
            inital_disturbtion = deg2rad(0.1);
            inital_quaternion =  smu.unitQuat.rot.fromAxisAngle([0;1;0],inital_disturbtion); % pitch by 10 degree
            parameters_cells{1}.General.States.InitialStates.Plant.RigidBody.attitude_quaternion_BI = inital_quaternion;
        end
        
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
        function plot_attitude(obj)
            % Extract required data from logPlant
            logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");
            q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
            t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';
            N = size(q_BI,2);

            % Convert quaternions to direction cosine matrices and then to Euler angles
            roll = zeros(N,1);
            pitch = zeros(N,1);
            yaw = zeros(N,1);
            
            for k = 1:N
                % Convert attitude quaternion to DCM
                dcm_BI = smu.unitQuat.att.toDcm(q_BI(:,k));
                % Extract Euler angles from DCM (ZYX sequence: yaw, pitch, roll)
                % Using standard rotation matrix to Euler angle conversion
                pitch(k) = asin(-dcm_BI(1,3));
                yaw(k) = atan2(dcm_BI(1,2), dcm_BI(1,1));
                roll(k) = atan2(dcm_BI(2,3), dcm_BI(3,3));
            end
            
            % Convert to degrees
            roll = rad2deg(roll);
            pitch = rad2deg(pitch);
            yaw = rad2deg(yaw);

            figure('Name','Roll, Pitch, Yaw Angles (Body to Inertial)','NumberTitle','off');
            plot(t, roll, 'r', t, pitch, 'g', t, yaw, 'b');
            xlabel('Time [s]');
            ylabel('Angle [deg]');
            legend('Roll','Pitch','Yaw');
            title('Satellite Attitude: Roll, Pitch, Yaw (Body to Inertial)');
            grid on;
        end

        function compare_pitch(obj)
            % Extract required data from logPlant
            logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");
            q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
            t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';
            N = size(q_BI,2);

            % load analytical data
            [test_folder,~,~] = fileparts(mfilename("fullpath"));
            %[test_folder,~,~] = fileparts(matlab.desktop.editor.getActiveFilename);
            file = string(fullfile(test_folder, 'ol_parameters.mat'));
            analytical_data = load(file);
            angle = obj.parameters_cells{1}.Actuators.control_surfaces_angles__rad(2)
            omega_0 = interp1(analytical_data.control_surface_angles__rad',squeeze(analytical_data.ol_parameters(:,2,1)),angle);
            zeta = interp1(analytical_data.control_surface_angles__rad',squeeze(analytical_data.ol_parameters(:,2,2)),angle);

            % Convert quaternions to direction cosine matrices and then to Euler angles
            pitch = zeros(N,1);
            
            for k = 1:N
                % Convert attitude quaternion to DCM
                dcm_BI = smu.unitQuat.att.toDcm(q_BI(:,k));
                % Extract pitch angle from DCM
                pitch(k) = asin(-dcm_BI(1,3));
            end
            
            % Convert to degrees
            pitch = rad2deg(pitch);
            initial_pitch = pitch(1);
            pitch_analytical = initial_pitch * exp(-zeta * omega_0 * t) .* cos(omega_0 * sqrt(1 - zeta^2) * t);

            figure('Name','Pitch Angle (Body to Inertial)','NumberTitle','off');
            hold on;
            plot(t, pitch, 'g');
            plot(t, pitch_analytical, 'r--');
            legend('Simulated Pitch','Analytical Pitch');
            xlabel('Time [s]');
            ylabel('Pitch Angle [deg]');
            title('Satellite Attitude: Pitch Angle (Body to Inertial)');
            grid on;
        end

        function pitch_freq = eval_pitch_freq(obj)
            % Extract required data from logPlant
            logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");
            q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
            t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';
            N = size(q_BI,2);

            % Convert quaternions to direction cosine matrices and then to Euler angles
            pitch = zeros(N,1);
            
            for k = 1:N
                % Convert attitude quaternion to DCM
                dcm_BI = smu.unitQuat.att.toDcm(q_BI(:,k));
                % Extract pitch angle from DCM
                pitch(k) = asin(-dcm_BI(1,3));
            end
            
            % Convert to degrees
            pitch = rad2deg(pitch);
            
            % Calculate frequency of oscillation
            [pks, locs] = findpeaks(pitch);
            if length(locs) > 1
                period = mean(diff(t(locs)));
                pitch_freq = 1 / period;
            else
                pitch_freq = NaN; % Not enough peaks to determine frequency
            end
            
        end
    end
end