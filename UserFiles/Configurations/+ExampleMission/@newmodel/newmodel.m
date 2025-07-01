classdef newmodel < ExampleMission.DefaultConfiguration
    methods (Static)
        function parameters_cells = configureParameters()
            parameters_cells = configureParameters@ExampleMission.DefaultConfiguration();
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.model = 2;
            [this_folder,~,~] = fileparts(mfilename("fullpath"));
            lut_path = string(fullfile(this_folder, 'aerodynamic_coefficients_panel_method.csv'));
            LUT_data = readmatrix(lut_path);
            LUT_data = griddedInterpolant(LUT_data(:,1),LUT_data(:,2:5));
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.LUT_data = LUT_data;

            % PARAMETERS FROM DSMC
            n = 4.698e14;
            temperature__K = 934;
            surface_temperature__K = 300;
            particles_mass__kg = 16 * 1.6605390689252e-27;
            density__kg_per_m3 = particles_mass__kg * n;
            orbital_velocity__m_per_s = 7800;
            energy_accommodation = (7.5e-17 * n * temperature__K)/(1+7.5e-17 * n * temperature__K);
            %END PARAMETERS FROM DSMC
            gravitational_parameter_Earth__m3_per_s2 = 3.986004e14;

            %set correct parameters for the geometric model
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies{1}.energy_accommodation_coefficients = energy_accommodation*ones(1,12);
            parameters_cells{1}.Plant.SimplifiedVleoAerodynamics.bodies{1}.temperatures__K = surface_temperature__K*ones(1,12);

            %add correct atmospheric parameters
            parameters_cells{1}.Environment.atmospheric_mass_density__kg_per_m3 = density__kg_per_m3;
            parameters_cells{1}.Environment.atmospheric_number_density__1_per_m3 = n;
            parameters_cells{1}.Environment.atmospheric_temperature__K = temperature__K;

            %set correct orbital velocity
            parameters_cells{1}.General.States.InitialStates.Plant.RigidBody.velocity_BI_I__m_per_s = [ orbital_velocity__m_per_s ; 0 ; 0 ];
            parameters_cells{1}.General.States.InitialStates.Plant.RigidBody.position_BI_I__m = [0 ; 0; -gravitational_parameter_Earth__m3_per_s2/(orbital_velocity__m_per_s^2)];

            %reduce sim time for faster verification
            stop_time_setting = parameters_cells{1}.Settings(1); % StopTime is the first setting
            stop_time_setting.value = "100"; % Change from default 1000s to 500s
            parameters_cells{1}.Settings(1) = stop_time_setting;

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

    end
    methods
        function plot_torques_and_forces(obj)
            % Assuming you have:
            logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");

            % Extract forces and torques from logPlant
            forces = logPlant.Values.Forces;
            torques = logPlant.Values.Torques;
            t_force = forces.aerodynamic_force_B__N.Time;
            t_torque = torques.net_torque_B__N_m.Time;
            labels = {'roll','pitch','yaw'};
            force_labels = {'x','y','z'};

            figure('Name','Torques and Aerodynamic Force (B-frame)','NumberTitle','off');

            subplot(3,2,1)
            plot(t_torque, torques.net_torque_B__N_m.Data)
            xlabel('Time [s]'); ylabel('Torque [Nm]');
            title('Net Torque (B-frame)');
            legend(labels);

            subplot(3,2,2)
            plot(t_torque, torques.aerodynamic_torque_B__Nm.Data)
            xlabel('Time [s]'); ylabel('Torque [Nm]');
            title('Aerodynamic Torque (B-frame)');
            legend(labels);

            subplot(3,2,3)
            plot(t_torque, torques.magnetic_torque_B__N_m.Data)
            xlabel('Time [s]'); ylabel('Torque [Nm]');
            title('Magnetic Torque (B-frame)');
            legend(labels);

            subplot(3,2,4)
            plot(t_torque, torques.reaction_torque_B__N_m.Data)
            xlabel('Time [s]'); ylabel('Torque [Nm]');
            title('Reaction Torque (B-frame)');
            legend(labels);

            subplot(3,2,5)
            plot(t_torque, torques.gyroscopic_torque_B__N_m.Data)
            xlabel('Time [s]'); ylabel('Torque [Nm]');
            title('Gyroscopic Torque (B-frame)');
            legend(labels);

            subplot(3,2,6)
            plot(t_force, forces.aerodynamic_force_B__N.Data)
            xlabel('Time [s]'); ylabel('Force [N]');
            title('Aerodynamic Force (B-frame)');
            legend(force_labels);

            sgtitle('Torques and Aerodynamic Force Acting on Satellite (Body Frame)');
        end
        function plot_attitude(obj)
            % Extract required data from logPlant
            logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");
            q_BI = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Data'; % 4xN [w x y z]
            pos_I = logPlant.Values.PlantStates.RigidBody.position_BI_I__m.Data';      % 3xN
            vel_I = logPlant.Values.PlantStates.RigidBody.velocity_BI_I__m_per_s.Data';% 3xN
            t = logPlant.Values.PlantStates.RigidBody.attitude_quaternion_BI.Time';

            N = size(q_BI,2);
            q_BT = zeros(4,N); % Preallocate as 4xN for smu functions

            for k = 1:N
                % Compute Tangentail wrt. Inertial quaternion at each time step
                q_TI = smu.frames.rotationInertialToTangential(pos_I(:,k), vel_I(:,k)); % 4x1 column vector
                
                % Get Inertial wrt. Tangential
                q_IT = smu.unitQuat.invert(q_TI); 
                
                % Body wrt. tangential
                q_BT(:,k) = smu.unitQuat.att.composition(q_BI(:,k), q_IT);
            end

            % Convert quaternions to direction cosine matrices and then to Euler angles
            roll = zeros(N,1);
            pitch = zeros(N,1);
            yaw = zeros(N,1);
            
            for k = 1:N
                % Convert attitude quaternion to DCM
                dcm_BT = smu.unitQuat.att.toDcm(q_BT(:,k));
                % Extract Euler angles from DCM (ZYX sequence: yaw, pitch, roll)
                % Using standard rotation matrix to Euler angle conversion
                pitch(k) = asin(-dcm_BT(1,3));
                yaw(k) = atan2(dcm_BT(1,2), dcm_BT(1,1));
                roll(k) = atan2(dcm_BT(2,3), dcm_BT(3,3));
            end
            
            % Convert to degrees
            roll = rad2deg(roll);
            pitch = rad2deg(pitch);
            yaw = rad2deg(yaw);

            figure('Name','Roll, Pitch, Yaw Angles (Body to Tangential)','NumberTitle','off');
            plot(t, roll, 'r', t, pitch, 'g', t, yaw, 'b');
            xlabel('Time [s]');
            ylabel('Angle [deg]');
            legend('Roll','Pitch','Yaw');
            title('Satellite Attitude: Roll, Pitch, Yaw (Body to Tangential)');
            grid on;
        end

    function  plot_angular_acceleration(obj)
    % Extract required data from logPlant
    logPlant = getElement(obj.simulation_outputs.logsout, "LogPlantDynamics");
    
    % Extract angular acceleration data
    angular_accel = logPlant.Values.PlantFeedthrough.RigidBodyAccelerations.rotational_acceleration_BI_B__rad_per_s2.Data';
    t = logPlant.Values.PlantFeedthrough.RigidBodyAccelerations.rotational_acceleration_BI_B__rad_per_s2.Time';
    
    % Also extract angular velocity for comparison
    angular_vel = logPlant.Values.PlantStates.RigidBody.angular_velocity_BI_B__rad_per_s.Data';
    t_vel = logPlant.Values.PlantStates.RigidBody.angular_velocity_BI_B__rad_per_s.Time';
    disp(angular_vel);
    % Labels for axes
    labels = {'Roll (x)', 'Pitch (y)', 'Yaw (z)'};
    colors = {'r', 'g', 'b'};
    
    figure('Name','Angular Velocity and Angular Acceleration (Body Frame)','NumberTitle','off');
    
    % Top row: Angular acceleration for each axis
    for i = 1:3
        subplot(2,3,i)
        plot(t, angular_accel(i,:), colors{i}, 'LineWidth', 1.5);
        xlabel('Time [s]'); 
        ylabel('Angular Acceleration [rad/s²]');
        title(['Angular Acceleration - ' labels{i}]);
        grid on;
    end
    
    % Bottom row: Angular velocity for each axis
    for i = 1:3
        subplot(2,3,i+3)
        plot(t_vel, angular_vel(i,:), colors{i}, 'LineWidth', 1.5);
        xlabel('Time [s]'); 
        ylabel('Angular Velocity [rad/s]');
        title(['Angular Velocity - ' labels{i}]);
        grid on;
    end
    
    sgtitle('Satellite Angular Motion (Body Frame)');
end
    end
end