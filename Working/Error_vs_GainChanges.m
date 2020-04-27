%% 2D Trajectory Error Deltas in Response to Changes in Control Gains
% 
%  Iteratively runs changes in control gains over the simulation and
%  determines changes in error values to predict direction of better
%  control solution.
%
%  --Ensure all necessary files are in the appropriate folder.
%  --This script runs many unnecessary iterations of the simulation (it
%    runs the errors for when no gains are changed 8 times, one for each
%    gain). You can increase the speed threefold by auto-filling ErrorData
%    and skipping over these extra simulation runs (but you have to
%    implement the code for it, as it isn't implemented here).
%  
% written by Ethan Marcello
% last updated 26APR2020


%% Create outer loop to iterate over each path speed
for speed_red = [1 5 10 20]
    %% Setup simulation
    %Note: This script is located in the "Working" folder, and accesses the
    % simulation parameters from the folder "Capstone_Simulation\Model_configuration_files". 
    % If location of parameter files is different, must change the load command 
    % to reflect the correct path to these files.

    load('..\Model_configuration_files\IChumm.mat'); % load the initial conditions for a starting-from-flight simulation.

    load('..\Model_configuration_files\crazyflie(1)Model_X.mat'); % load the quadcopter model

    % MUST EDIT THIS FOR EVERY RUN WHEN CHANGING THE TRAJECTORY PATH!!!
    trajectory_path = '..\Model_configuration_files\IChumm_humm_traj_path_slow'+string(speed_red)+'x.mat';
    load(trajectory_path); % load the trajectory to fly, variable in ws is a timeseries "path"

    % Adjusting IC velocity based on speed of trajectory that is req'd.
    if (extractBetween(trajectory_path,'files\','_humm_') == "IChumm")
        path_speed_str = extractBetween(trajectory_path, 'slow', 'x.mat'); % extracting path speed from trajectory file name
        path_speed = str2double(path_speed_str); % converting the path speed to an integer.
        IC.U = IC.U/path_speed; % adjusting initial velocities as appropriate for path speed
        IC.W = IC.W/path_speed; 
        sim_time = 20; % shortening sim time length
    else
        sim_time = 45;
    end

    %% Control Parameters
    % I begin by creating a loop to iterate over all of the gain alterations. 
    for iter=1:8
        for testi=1:3
            switch testi
                case 1
                    mult_fact = 0.95;
                case 2
                    mult_fact = 1.0;
                case 3
                    mult_fact = 1.05;
            end
            
            if speed_red ~= 1
                gains = [6, 0.1, 6, 1.1, 3, 6, 1.1, 3.3]; %baseline gains to be used
            else
                gains = [0.32, 0.1, 2, 1.1, 1.2, 2, 1.1, 3.3]; %baseline gains to be used
            end

            gains(iter) = gains(iter)*mult_fact; % 5 percent change to gain.

            % These parameters effect X position control
            Kp_x = gains(1); % original value 0.32
            Kd_x = gains(2); % original value 0.1
            % These parameters effect theta control
            Kp_theta = gains(3); % original value 2
            Ki_theta = gains(4); % original value 1.1
            Kd_theta = gains(5); % original value 1.2
            %These paramaters effect altitude control (Z)
            Kp_alt = gains(6); % original value 2
            Ki_alt = gains(7); % original value 1.1
            Kd_alt = gains(8); % original value 3.3

            att_lim = 45; % in degrees here, multiplied by pi/180 for conversion to radians in sim

            %% Run Simulation

            sim('PC_Quadcopter_Simulation.slx', sim_time) % Runs simulation for sim_time seconds

            %% 2D Trajectory import and data manipulation (time independent)

            %tout = tout; % Don't forget time is in the tout variable produced by the sim.
            theta = yout(:,5); %pitch information
            theta_cmd = yout(:,22); %pitch command information
            Thrust = (yout(:,13)+yout(:,14)+yout(:,15)+yout(:,16))/9000; %Value proportional to thrust. Not actual thrust.
            X = yout(:,10);
            Y = yout(:,11);
            Z = yout(:,12);
            X_cmd = yout(:,25);
            Y_cmd = yout(:,26);
            Z_cmd = yout(:,24);


            index = 1; %variable used to indicate start of humm traj flight

            endi = index; %variable used to indicate end of the trajectory flight. To be determined below.

            %loops through data until the X_cmd (a position cmd determined by input
            %trajectory, and therefore will not change at that timestep regardless of
            %the actual position of the quadcopter) is the same as the command 2
            %iterations forward.
            while(yout(endi,25) ~= yout((endi+2),25))
                endi = endi+1;
                if(endi>=size(tout))
                    endi = size(tout);
                    break;
                end
            end

            %% Error Calculations

            xerr = X(index:endi)-X_cmd((index):(endi)); %errors in X over entire path
            zerr = Z(index:endi)-Z_cmd((index):(endi)); %errors in Z over entire path
            err_2D = sqrt(xerr.^2+zerr.^2); %2D distance errors over the entire path
            avgerr_2D = mean(err_2D);
            stderr_2D = std(err_2D); %std deviation of errors
            maxerr_2D = max(err_2D);
            
            % Set up data structure for saving
            if testi==1 && iter==1
                error_data = [avgerr_2D; stderr_2D; maxerr_2D];
            else
                error_data = [error_data [avgerr_2D; stderr_2D; maxerr_2D]];
            end
            
            %save("Error_Figures/ErrorData_"+extractAfter(trajectory_path,"slow"), "data", "avgerr_2D", "stderr_2D", "maxerr_2D");
        end % captures internal for (testi 1:3) where factor is at gain, 5% below, and 5% above respectively.

    end % captures for (iter 1:8)
    
    switch speed_red
        case 1
            ErrorData.sr1x = error_data;
            fprintf("First trajectory iteration complete\n");
        case 5
            ErrorData.sr5x = error_data;
            fprintf("Second trajectory iteration complete\n");
        case 10
            ErrorData.sr10x = error_data; %sr is "speed reduction"
            fprintf("Third trajectory iteration complete\n");
        case 20
            ErrorData.sr20x = error_data;
            fprintf("Last trajectory iteration complete\nError data saved in structure 'ErrorData'\n");
    end %end of switch
    
end % For each trajectory path 1,5,10,20

ErrorData.readme = "Rows are avg, std, and max error values. Each column is one trial. Each 3 columns is one gain manipulation in the order of 5% decrease, base, 5% increase";
save('Error_Figures/ErrorData.mat', 'ErrorData');

