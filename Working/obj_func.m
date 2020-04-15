%% obj_func(gains) expects a 1x8 vector and returns a column vector
% The output is the simulation output parameter.

function f = obj_func(gains)
    f = zeros(size(gains(:,1)));
    
    for n = 1:length(gains(:,1))
        % Edit simulation params as gains are adjusted by optimizer fmincon
        Kp_x = gains(n,1); % grab nth row, 1st column value.
        Kd_x = gains(n,2); % n is the iteration number.
        Kp_theta = gains(n,3);
        Ki_theta = gains(n,4);
        Kd_theta = gains(n,5);
        Kp_alt = gains(n,6);
        Ki_alt = gains(n,7);
        Kd_alt = gains(n,8);

        % Run Simulation
        sim('PC_Quadcopter_Simulation.slx', 20) % Runs simulation for 20 seconds

        % 2D Trajectory import and data manipulation (time independent)
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

        %%%%% ----NOT STARTING FROM GROUND FOR THIS---- %%%%%
        % if starting from the ground, then time is required to get into position
        % if IC.U == 0
        %     while(tout(index) < 15) % increment until quadcopter is at beginning of maneuver to start pulling trajectory data.
        %         index = index+1;
        %     end
        % end
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

        % Error Calculations

        xerr = X(index:endi)-X_cmd((index-1):(endi-1)); %errors in X over entire path
        zerr = Z(index:endi)-Z_cmd((index-1):(endi-1)); %errors in Z over entire path
        err_2D = sqrt(xerr.^2+zerr.^2); %2D distance errors over the entire path
        avgerr_2D = mean(err_2D);
        stderr_2D = std(err_2D); %std deviation of errors
        maxerr_2D = max(err_2D);


        f(n) = avgerr_2D; % function value return (this will try to minimize avgerage error)
    end
    % should return the full f vector
end % end function

