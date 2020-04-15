%% obj_func(gains) expects a 1x8 vector and returns a column vector
% The output is the simulation output parameter.

function f = obj_func(gains)
    %f = zeros(size(gains(:,1)));
    mdl = 'PC_Quadcopter_Simulation';
    openmdl = find_system('SearchDepth',0);
    if size(openmdl) == [0 1]
        open_system(mdl);
    end
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('Kp_x',gains(1)); %variables are set in the global workspace
    in = in.setVariable('Kd_x',gains(2));
    in = in.setVariable('Kp_theta',gains(3));
    in = in.setVariable('Ki_theta',gains(4));
    in = in.setVariable('Kd_theta',gains(5));
    in = in.setVariable('Kp_alt',gains(6));
    in = in.setVariable('Ki_alt',gains(7));
    in = in.setVariable('Kd_alt',gains(8));
    % Does fmincon auto-loop this? --- YES I believe so.
    %for n = 1:length(gains(:,1))
    n = 1; % delete if loop is re-implemented
        % Edit simulation params as gains are adjusted by optimizer fmincon
%         Kp_x = gains(n,1); % grab nth row, 1st column value.
%         Kd_x = gains(n,2); % n is the iteration number.
%         Kp_theta = gains(n,3);
%         Ki_theta = gains(n,4);
%         Kd_theta = gains(n,5);
%         Kp_alt = gains(n,6);
%         Ki_alt = gains(n,7);
%         Kd_alt = gains(n,8);

        % Run Simulation
        simOut = sim(in) % Runs simulation using SimulationInput params

        % 2D Trajectory import and data manipulation (time independent)
        %simOut.tout = simOut.tout; % Don't forget time is in the simOut.tout variable produced by the sim.
        theta = simOut.yout(:,5); %pitch information
        theta_cmd = simOut.yout(:,22); %pitch command information
        Thrust = (simOut.yout(:,13)+simOut.yout(:,14)+simOut.yout(:,15)+simOut.yout(:,16))/9000; %Value proportional to thrust. Not actual thrust.
        X = simOut.yout(:,10);
        Y = simOut.yout(:,11);
        Z = simOut.yout(:,12);
        X_cmd = simOut.yout(:,25);
        Y_cmd = simOut.yout(:,26);
        Z_cmd = simOut.yout(:,24);

        index = 1; %variable used to indicate start of humm traj flight

        %%%%% ----NOT STARTING FROM GROUND FOR THIS---- %%%%%
        % if starting from the ground, then time is required to get into position
        % if IC.U == 0
        %     while(simOut.tout(index) < 15) % increment until quadcopter is at beginning of maneuver to start pulling trajectory data.
        %         index = index+1;
        %     end
        % end
        endi = index; %variable used to indicate end of the trajectory flight. To be determined below.

        %loops through data until the X_cmd (a position cmd determined by input
        %trajectory, and therefore will not change at that timestep regardless of
        %the actual position of the quadcopter) is the same as the command 2
        %iterations forward.
        while(simOut.yout(endi,25) ~= simOut.yout((endi+2),25))
            endi = endi+1;
            if(endi>=size(simOut.tout))
                endi = size(simOut.tout);
                break;
            end
        end

        % Error Calculations
        index = index+1; % eliminate index error
        xerr = X(index:endi)-X_cmd((index-1):(endi-1)); %errors in X over entire path
        zerr = Z(index:endi)-Z_cmd((index-1):(endi-1)); %errors in Z over entire path
        err_2D = sqrt(xerr.^2+zerr.^2); %2D distance errors over the entire path
        avgerr_2D = mean(err_2D);
        stderr_2D = std(err_2D); %std deviation of errors
        maxerr_2D = max(err_2D);


        f = avgerr_2D; % function value return (this will try to minimize avgerage error)
    %end
    % should return the full f vector
end % end function

