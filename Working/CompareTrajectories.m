%% 2D Trajectory Comparison & Analysis
% 
%  Takes in 2D trajectory data (in the X and Z coordinate frame) from the
%  simout variable 'yout' for the PC_Quadcopter_Simulation.slx and compares
%  it to the commanded trajectory positions. 
%
%  -----------------USE THIS SCRIPT TO RUN THE SIMULATION------------------
%  
%  See "Setup and run simulation" section below and edit the
%  necessary parameters/load files to run the path you want. Remember to
%  edit both the IC (uncomment the line you want) and the trajectory path 
%  filename.
%
%  TODO: -Determines if trajectory was satisfactory.
%        -Incoroprates time sensitive calculations (already does)
%        -??
%  
% written by Ethan Marcello
% last updated 05APR2020

%% Setup simulation
%Note: This script is located in the "Working" folder, and accesses the
% simulation parameters from the folder "Capstone_Simulation\Model_configuration_files". 
% If location of parameter files is different, must change the load command 
% to reflect the correct path to these files.

%clear; % clean up workspace

% MUST UNCOMMENT CORRECT IC FOR THE PATH FLYING
% load('..\Model_configuration_files\IC.mat'); % load the initial conditions for the quadcopter
  load('..\Model_configuration_files\IChumm.mat'); % load the initial conditions for a starting-from-flight simulation.

load('..\Model_configuration_files\crazyflie(1)Model_X.mat'); % load the quadcopter model

% MUST EDIT THIS FOR EVERY RUN WHEN CHANGING THE TRAJECTORY PATH!!!
trajectory_path = '..\Model_configuration_files\IChumm_humm_traj_path_slow10x.mat';
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

% These parameters effect X position control
Kp_x = 6; % original value 0.32
Kd_x = 0.1; % original value 0.1
att_lim = 45; % in degrees here, multiplied by pi/180 for conversion to radians in sim

% These parameters effect theta control
Kp_theta = 6; % original value 2
Ki_theta = 1.1; % original value 1.1
Kd_theta = 3; % original value 1.2
%These paramaters effect altitude control (Z)
Kp_alt = 6; % original value 2
Ki_alt = 1.1; % original value 1.1
Kd_alt = 3.3; % original value 3.3


%% Run Simulation

sim('PC_Quadcopter_Simulation.slx', sim_time) % Runs simulation for sim_time seconds

%% Important display parameters:
ss = 3; %data display step size (larger number will display less data points)

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

load('hummTraj.mat'); %load in true hummingbird trajectory data (if needed for testing, but commanded path should be nearly the same)

fig = figure(1);
clf;
%fig.OuterPosition = [100 100 650 650]; %can adjust where figure shows up if desired
ax = axes;
ax.FontSize=12;
%ax.XTick = -10:2:7;
index = 1; %variable used to indicate start of humm traj flight

% if starting from the ground, then time is required to get into position
if IC.U == 0
    while(tout(index) < 15) % increment until quadcopter is at beginning of maneuver to start pulling trajectory data.
        index = index+1;
    end
end
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

hold on;
% The commanded position at each timestep should be identical to the actual
% position in a perfect-control scenario.
plot(X(index:ss:endi),Z(index:ss:endi),'bo','LineWidth',1.5); %state data X,Z
plot(X_cmd((index):ss:(endi)),Z_cmd((index):ss:(endi)),'r+','LineWidth',1.5); %commands data X,Z
%plot(hummTraj(:,2),hummTraj(:,3)+1,'c*','LineWidth',1.5); %actual hummingbird data (adds one to the Z coordinate to match sim height)
ax.DataAspectRatio = [1 1 1]; %equalizes scale on xy axis
grid on;

%Add thrust vectors using "quiver" function
%trim data
numVectors = 5;
splice = round(linspace(index,endi,numVectors)); %evenly spaces velocity vects throught
theta1 = theta(splice);
Thrust1 = Thrust(splice);
for numVectors = 1:5
    a = Thrust1(numVectors)*cos(theta1(numVectors));
    b = Thrust1(numVectors)*sin(theta1(numVectors));
    quiver(X(splice(numVectors)),Z(splice(numVectors)),b,a,0,'g','LineWidth',2); %displays vector
end

tit = title(path.name,'FontSize',20);
%NOTE: Thrust vectors are only proportional to thrust, and they are evenly
%spaced throughout the displayed dataset.
leg = legend('Path Flown','Path Commanded','Thrust Vector','FontSize',16);
xl = xlabel('X (m)','FontSize',16);
yl = ylabel('Z (m)','FontSize',16);

%% Error Calculations

xerr = X(index:endi)-X_cmd((index):(endi)); %errors in X over entire path
zerr = Z(index:endi)-Z_cmd((index):(endi)); %errors in Z over entire path
err_2D = sqrt(xerr.^2+zerr.^2); %2D distance errors over the entire path
avgerr_2D = mean(err_2D);
stderr_2D = std(err_2D); %std deviation of errors
maxerr_2D = max(err_2D);
% Set up data structure for saving
data = [xerr; zerr; err_2D];
%save("Error_Figures/ErrorData_"+extractAfter(trajectory_path,"slow"), "data", "avgerr_2D", "stderr_2D", "maxerr_2D");

%% Plot Error Graph(s)

traj_time = tout(index:endi); %makes a variable of same size as error vectors for plotting.
traj_time = traj_time - traj_time(1); % subtract the start time so time starts from zero.

fig2 = figure(2);
clf;
% ax2 = axes(fig2,'DataAspectRatio',[1 1 1]); % equalizes on the xy axis
hold on;
plot(traj_time, xerr, 'LineWidth', 1.5);
plot(traj_time, zerr, 'LineWidth', 1.5);
plot(traj_time, err_2D, 'LineWidth', 1.5);
title('Position Errors at' + extractAfter(path.name,'ory'), 'FontSize', 20);
legend('Error in X', 'Error in Z', 'Total 2D Distance Error','FontSize',12);
tl = xlabel('Time (s)','FontSize',16);
zl = ylabel('Error (m)','FontSize',16);
grid on;

fig3 = figure(3);
clf;
% ax3 = axes(fig3,'DataAspectRatio',[1 1 1]); % equalizes on the xy axis
hold on;
plot(traj_time, theta(index:endi));
plot(traj_time, theta_cmd(index:endi));
title('Theta Errors at' + extractAfter(path.name,'ory'), 'FontSize', 20);
legend('Theta','Theta Cmd', 'FontSize',12);
tl = xlabel('Time (s)','FontSize',16);
zl = ylabel('Value (rad)','FontSize',16);
grid on;

