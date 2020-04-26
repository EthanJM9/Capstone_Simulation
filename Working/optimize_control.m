%% Runs the optimization on the control gains for the simulation

%% Setup simulation
%Note: This script is located in the "Working" folder, and accesses the
% simulation parameters from the folder "Capstone_Simulation\Model_configuration_files". 
% If location of parameter files is different, must change the load command 
% to reflect the correct path to these files.

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

% attitude limiter, in degrees
att_lim = 45;

%% Prepare Optimization
% define initial gains vector
% 0.32, 0.1, 2.0, 1.1, 1.2,
gains0 = [6.0, 1.1, 3.3]; %Kp_x, Kd_x, Kp_theta, Ki_theta, Kd_theta, Kp_alt, Ki_alt, Kd_alt

% These parameters effect X position control
Kp_x = 6; % original value 0.32
Kd_x = 0.1; % original value 0.1
% These parameters effect theta control
Kp_theta = 6; % original value 2
Ki_theta = 1.1; % original value 1.1
Kd_theta = 3; % original value 1.2

lb = zeros(1,3);
ub = ones(1,3).*35; %upper bound at gains of 35
     
%%
% setting optimization parameters
options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','sqp','DiffMinChange',0.1);

% reports gains (vector containing control gains), and the value returned by function
[gains,fval] = fmincon(@obj_func,gains0,...  % objective function input, and initial point
              [],[],[],[],lb,ub,[],options)  % lb is lower bound for gain vars.
                                          % options is the optimization parameter structure

% After running,  the output display table will show details related to the
% optimization process.