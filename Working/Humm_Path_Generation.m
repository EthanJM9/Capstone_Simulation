%% Hummingbird Trajectory Path Generation
%  Generates a path structure to be passed into the simulink model based on 
%  the hummingbird trajectory data. Can currently be time-adjusted to allow
%  for slower traveling speeds (focusing only on accuracy of the quad's 
%  traveled path rather than a complete match of (x,z,t) trajectory points)
% 
%  NOTE: Ensure this script is located in the correct directory so uisave
%  is to the correct location, namely 'Model configuration files'
%  
% written by Ethan Marcello based on 'Path_Construction_Demo_Script.m'
% last update 02DEC19

%% Generate path
clear path
load hummTraj;

sp_red = 5; %the speed reduction factor. Multiplied by the original trajectory time

X = [0,hummTraj(1,2),hummTraj(1,2), hummTraj(:,2)']; %double the start point so quad has time to settle before diving.
Y = zeros(1,length(X)); % meters
Z = [-1,hummTraj(1,3),hummTraj(1,3), hummTraj(:,3)'] + 1; %Original trajectory hits origin and don't want quad to hit ground so added 1.
raw_t = hummTraj(:,1)';
t = [0,10,14.9,(((raw_t-raw_t(1)).*sp_red)+15)]; % seconds
Psi = zeros(1,length(X)); % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
path.name = 'Hummingbird Trajectory ' + string(sp_red) + 'x Speed Reduction';
uisave('path','../Model configuration files/humm_traj_path_slow' + string(sp_red) + 'x');
clear X Y Z t Psi sp_red raw_t hummTraj