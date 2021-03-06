%{
 This demonstration script generates a timeseries in the workspace
 After saving the workspace, you can load it later to a position controler 
 as a timeseries of position commands.

Additional quadcopter modeling and simulation materials available at:
github.com/dch33/Quad-Sim

Copyright (C) 2014 D. Hartman, K. Landis, S. Moreno, J. Kim, M. Mehrer

License:
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

%% This demonstration script generates a timeseries in the workspace
%
X = [ 0, 1, 0,-1, 0, 1, 0, 0]; % meters
Y = [ 0, 0, 1, 0,-1, 0, 0, 0]; % meters
Z = [ 3, 3, 3, 3, 3, 3, 3, 1.5]; % meters
t = [ 0, 5,10,15,20,25,30, 35]; % seconds
Psi = [0,90*pi/180,0,180*pi/180,0,0,0,90*pi/180]; % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
clear X Y Z t Psi
uisave('path', 'Path_Diamond')
%clear path
%% This demonstration script generates a much more absurd path.
% Note that it takes a long time to run and each step is five seconds
% Team 37 Plot
X = [0,0,-1,1,4,   1,  4,   1,4,5.5,6.25,4.75,6.25,7,7.25,8.5,9.25,10,10,12,  14, 12,  14,12,15,17, 16,15,0];% meters 28pts
X = X*0.3048;
Y = [0,3, 3,3,3,2.25,1.5,0.75,0,  3, 1.5, 1.5, 1.5,0,   3,   0,  3, 0, 0, 0,0.75,1.5,2.25, 3, 3, 3,1.5, 0,0];
Y = Y*0.3048;
Z = [3];

tXY = 1:10:10*length(X); % seconds

Psi = [0]; % radians
path.x = timeseries(X,tXY);
path.y = timeseries(Y,tXY);
path.z = timeseries(Z,tXY);
path.psi = timeseries(Psi,tXY);
clear X Y Z tXY Psi 
uisave('path', 'Path_Team')
%clear path

%% Hover at 3 m
%
clear path
X = [ 0, 0, 0, 0, 0, 0, 0, 0]; % meters
Y = [ 0, 0, 0, 0, 0, 0, 0, 0]; % meters
Z = [ 1, 1, 1, 1, 1, 1, 1, 0.25]; % meters
t = [ 0, 5,10,15,20,25,30, 35]; % seconds
Psi = [0,90*pi/180,0,180*pi/180,0,0,0,90*pi/180]; % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
clear X Y Z t Psi
uisave('path', 'Path_3mHover')
%clear path
%% experimental
%
clear path
X = linspace(0,3,100); % meters
Y = zeros(1,100); % meters
Z = X.^2; % meters
t = linspace(0,10,100); % seconds
Psi = zeros(1,100); % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
clear X Y Z t Psi

%clear path

%% Hummingbird Trajectory Creation (assumes quadcopter must start from the ground)
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
uisave('path','Model configuration files/humm_traj_path_slow' + string(sp_red) + 'x');
clear X Y Z t Psi sp_red raw_t hummTraj

%% Hummingbird Trajectory Creation (with IC at start of path)
clear path
load ('Working/hummTraj.mat');

sp_red = 20; %the speed reduction factor. Multiplied by the original trajectory time

X = hummTraj(:,2)'; %double the start point so quad has time to settle before diving.
Y = zeros(1,length(X)); % meters
Z = [hummTraj(:,3)'] + 1; %Original trajectory hits origin and don't want quad to hit ground so added 1.
raw_t = hummTraj(:,1)';
t = ((raw_t-raw_t(1)).*sp_red); % seconds
Psi = zeros(1,length(X)); % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);
path.name = 'Hummingbird Trajectory ' + string(sp_red) + 'x Speed Reduction';
uisave('path','Model_configuration_files/IChumm_humm_traj_path_slow' + string(sp_red) + 'x');
clear X Y Z t Psi sp_red raw_t hummTraj


