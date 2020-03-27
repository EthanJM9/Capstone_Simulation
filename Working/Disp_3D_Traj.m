%% Display 3D OptiTrack Trajectories
% Code to display x,y,z data of the drone

%% load dataset
clear
load Cf11_15NOV19_1;

%% data manipulation

x = x./1000; %conversions to meters from mm
y = y./1000;
z = z./1000;

%% plot it

figure(1);
clf;
ax = axes;
ax.FontSize=14;

pl = plot3(x,y,z,'LineWidth',1.5);
pl.Color = 'b';
tit = title('3D Crazyflie Trajectory');
tit.FontSize = 20;
leg = legend('Path Flown');
xl = xlabel('X (m)');
yl = ylabel('Y (m)');
zl = zlabel('Z (m)');