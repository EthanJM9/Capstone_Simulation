%% Script to test differing errors with different kp and kd gains

avgerrs = [];

for n=1:1:50
    Kp_theta = n;
    Kd_theta = 0.1;
    att_lim = 45; % in degrees here, multiplied by pi/180 for conversion to radians in sim
    CompareTrajectories
    avgerrs = [avgerrs, avgerr_2D];
end

