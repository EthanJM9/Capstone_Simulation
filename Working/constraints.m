function [c,ceq] = constraints(gains)
% This function takes in the control gains for the simulation and defines
% the optimization constraints associated with them (values must be greater than 0)
    c = [-gains(1) -gains(2) -gains(3) -gains(4) -gains(5) -gains(6) -gains(7) -gains(8)]; %inequality constraint c represents <=0
    ceq = [ ]; % equality constraint
end

