function [c,ceq] = unitdisk(x)
    c = x(1)^2 + x(2)^2 - 1; %inequality constraint c represents <=0
    ceq = [ ]; % equality constraint
end

