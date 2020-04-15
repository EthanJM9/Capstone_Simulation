%% tests the optimization demo
% Constrained, nonlinear, solver-based solution

% setting optimization parameters
options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','interior-point');

% reports x (vector containing [x1 x2]), the value returned by function
[x,fval] = fmincon(@rosenbrock,[0 0],...  % notice rosenbrock is the name of the objective function
    [],[],[],[],[],[],@unitdisk,options)  % unitdisk is the name of the constraints function
                                          % options is the optimization parameter structure

% After running,  the output display table will show details related to the
% optimization process.