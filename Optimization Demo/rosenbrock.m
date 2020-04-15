%% ROSENBROCK(x) expects a two-column matrix and returns a column vector
% The output is the Rosenbrock function, which has a minimum at
% (1,1) of value 0, and is strictly positive everywhere else.

function f = rosenbrock(x)

f = 100*(x(:,2) - x(:,1).^2).^2 + (1 - x(:,1)).^2;