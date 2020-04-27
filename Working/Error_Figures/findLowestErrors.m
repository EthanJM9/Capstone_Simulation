function [sm_avg_i, improve_dir_gain] = findLowestErrors(ErrorData, error_to_minimize)
%findLowestErrors finds the lowest error values in ErrorData for the error_to_minimize parameter.
%   The function returns the index of the lowest error (best improvement)
%   and a vector of values corresponding to which modification (if any)
%   should be made to each respective gain to help lower the errors.
    e = error_to_minimize; % shorthand to reduce clutter
    improve_dir_gain = zeros(1,8);

    %for speed reduction 1x
    sm_avg = 1000;
    sm_avg_i = 1;
    for i=1:24
        if ErrorData(e,i) <= sm_avg
            sm_avg = ErrorData(e,i);
            sm_avg_i = i; 
        end
    end
    % order is decrease, base, then increase
    count = 1;
    for i=1:3:24
        [~,I] = min(ErrorData(e,i:(i+2)));
        improve_dir_gain(1,count) = I-2; % will be -1 if decrease is better, 1 if increase is better, and 0 if no change is better.
        count = count + 1;
    end
end

