%% Process Errors
% After running Error_vs_GainChanges.m script, this script determines which
% gains are most significant for improving simulation errors.
%
% Written by Ethan Marcello 26APR2020

%% Import ErrorData

load('USE_ErrorData.mat');

%% Find largest error improvement & improvement direction for each gain.
% This section is for average error.

% function takes in 2 parameters. (Data,error_to_minimize) where
% error_to_minimize must be a value 1:3 corresponding to avg error, std
% error, and max error
[i1, gain_improve1] = findLowestErrors(ErrorData.sr1x,1);
[i5, gain_improve5] = findLowestErrors(ErrorData.sr5x,1);
[i10, gain_improve10] = findLowestErrors(ErrorData.sr10x,1);
[i20, gain_improve20] = findLowestErrors(ErrorData.sr20x,1);

% While the rest of this could be interesting, it really isn't important
% because it just leads to a lot of speculation. Just focus on the
% improvements for average error since this is what I'm trying to minimize.

% uncomment the following lines to re-add in extra data filtering.
% %% improve for std
% % 
% [std_i1, std_gain_improve1] = findLowestErrors(ErrorData.sr1x,2);
% [std_i5, std_gain_improve5] = findLowestErrors(ErrorData.sr5x,2);
% [std_i10, std_gain_improve10] = findLowestErrors(ErrorData.sr10x,2);
% [std_i20, std_gain_improve20] = findLowestErrors(ErrorData.sr20x,2);
% 
% %% improve for max
% 
% [max_i1, max_gain_improve1] = findLowestErrors(ErrorData.sr1x,3);
% [max_i5, max_gain_improve5] = findLowestErrors(ErrorData.sr5x,3);
% [max_i10, max_gain_improve10] = findLowestErrors(ErrorData.sr10x,3);
% [max_i20, max_gain_improve20] = findLowestErrors(ErrorData.sr20x,3);
% 
% %% correlations between errors
% 
% corr_1x = [gain_improve1; std_gain_improve1; max_gain_improve1];
% corr_5x = [gain_improve5; std_gain_improve5; max_gain_improve5];
% corr_10x = [gain_improve10; std_gain_improve10; max_gain_improve10];
% corr_20x = [gain_improve20; std_gain_improve20; max_gain_improve20];
