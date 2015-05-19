function [sets] = auto_train(r_sets, existing_sets)
% Function for automatically training the color-histogram matching tool
% Run through the r_sets and generate distributions, appending them to sets

    for k = 1:length(r_sets)
        r_sets{k}.distributions = apc_get_distributions(r_sets{k}.image);
    end

    if nargin == 2
        sets = [existing_sets, r_sets];
    else
        sets = r_sets;
    end
end