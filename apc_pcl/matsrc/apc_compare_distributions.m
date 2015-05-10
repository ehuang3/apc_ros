function [result] = compare_distributions(distributions, dataset, method)
    % This uses color histograms
    if nargin < 3
        method = 'js';
    end
    minimum = inf;
    result_num = 0;

    for setnum = 1:length(dataset)
        target = dataset{setnum}.distributions;
        divergences = apc_get_divergences(distributions, target, method);

        comparison = max(divergences(:));
        if minimum > comparison
            minimum = comparison;
            result_num = setnum;
        end
    end

    disp(minimum)
    result = dataset{result_num}.name;
end