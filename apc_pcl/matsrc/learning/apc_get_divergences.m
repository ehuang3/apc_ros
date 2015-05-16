function [divergences] = get_divergences(distributions_1, distributions_2, method)
    if nargin < 3
        method = 'js';
    end
    divergences = zeros(size(distributions_1, 1));
    for k = 1:size(distributions_1, 1)
        divergences(k) = apc_divergence(distributions_1(1, :), distributions_2(1, :), method);
    end
end