function [divergences] = get_divergences(distributions_1, distributions_2, method)
    % get_divergences(distributions_1, distributions_2, method)
    % Compute the divergences between distributions 1 and 2 using $method
    if nargin < 3
        method = 'js';
    end
    divergences = zeros(size(distributions_1, 1), 1);
    for k = 1:size(distributions_1, 1)
        divergences(k) = apc_divergence(distributions_1(k, :), distributions_2(k, :), method);
    end
end