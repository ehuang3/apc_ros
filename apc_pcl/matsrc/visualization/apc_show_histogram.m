function [histogram] = apc_histogram(distributions, distributions_2)
    x = 1:length(distributions);
    figure
    hold on
    plot(x, distributions(1, :)', 'r-');
    plot(x, distributions(2, :)', 'g-');
    plot(x, distributions(3, :)', 'b-');
    if nargin == 2
        plot(x, distributions_2(1, :)', 'r.');
        plot(x, distributions_2(2, :)', 'g.');
        plot(x, distributions_2(3, :)', 'b.');
    end
    hold off
end