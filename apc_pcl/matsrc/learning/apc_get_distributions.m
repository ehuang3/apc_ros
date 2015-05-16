function [distributions] = apc_get_distributions(sub_image, range)
    if nargin < 2
        range = 1:255;
    end

    ri = double(sub_image(:, :, 1));
    gi = double(sub_image(:, :, 2));
    bi = double(sub_image(:, :, 3));
    r = apc_compute_distribution(ri(:), range);
    g = apc_compute_distribution(gi(:), range);
    b = apc_compute_distribution(bi(:), range);
    distributions = [r; g; b];
end