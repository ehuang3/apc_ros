function [distributions] = apc_get_distributions(sub_image)
    ri = double(sub_image(:, :, 1));
    gi = double(sub_image(:, :, 2));
    bi = double(sub_image(:, :, 3));
    r = apc_compute_distribution(ri(:));
    g = apc_compute_distribution(gi(:));
    b = apc_compute_distribution(bi(:));
    distributions = [r; g; b];
end