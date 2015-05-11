function [segmented] = apc_segment(image, target_object, sets, show)
    % Run the segmentation algorithm on an image and get the results
    if nargin < 4
        show = false;
    end

    % CDF and JS both work well, but in different areas....sum their segmentation results?
    comparison_func = @(block) compare_to(block.data, sets, target_object);
    compared_1 = blockproc(image, [25, 25], comparison_func);
    compared_2 = blockproc(image, [50, 50], comparison_func);
    compared_3 = blockproc(image, [100, 100], comparison_func);

    compared = compared_1 + compared_2 + compared_3;

    acceptability_thresh = prctile(compared(:), 99)  % 99th percentile
    segmented = compared > acceptability_thresh;

    if show
        figure,imagesc(compared)
        title('Heat-map')
        rgb_compared = repmat(compared, [1, 1, 3]) / max(compared(:));
        figure, imshow(uint8(double(image) .* rgb_compared));
        title('Highlighted')
        segmask = repmat(segmented, [1, 1, 3]);
        figure, imshow(uint8(double(image) .* double(segmask)))
    end
end

function [B] = compare_to(image, sets, target_object)
    distributions = apc_get_distributions(image);
    [name, distance] = apc_compare_distributions(distributions, sets, 'js');
    sz = size(image);
    if strcmp(name, target_object)
        % B = true(sz(1:2));
        B = ones(sz(1:2)) / distance;
    else
        B = zeros(sz(1:2));
    end
end