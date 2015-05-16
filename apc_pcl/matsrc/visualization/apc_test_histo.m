function [choice] = test_histo(image, sets, times)
    if nargin < 3
        times = 1
    end

    figure, imshow(image)
    for k = 1:times
        [x, y, sub_image, rect] = imcrop();
        distributions = apc_get_distributions(sub_image);
        [choice, distance] = apc_compare_distributions(distributions, sets, 'js')
    end
end