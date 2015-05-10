function [choice] = test_histo(image, sets)
    figure, imshow(image)
    [x, y, sub_image, rect] = imcrop();

    distributions = apc_get_distributions(sub_image);
    choice = apc_compare_distributions(distributions, sets)
end