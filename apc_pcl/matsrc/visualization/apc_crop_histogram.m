function [crop_histogram] = apc_crop_histogram(image)
    figure, imshow(image)
    [x, y, cropped, rect] = imcrop(image);
    distributions = apc_get_distributions(cropped);
    apc_show_histogram(distributions);
end