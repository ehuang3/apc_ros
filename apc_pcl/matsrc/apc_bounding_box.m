function [xywh] = bounding_box(image, object_name, object_list, training_data)
    % Return a boudning box in image containing $object_name
    segmentation = apc_segment(image, object_name, training_data);
    cc = bwconncomp(segmentation);
    stats = regionprops(cc, 'Area', 'BoundingBox');
    [biggest, index] = max([stats.Area]);
    biggest_region = stats(index);
    bounding_box = biggest_region.BoundingBox;

    xywh = bounding_box;
end