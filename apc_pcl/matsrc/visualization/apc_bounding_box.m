function [xywh_list, success] = bounding_box(image, object_name, object_list, training_data, visualize)
    % Return a boudning box in image containing $object_name
    if nargin < 5
        visualize = false;
    end
    object_count = sum(strcmp(object_list, object_name))
    assert(object_count > 0, 'object_count must be greater than zero')

    [segmentation, acceptability] = apc_segment(image, object_name, training_data, 0);
    
    if acceptability < 0.5
        xywh_list = [];
        success = false;
    end
    cc = bwconncomp(segmentation);

    xywh_list = [];

    stats = regionprops(cc, 'Area', 'BoundingBox');
    for k = 1:object_count        
        [biggest, index] = max([stats.Area]);
        biggest_region = stats(index);
        % Delete this entry
        stats(index) = [];
        bounding_box = biggest_region.BoundingBox;
        xywh_list = [xywh_list; bounding_box]
    % xywh = bounding_box;
end