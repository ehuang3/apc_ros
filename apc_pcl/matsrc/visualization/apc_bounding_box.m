function [xywh_list] = bounding_box(image, object_name, object_list, training_data, visualize)
    % Return a boudning box in image containing $object_name
    if nargin < 5
        visualize = false;
    end
    object_count = sum(strcmp(object_list, object_name))
    assert(object_count > 0, 'object_count must be greater than zero')

    segmentation = apc_segment(image, object_name, training_data, visualize);
    cc = bwconncomp(segmentation);

    xywh_list = [];

    if visualize
        figure, imshow(image)
        title(object_name)
    end
    
    stats = regionprops(cc, 'Area', 'BoundingBox');
    for k = 1:object_count

        figure
        title(['hit iteration' k])
        pause(3)
        
        [biggest, index] = max([stats.Area]);
        biggest_region = stats(index);
        % Delete this entry
        stats(index) = [];
        bounding_box = biggest_region.BoundingBox;
        xywh_list = [xywh_list; bounding_box]

        if visualize
            rectangle('Position', bounding_box)
        end
    % xywh = bounding_box;
    figure,
    title('We completed the operation, then what?')
end