function [image_sets] = record_image(image, object_name, existing_sets)
    new.name = object_name;
    new.image = image;
    image_sets = [existing_sets, new]
end