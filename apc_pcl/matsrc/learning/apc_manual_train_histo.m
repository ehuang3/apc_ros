function [sets] = manual_train(image, object_name, attempts, existing_sets)
    % Open the image, allow the user to manually select a region and name the object
    % Add a more robust way to do this in Python 
    % i.e. a service that knows what is in each bin and quickly cycles images
    if nargin < 3
        attempts = 1;
    end
    sets = cell(1, attempts);

    figure, imshow(image)
    title(['Highlight ', object_name])
    for k = 1: attempts
        sub_struct.name = object_name
        [x, y, train_section, rect] = imcrop();
        if isequal(train_section, [])
            break;
        end
        sub_struct.image = train_section;
        % sub_struct.distributions = apc_get_distributions(train_section);
        sets{k} = sub_struct;
    end

    if nargin == 4
        sets = [existing_sets, sets];
    end
    close all;
end