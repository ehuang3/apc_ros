function [result] = chunk_proc(image, chunksize, func)
    % [result] = chunk_proc(image, chunksize, func)
    % Process the image in chunks using func and return a cell array of the results
    % chunksize must be a single scalar

    cells = {};
    im_size = [size(image, 1), size(image, 2)];
    % iterations = floor(min(im_size) / chunksize);
    iterations_x = im_size(1) / chunksize;
    iterations_y = im_size(2) / chunksize;
    cellnum = 1;

    for x = 0:iterations_x - 1
        for y = 0:iterations_y - 1
            x_pos = (x * chunksize) + 1;
            y_pos = (y * chunksize) + 1;
            sub_image = image(x_pos: x_pos + chunksize, y_pos: y_pos + chunksize, :);
            func_result = func(sub_image);

            % If our function did not find anything meaningful, don't add anything to the cellarray
            if size(func_result) ~= size([])
                cells = [cells, func_result];
                cellnum = cellnum + 1;
            end
        end
    end
    result = cells;
end