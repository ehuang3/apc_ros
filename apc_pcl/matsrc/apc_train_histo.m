function [compared] = apc_train_histo(image, test)
    % This is Jacob's implementation of pseudo-DPMS (We are NOT using HoG here)
    % Try: HSV (Just H) distance
    % More clever weighted distance -- frustrating to do matrix multiplication on image-style vector sets
    % comp = apc_train_histo(train_2, test);
    
    figure, imshow(image);
    title('Select the correct region');
    [x, y, train_section, rect] = imcrop();

    ideal_distributions = apc_get_distributions(train_section);

    comparison_func = @(block) compare_to(block.data, ideal_distributions);
    compared = blockproc(test, [50, 50], comparison_func);
    % close all;
    figure,imshow(test)
    title('target image')
    pxnorm = max(compared, [], 3); % Max and min show similar performance
    figure, imagesc(pxnorm);
    title('Divergence')
end

function [B] = compare_to(block, ideal)
    distributions = apc_get_distributions(block);
    divergences = get_divergences(distributions, ideal);
    B = make_sub_image(divergences, size(block));
end

function [subim] =  make_sub_image(divergences, imsz)
    sz = [imsz(1), imsz(2)];
    r_im = repmat(divergences(1), sz);
    g_im = repmat(divergences(2), sz);
    b_im = repmat(divergences(3), sz);
    subim = cat(3, r_im, g_im, b_im);
end