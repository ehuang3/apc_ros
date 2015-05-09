function [compared] = apc_train_histo(image, test)
    % This is Jacob's implementation of pseudo-DPMS (We are NOT using HoG here)
    figure, imshow(image);
    title('Select the correct region');
    [x, y, train_section, rect] = imcrop();

    ideal_distributions = get_distributions(train_section);

    comparison_func = @(block) compare_to(block.data, ideal_distributions);
    compared = blockproc(test, [50, 50], comparison_func);
    close all;
    pxnorm = sqrt(sum(compared .^ 2, 3));
    figure, imagesc(pxnorm);
    title('Divergence')
end

function [B] = compare_to(block, ideal)
    distributions = get_distributions(block);
    divergences = get_divergences(distributions, ideal);

    if divergences == [0, 0, 0]
        disp('s')
    end
    B = make_sub_image(divergences, size(block));
end

function [subim] =  make_sub_image(divergences, imsz)
    sz = [imsz(1), imsz(2)];
    r_im = repmat(divergences(1), sz);
    g_im = repmat(divergences(2), sz);
    b_im = repmat(divergences(3), sz);
    subim = cat(3, r_im, g_im, b_im);

end

function [distributions] = get_distributions(sub_image)
    ri = double(sub_image(:, :, 1));
    gi = double(sub_image(:, :, 2));
    bi = double(sub_image(:, :, 3));
    r = apc_compute_distribution(ri(:));
    g = apc_compute_distribution(gi(:));
    b = apc_compute_distribution(bi(:));
    distributions = [r; g; b];
    size(distributions)
end

function [divergences] = get_divergences(distributions_1, distributions_2)
    divergences = [ ...
        apc_divergence(distributions_1(1, :), distributions_2(1, :), 'js'), ...
        apc_divergence(distributions_1(2, :), distributions_2(2, :), 'js'), ...
        apc_divergence(distributions_1(3, :), distributions_2(3, :), 'js') ...
    ]
end