function [separated] = apc_pre_train(sets, targets)
    separated = {};
    for k = 1:length(sets)
        if ismember(sets{k}.name, targets)
            separated = [separated, sets{k}];
        end
    end
end