function [names] = whatwegot(sets)
    for k = 1:length(sets)
        disp(sets{k}.name)
    end
end