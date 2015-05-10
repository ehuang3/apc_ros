function [dist] = probability_distribution(set, range)
	% Return the probability distribution for a set
	if nargin < 2
		range = 1:255;
	end
	dist = hist(set, range) ./ numel(set);
end