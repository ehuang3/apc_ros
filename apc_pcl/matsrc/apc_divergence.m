function [distance] = divergence(set_1, set_2, method)
	% Determine the Bregman diveregence specified in the string "method" to difference the two distributions
	if nargin < 3
		method = 'js'
		% For APC datasets, this (Jensen-Shannon) worked the best
	end

	if strcmp(method, 'kl')
		% Kullbeck-Leibler divergence
		distance = kl_divergence(set_1, set_2);
	elseif strcmp(method, 'js')
		% Jensen-Shannon (Symmetrized KL-Divergence -- its square-root is metric)
		distance = jensen_shannon(set_1, set_2);
	elseif strcmp(method, 'is')
		% Itakura Saito
		distance = itakura_saito(set_1, set_2);
	elseif strcmp(method, 'jsis')
		% Jensen-Shannon style itakura saito
		distance = jensen_shannon_itakura_saito(set_1, set_2);
	elseif strcmp(method, 'cdf')
		% Cumulative distribution function distance
		distance = cdf_distance(set_1, set_2);
	elseif strcmp(method, 'mahalanobis')
		% Does this actually work?
		distance = mahalanobis(set_1, set_2);
	else
		error(['No such method', method])
	end
end

function [dist] = mahalanobis(set_1, set_2)
	mahal_dist = mahal(set_1', set_2');
	dist = sum((set_1 - repmat(mean(set_2), 1, length(set_1))).^2, 2);
end

function [cdf] = compute_cdf(set)
	cdf = zeros(length(set), 1);
	for k = 1:length(set)
		cdf(k) = sum(set(1:k));
	end
end

function [dist] = cdf_distance(set_1, set_2)
	cdf_1 = cumsum(set_1);
	cdf_2 = cumsum(set_2);

	dist = 0.0;
	alpha = 1.8;
	for k = 1:length(cdf_1)
		single_dist = (abs(cdf_1(k) - cdf_2(k)) ^ alpha);
		dist = dist + single_dist;
	end
end

function [dist] = kl_divergence(P, Q)
	% Compute kullbeck-Leibler divergence
	% Q((Q == 0) & (P ~= 0)) = 0.01;
	log_ = log(P ./ Q);
	log_(isinf(log_)) = 0.0;
	log_(isnan(log_)) = 0.0;
	dist = sum(P .* log_);
end

function [dist] = jensen_shannon(P, Q)
	% Happier K-L Divergence
	M = 0.5 * (P + Q);
	dist = (0.5 * kl_divergence(P, M)) + (0.5 * kl_divergence(Q, M));
end

function [dist] = itakura_saito(P, Q)
	% Itakura-Saito divergence, sucks
	log_ = log(P ./ Q);
	log_(isinf(log_)) = 0.0;
	log_(isnan(log_)) = 0.0;
	div = (P ./ Q);
	div(isinf(div)) = 0.0;
	div(isnan(div)) = 0.0;
	dist = sum(div - log_ - 1.0);
end

function [dist] = jensen_shannon_itakura_saito(P, Q)
	% Itakura Saito in the style of Jensen-Shannon
	M = 0.5 * (P + Q);
	dist = (0.5 * itakura_saito(P, M)) + (0.5 * itakura_saito(Q, M));
end