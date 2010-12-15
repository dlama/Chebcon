function e = isempty(A)
% ISEMPTY   True for empty chebcon. Domain, Cost, State and x0 need to be specified.

e = isempty(A.dom) && isempty(A.cost) && isempty(A.state) && isempty(A.x0);

end