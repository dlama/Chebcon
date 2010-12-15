function [ val ] = endcost( cost, y, x, i, b )
%ENDCOST use to compute partial dervitives
% INPUT :
%   cost : anon function for the endcost
%   y : state vector
%   x : variable to differentiate for
%   i : x_i for differentiation
%   b : end of domain to evaluate other states
% OUTPUT :
%   val : value of the endcost

for j=1:size(y,2)
    if j ~=i
        z=y(:,j);
        v{j}=z(b);
    else    
        v{j}=x;
    end
end

val = cost(v);

end

