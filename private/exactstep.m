function [val, sigma ] = exactstep(C, u, s ,odeopts)
%EXACTSTEP Returns the optimal sigma
% INPUT :
%   C : chebcon
%   u : current control as chebfun
%   s : search direction as chebfun
% OUTPUT :
%   val : Value of C at u+sigma*s
%   sigam : computed stepsize

% Maximum stepsize here is 1, anything up to inf would be possible aswell
[d]=domain(0,1);


% Chebfun does not construct chebfuns that require solving odes for
% evaluation very well when no fixed discretisation is chosen
n=40;

val=chebfun(@(t) feval(C,u+t.*s,odeopts),d,n,'vectorize');

[val,sigma] = min(val);

end