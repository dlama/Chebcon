function [sigma] = armijo(C, u, s, gr, beta, gamma,odeopts,tol)
%ARMIJO Armijo-rule 
% INPUT :
%   C : chebcon
%   u : current control as chebfun
%   s : search direction as chebfun
%   gr : gradient
%   beta : constant
%   gamma : constant
% OUTPUT :
%   sigma : computed stepsize



sigma = 1;
Cu = feval(C,u,odeopts);
prod = gamma * gr' * s;


while feval(C,u + sigma * s,odeopts) - Cu > sigma * prod
    sigma = beta * sigma;
    %break cond
    if sigma < tol^3
        sigma=0;
        break
    end
end