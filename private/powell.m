function [ sigma ] = powell(C,u,s,gr,beta,gamma,theta,odeopts,tol)
%POWELL Powell-Wolfe rule
% INPUT :
%   C : chebcon
%   u : current control as chebfun
%   s : search direction as chebfun
%   gr : gradient
%   beta : as in armijo
%   gamma : as in armijo
%   theta : constant
% OUTPUT :
%   sigma : computed stepsize
sigma = 1;
ibeta=1/beta;

Cu=feval(C,u,odeopts);
prod = gr'*s;

% sigma_+ and sigma_-
if feval(C,u + sigma * s,odeopts) - Cu > sigma*gamma*prod
    sigmam=beta*sigma;
    while feval(C,u + sigmam * s,odeopts) - Cu > sigmam*gamma*prod
        sigmam = beta * sigmam;
        %break cond
        if sigmam < tol^3
            sigma=0;
            break
        end
    end
    sigmap=ibeta*sigmam;
else
    sigmap=ibeta*sigma;
    while feval(C,u + sigmap * s,odeopts) - Cu <= sigmap*gamma*prod
        sigmap = ibeta*sigmap;
    end
    sigmam=beta*sigmap;
end

% bisection for powell-wolfe
sigma=sigmam;
grs=diff(C,u+sigma*s,odeopts,tol);
while grs'*s <= theta*prod
    sigma = (sigmam+sigmap)/2;
    if feval(C,u + sigma * s,odeopts) - Cu <= sigma*gamma*prod
        sigmam=sigma;
        % gradient only needs update if sigma=sigmam changes
        grs=diff(C,u+sigma*s,odeopts);
    else
        sigmap=sigma;
    end
end
sigma=sigmam;
end

