function [ sigma ] = mixedstep(C, u, s,beta,gamma,odeopts,tol)
%MIXEDSTEP Hybrid steprule with chebfun optimum with fixed x as first guess and then
%   armijo until the armijo condition is satisfied
% INPUT :
%   C : chebcon
%   u : current control as chebfun
%   s : search direction as chebfun
%   x : current state as chebfun columns
%   beta : as in armijo
%   gamma : as in armijo
%   tol : break tolerance when to guess that s is no descent direction at
%   all
% OUTPUT :
%   sigma : computed stepsize

[d]=domain(0,10);
one = chebfun(1,C.dom);

x = ode45(@(t,x) C.state(x,u(t)),C.dom,C.x0);

% transform x from ode syntax so that y(i) = x(:,i)
if size(C.state(one,one)',1)==1
    %% 1-D
    
    y=x;
else
    %% N-D
    
    y=odesyn(x(:,1));
    for i=2:size(C.state(one,one)',2)
        y(i)=x(:,i);
    end
end
    
% integrate costate equation    
p = costate(C,u,x,y,odeopts);

% build the chebfun from the reduced cost-functional
val=chebfun(@(t) reducedcost( C,x,y,p,u+t.*s),d,'vectorize');

% get the minimum
[val,sigma] = min(val);

if sigma == 0
    % when this happens the reduced cost functional is not a good
    % approximation, and we fall back to normal armijo.
    sigma = 1;
end


% go on with armijo
Cu = feval(C,u,odeopts);
prod = gamma * (-s)' * s;

while feval(C,u + sigma * s,odeopts) - Cu > sigma * prod
    sigma = beta * sigma;
    if sigma < tol^3
        sigma=0;
        break
    end
end


end

