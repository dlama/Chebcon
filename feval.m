function c = feval(C,u,varargin)
% FEVAL Evaluate the chebcon.
% INPUT :
%   C : chebcon
%   u : current control
%  optional :
%   odeopts : odeopts for the solver of the state equation
%   or
%   x : current state as chebfun columns
% OUTPUT :
%   gr : chebfun representation of the Gradient

% incomplete Controls have value 0
if isempty(C)
   c=0;
   return
end

% check if additional arguments are specified
if nargin > 2
    % check if varargin{1} is a chebfun or an odeset
    if isa(varargin{1},'chebfun')
        x = varargin{1};
    else
        odeopts=varargin{1};
        x = ode113(@(t,x) C.state(x,u(t,:)),C.dom,C.x0,odeopts);
    end
else
    tol = 10^-8;
    odeopts = odeset('abstol',tol,'reltol',tol);
    x = ode113(@(t,x) C.state(x,u(t,:)),C.dom,C.x0,odeopts);
end
% accept x as input to speed up calculation if x is already computed.
% integrate state equation, chose odesolver depending on tolerance

% check if ode113 converged
if domain(u)==domain(x)
    % matlab has no ~= for domain
else
    error('CHEBCON:min:State','ODE45 failed to converge when integrating the state equation.')
end

% scalar case
h=chebfun(C.endcost(x),C.dom);

e=C.dom.ends;
c=sum(C.cost(x,u))+h(e(2));


end