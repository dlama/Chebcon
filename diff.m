function [gr] = diff(C,u,varargin)
% DIFF  Computes the Gradient of C.
%	Computes the Hamilton operator of the System, differentiates it
%   and gets the chebfun representation
% INPUT :
%   C : chebcon
%   u : current control
%  optional
%   odeopts : settings for ode113/ode45
%   x : current state as chebfun columns
%   y : current state as odesyn
%   p : current costate as chebfun columns
% OUTPUT :
%   gr : chebfun representation of the Gradient

one = chebfun(1,C.dom);

if nargin < 3
    tol = 1e-8;
    odeopts = odeset('abstol',tol,'reltol',tol);
else
    odeopts= varargin{1};
end


%check wether x,y,p are given, if not compute
if nargin < 6
    % integrate state equation
    x = ode113(@(t,x) C.state(x,u(t)),C.dom,C.x0,odeopts);
    
    % transform x from ode syntax so that y(i) = x(:,i)
    if size(C.state(one,one)',1)==1
        %%1-D
        
        y=x;
    else
        %%N-D
        
        y=odesyn(x(:,1));
        for i=2:size(C.state(one,one)',2)
            y(i)=x(:,i);
        end
    end
    
    % integrate costate equation    
    % 	dp/dt = -dg/dx - p'*df/dx
    p = costate(C,u,x,y,odeopts);
else
    x=varargin{2};
    y=varargin{3};
    p=varargin{4};
end

% Hamiltonian operator
if size(p,2)==1
    %% 1-D
    
    H = @(x, u, p) C.cost(x, u) + p.*C.state(y,u);
else
    %% N-D
    
    H = @(x, u, p) C.cost(x, u);
    state=C.state(y,u).';
    % p'*state(x,u) :
    for i=1:size(p,2)
        H = @(x, u, p) H(x,u,p) + p(:,i).*state(:,i);
    end
end

% differentiate the operator dH/du

gr=[];
for i=1:size(u,2)
    DH = diff(H(x,u,p),u(:,i));
    gr=[gr,DH*one];
end



end
