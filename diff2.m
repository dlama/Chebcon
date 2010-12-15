function [d,gr,stepmode] = diff2(C,u,x,y,p,stepmode)
%% THIS SHOULD NOT BE USED
% DIFF2  Computes and solves the Newton equation for Control Problems.
% INPUT :
%   C : chebcon
%   u : current control
%   x : current state as chebfun columns
%   y : current state as ode45 syntax used in C.state
%   p : current costate as chebfun (cell array of chebfuns)
% OUTPUT :
%   gr : chebfun representation of the Gradient
%   d : solution of the Newton Equation
one = chebfun(1,C.dom);


% Hamiltonian operator
if size(p,2)==1
    %% 1-D
    
    H = @(x, u, p) C.cost(x, u) + p.*C.state(y,u);
else
    %% N-D
    
    H = @(x, u, p) C.cost(x, u);
    state=C.state(y,u).';
    %for higher dimensions add p'*state(x,u), this is the scalar product
    for i=1:length(p)
        H = @(x, u, p) H(x,u,p) + p{i}.*state(:,i);
    end
end

% differentiate the operator 
DH = diff(H(x,u,p),u);
% chebfun representation
gr=DH*one;

% second variation
DDH=diff(DH(x,u,p),u);

try
    d=DDH\gr;
catch
    warning('CHEBCON:min:diff2:NewtonEq','Not possible to solve Newton-equation, changing stepmode to gradient')
    stepmode='gradient';
    d=chebfun(1,C.dom);
end
    
end
