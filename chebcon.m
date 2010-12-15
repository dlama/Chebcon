function C = chebcon(D,Cost,F,X0,varargin)
% CHEBCON  Control constructor for the chebfun system.
% C = chebcon(D,C,F,X0,E);
% INPUT :
%   D : domain of the control
%   C : anononymous function of x and u giving the cost function of the control problem. 
%           in higher dimensions x_i is called by x(:,i)
%   F : anononymous function of x and u giving the rhs of the state equation 
%           in higher dimensions when f is a vector valued function
%           the correct syntax is @(x,u) [f1,f2,...].' . Here x_i must be
%           called with x(i).
%   X0 : scalar giving the initial value for the state equation
%  optional
%   E : anonynous function giving the endcost value, evaluated at x and u
%   at the end of the domain
%           in higher dimensions x_i is called as x(:,i) here

persistent default_C
if isnumeric(default_C)
    default_C = Func_ini;
    default_C = class(default_C,'chebcon');
end
C= default_C;

C.dom = D;
C = set(C,'cost',Cost);
C = set(C,'state',F);
C = set(C,'x0',X0);

if nargin > 4
    C = set(C,'endcost',varargin{1});
end


end

function C = Func_ini()
C = struct([]);
C(1).dom =[];
C(1).cost = [];
C(1).costshow = [];
C(1).endcost = @(x) 0;
C(1).endcostshow = {char(@(x) 0)};
C(1).state = [];
C(1).stateshow = [];
C(1).x0 = [];
end