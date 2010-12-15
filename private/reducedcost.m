function [ val ] = reducedcost(C,x,y,p,u)
%REDUCEDCOST Reduced cost functional
% INPUT :
%   C : chebcon
%   x : current state as chebfun columns
%   y : current state as ode45 syntax used in C.state
%   p : current costate as chebfun (cell array of chebfuns)
%   u : current control
% OUTPUT :
%   val : value

% init functions

dx=diff(x);

if size(p,2)==1
    %% 1-D
    
    % placeholder
else
    %% N-D
    
    fstate = chebfun(0,C.dom);
    state=C.state(y,u).';
    % inner product
    for i=1:size(p,2)
        %<p,C.state - dx/dt>
        fstate = fstate + p(:,i).*(state(:,i) - dx(:,i));
        
    end
end


%endvalue
h=chebfun(C.endcost(x),C.dom);
e=C.dom.ends;
ev=h(e(2));

if size(x,2) >1
    val=sum(C.cost(x,u) + fstate) + ev;
else
    val=sum(C.cost(x,u) + p.*(C.state(y,u)-diff(x))) + ev;
end



end

