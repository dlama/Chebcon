function [ p ] = costate( C, u, x, y , odeopts)
%COSTATE Integrates the costate-eq. for a given control and state
% 	dp/dt = -dg/dx - p'*df/dx
% INPUT :
%   C : chebcon
%   u : current control as chebfun
%   x : current state as chebfun columns
%   y : current state as ode45 syntax used in C.state
% OUTPUT :
%   p : current costate


% endcost temporary domain
[a b] = domain(x(:,1));


% operators and functions
D = diff(C.dom);
one = chebfun(1,C.dom);

% integrate costate equations
if size(x,2)==1
    %% 1-D
        
    L = D + diff(C.state(x,u),x);
    % endcost to rbc
    h=C.endcost(x);
    if isa(h,'double')
        % this happens when endcost is constant
        ev=0;
    else
        dh = diff(h,x)*one;
        ev= dh(b);
    end
    L.rbc = ev;
    %integrate
    rhs = -diff(C.cost(x,u),x)*one;
    p = L\rhs;    
else
    %% N-D
        
    % Since we have a coupled system of ODEs here, we will transform
    % the problem from an EVP an IVP  with the transformation T(t)=-t 
    % so that we can use ode113 to solve the problem.
        
    % Domain
    e=C.dom.ends;
    idom=domain(-e(2),-e(1));   
    
    ev=[];
    
    % ODE
    ode = @(t,p)[];
    
    for i=1:size(x,2)
            % initial value
            h=C.endcost(x);
            if isa(h,'double')
                % this happens when endcost is constant
                ev(i)=0;
            else
                dh = diff(h,x(:,i))*one;
                ev(i) = dh(b);
            end
            % dg/dx_i
            dcost = diff(C.cost(x,u),y(i))*one;
            idcost = chebfun(@(t) dcost(-t),idom);
            
            state=C.state(y,u).';
            
            % df/dx_i
            dstate = diff(state,y(i))*one;
            idstate = chebfun(@(t) dstate(-t,1),idom);
            
            % <p,df/dx_i>
            pstate = @(t,p) p(1).*idstate(t);
            for j=2:size(x,2)
                idstate = chebfun(@(t) dstate(-t,j),idom);
                pstate = @(t,p) pstate(t,p) + p(j).*idstate(t);
            end
            
            % d p_j / dt = dg/dx_i + <p,df/dx_i>
            ode = @(t,p) [ode(t,p), idcost(t) + pstate(t,p)]; 
    end
    
    %dp/dt = -dH/dx on inverse domain
    ode = @(t,p) ode(t,p).';
    
    ip = ode113(@(t,p) ode(t,p),idom,ev,odeopts);
    
    % Transform back to original domain
    
    p=[];
    for i=1:size(ip,2)
        p = [p, chebfun(@(t) ip(-t,i),C.dom)];
    end
end
    
end

