function [u,x,val] = min( C, u, tol, odetol, varargin )
%MIN solves the optimal control problem given by C
%   with a steepest descent method starting at u
% INPUT :
%   C : chebcon
%   u : current control
%   tol : tolerance for break condition (||s||<tol)
%   odetol : tolerance passed to underlying odesolver (ode113)
%  optional:
%   stepsize : arm/exact/mixed as string
%   stepmode : current costate as chebfun (cell array of chebfuns)
% OUTPUT :
%   u : control mimimizing the given problem
%   val : Value of C at u

%chebfunoprefs
splitting on
chebfunpref('minsamples',10)
figure(4)
clf
valn=[];
nsn=[];

%odeprefs
odeopts = odeset('abstol',odetol,'reltol',odetol);

%check for empty C
if isempty(C)
    error('CHEBCON:min:empty','Problem is not well defined')
end

% select stepmode and stepsize
if nargin >4
    stepsize=varargin{1};
    if nargin > 5
        stepmode=varargin{2};
    else
        stepmode='grad';
    end
else
    stepsize='mixed';
    stepmode='grad';
end

% one function
one = chebfun(1,C.dom);

figure(1);
clf;

it=0;
while true
    it=it+1;
 
    
    %% State equation
    
    % integrate state equation
    x = ode113(@(t,x) C.state(x,u(t,:)),C.dom,C.x0,odeopts);

    
    % check if ode113 converged
    if domain(u)==domain(x)
        % matlab has no ~= for domains
    else
        error('CHEBCON:min:State','ODE45 failed to converge when integrating the state equation.')
    end
    

    % transform x to ode syntax so that y(i) = x(:,i)
    if size(x,2)==1
        y=x;
    else
        y=odesyn(x(:,1));
        for i=2:size(x,2)
            y(i)=x(:,i);
        end
    end
    
    
    %% Costate Equation
    
    % integrate costate equation    
    p = costate(C,u,x,y,odeopts);
    
    
    %% Search Direction
    
    switch stepmode
        case 'gradient'
            gr=diff(C,u,odeopts,x,y,p);
            s = -gr;
            step='grad';
        case 'newton'            
            % constants for Newton-Condition
            a1=0.9;
            a2=1;
            p=1;
            
            [d,gr,stepmode] = diff2(C,u,x,y,p,stepmode); 
            %Newton-Condition
            if -gr'*d >=min([a1,a2*norm(d).^(p)])*norm(d)^2 && strcmp(stepmode,'newton')
                s=d;
                step='newton';
            else
                s=-gr;
                step='grad';
            end
        otherwise
            gr=diff(C,u,odeopts,x,y,p);
            s = -gr;    
            step='grad';
    end
    
    % break condition
    ns=norm(s,inf);
    if ns < tol
        disp(['Terminating after ',num2str(it-1),' steps with norm(s,inf) =',num2str(ns)]);
        val=feval(C,u,x);
        valn = [valn val];
        nsn = [nsn ns];
        
        % Graphical output
        
        figure(1)
        clf
        set(gcf,'Position',[1 1 600 600])
        hold on
        plot(u,'.-r')
        plot(x,'.-g');
        title('Minimizing x(t) and u(t)');
        xlabel('t');
        legend('u(t)','x(t)',2)
        drawnow
        
        figure(2)
        clf
        set(gcf,'Position',[1 1 600 200])
        semilogy(valn,'x-r')
        title('Convergence of Performance Measure');
        xlabel('Iteration');
        drawnow
        
        
        figure(3)
        clf
        set(gcf,'Position',[1 1 600 200])
        semilogy(nsn,'x-g');
        title('Convergence of ||s||_{inf} ');
        xlabel('Iteration');
        drawnow
        
        break
    end
        

    
    
    %%  Stepsize
    
    % chosing the stepsize 
    switch stepsize
        case 'exact'
            [val,sigma] = exactstep(C,u,s,odeopts);
        case 'arm'
            beta = 0.5;
            gamma = 1e-3;
            sigma = armijo(C,u,s,gr,beta,gamma,odeopts,tol);
        case 'pw'
            beta = 0.5;
            gamma = 1e-3;
            theta= 0.6;
            sigma = powell(C,u,s,gr,beta,gamma,theta,odeopts,tol);
        case 'mixed'
            beta = 0.5;
            gamma = 1e-3;
            sigma = mixedstep(C,u,s,beta,gamma,odeopts,tol);
        otherwise
            beta = 0.5;
            gamma = 1e-3;
            sigma = mixedstep(C,u,s,beta,gamma,odeopts,tol);
    end
            
    % break condition, when we no longer have a descent direction
    % this mostly happens due to rounding errors when very close to
    % optimal solution
    if sigma == 0
        val = feval(C,u,x);
        disp(['Iteration: ',num2str(it),' Step: ',step,' sigma=',num2str(sigma),' Value=',num2str(val),' ||s||_inf=',num2str(norm(s,inf))])
        warning('CHEBCON:min:Gradient','Search direction is no longer a descent direction')
        return
    end
    
    %  compute new control
    un=[];
    for i=1:size(u,2)
        un=[un,chebfun(u(:,i) + sigma * s(:,i),C.dom)];
    end
    u=un;
    val = feval(C,u,odeopts);
    valn = [valn val];
    nsn = [nsn ns];
    
    %% Text output
    
    figure(4)
    hold on
    set(gcf,'Position',[1 1 600 600])
    
    plot(u,'.-r')
    plot(x,'.-g');
    title('Current x(t) and u(t)');
    xlabel('t');
    legend('u(t)','x(t)',2)
    drawnow
    
    disp(['Iteration: ',num2str(it),' Step: ',step,' sigma=',num2str(sigma),' Value=',num2str(val),' ||s||_inf=',num2str(norm(s,inf))]) 
    for i=1:size(u,2)
        disp(['Length(u',num2str(i),'): ',num2str(length(u(:,i)))])
    end
    for i=1:size(x,2)
        disp(['Length(x',num2str(i),'): ',num2str(length(x(:,i)))])
    end
end

