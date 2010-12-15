function C = set(C,varargin)
% SET Set chebcon properties.
%

propertyArgIn = varargin;
while length(propertyArgIn) >= 2,
    prop = propertyArgIn{1};
    val = propertyArgIn{2};
    propertyArgIn = propertyArgIn(3:end);
    switch prop
        case 'dom'
            C.dom = val;
        case 'cost'
            C.cost = val;
            if ~iscell(val)
              C.costshow = {char(val)}; 
            else
              C.costshow = cellfun(@char,val,'uniform',false);
            end
        case 'endcost'
            C.endcost = val;
            if ~iscell(val)
              C.endcostshow = {char(val)}; 
            else
              C.endcostshow = cellfun(@char,val,'uniform',false);
            end
        case 'state'
            C.state = val;
            if ~iscell(val)
              C.stateshow = {char(val)}; 
            else
              C.stateshow = cellfun(@char,val,'uniform',false);
            end
        case 'x0'
            C.x0 = val;
        otherwise
            error('CHEBCON:set:unknownprop','Unknown chebcon property')
    end
end
end
