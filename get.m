function val = get(C, propName)
% GET   Get chebfunct properties.


switch propName
    case 'dom'
        val = C.dom;
    case 'cost'
        val = C.cost;
    case 'endcost'
        val = C.endcost;
    case 'state'
        val = C.state;
    case 'x0'
        val = C.x0;
    otherwise
        error('CHEBCON:get:propname',[propName,' is not a valid Control property'])
end
