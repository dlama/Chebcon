function varargout = subsasgn(f,index,varargin)
% SUBSASGN   Modify a chebcon.

idx = index(1).subs;
vin = varargin{:};
switch index(1).type
    case '.'
        varargout = {set(f,idx,vin)};
    otherwise
        error('CHEBCON:subsasgn:indexType',['Unexpected index.type of ' index(1).type]);
end