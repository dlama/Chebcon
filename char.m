function s = char(A)
% CHAR  Convert Control to pretty-printed string.


if isempty(A)
    s = '   (empty chebcon)';
else
    s = '   operating on chebfuns defined on:';
    s = char(s,['  ' char(A.dom)],' ');
    
    if ~isempty(A.cost)
      s = char(s, '   cost function:');
      for j = 1:length(A.costshow)
          s = char(s,['     ',char(A.costshow{j})]);
      end
    end
    s = char(s,' ');
    
	if ~isempty(A.state)
      s = char(s, '   rhs of the state equation:');
      for j = 1:length(A.stateshow)
          s = char(s,['     ',char(A.stateshow{j})]);
      end
    end
    s = char(s,' ');
    
    if ~isempty(A.endcost)
      s = char(s, '   end-state cost function:');
      for j = 1:length(A.endcostshow)
          s = char(s,['     ',char(A.endcostshow{j})]);
      end
    end
    s = char(s,' ');
    
 	if ~isempty(A.x0)
      s = char(s, '   initial value of the state equation:');
      for j = 1:length(A.x0)
          s = char(s,['     ',num2str(A.x0(j))]);
      end
    end
    s = char(s,' ');
    
end

end  % main function


function s = bc2char(b)

if ~iscell(b), b = {b}; end

s = repmat({'     '},1,length(b));
for k = 1:length(b)
  if isnumeric(b{k})  % number
    s{k} = [s{k}, num2str(b{k})];
  elseif ischar(b{k})  % string
    s{k} = [s{k}, b{k}];
  else  % function
    s{k} = [s{k},char(b{k}),' = 0'];
  end
end

end
