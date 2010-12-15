function display(A)
% DISPLAY Pretty-print a chebcon.
% DISPLAY is called automatically when a statement that results in a chebcon
% output is not terminated with a semicolon.


loose = ~isequal(get(0,'FormatSpacing'),'compact');
if loose, disp(' '), end
disp([inputname(1) ' = chebcon']);
if loose, disp(' '), end
s = char(A);
if ~loose   
  s( all(isspace(s),2), : ) = [];  % remove blank lines
end
disp(s)
if loose, disp(' '), end

end

